"""BLE transport for the XGO-lite: bleak client running on a background
asyncio thread with a synchronous API for callers.

- `write(addr, data)` queues a register write; latest-wins coalescing for the
  high-rate MOTOR_ANGLE stream so control lag never accumulates.
- `read(addr, length, timeout)` performs a request/response round-trip and
  blocks the caller until the matching notification arrives.
"""

import asyncio
import concurrent.futures
import threading
import time

from bleak import BleakClient, BleakScanner

from .protocol import (
    PacketParser,
    build_read_packet,
    build_write_packet,
)

CHAR_WRITE = "0000fff2-0000-1000-8000-00805f9b34fb"
CHAR_NOTIFY = "0000fff1-0000-1000-8000-00805f9b34fb"


class BleTransport:
    """Synchronous wrapper around a bleak client on its own event loop."""

    def __init__(self, address: str,
                 char_write: str = CHAR_WRITE,
                 char_notify: str = CHAR_NOTIFY,
                 min_write_interval: float = 0.002,
                 on_notify=None):
        self.address = address
        self.char_write = char_write
        self.char_notify = char_notify
        self.min_write_interval = min_write_interval
        # Optional extra callback: on_notify(addr:int, payload:bytes)
        self.on_notify = on_notify

        self._loop = asyncio.new_event_loop()
        self._client = None
        self._write_q = None
        self._req_q = None
        self._parser = PacketParser()
        self._pending = {}          # addr -> list[concurrent.futures.Future]
        self._pending_lock = threading.Lock()
        self._connected = threading.Event()
        self._closed = False
        self._last_write = 0.0

        self._thread = threading.Thread(
            target=self._run, name="xgo-ble", daemon=True
        )

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self, timeout: float = 60.0):
        """Start the BLE thread and wait for the first connection."""
        self._thread.start()
        if not self._connected.wait(timeout):
            raise TimeoutError(
                f"BLE connection to {self.address} timed out")

    def close(self):
        self._closed = True
        if self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._shutdown(), self._loop)
        self._thread.join(timeout=5.0)

    async def _shutdown(self):
        if self._client and self._client.is_connected:
            await self._client.disconnect()

    def is_connected(self) -> bool:
        return self._connected.is_set()

    # ------------------------------------------------------------------
    # asyncio side
    # ------------------------------------------------------------------
    def _run(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main())

    async def _main(self):
        # control writes: latest-wins mailbox (stale commands are dropped)
        self._write_q = asyncio.Queue(maxsize=4)
        # read requests: FIFO, never dropped silently
        self._req_q = asyncio.Queue(maxsize=64)
        while not self._closed:
            device = await BleakScanner.find_device_by_address(
                self.address, timeout=10.0)
            if device is None:
                await asyncio.sleep(2.0)
                continue
            try:
                async with BleakClient(self.address, timeout=15.0) as client:
                    self._client = client
                    await client.start_notify(
                        self.char_notify, self._on_notify)
                    self._connected.set()
                    while self._connected.is_set() and not self._closed:
                        # read requests first — small and state matters
                        while not self._req_q.empty():
                            await client.write_gatt_char(
                                self.char_write,
                                self._req_q.get_nowait(),
                                response=False)
                            await self._pace()
                        try:
                            packet = await asyncio.wait_for(
                                self._write_q.get(), timeout=0.02)
                            while not self._write_q.empty():
                                packet = self._write_q.get_nowait()
                            await client.write_gatt_char(
                                self.char_write, packet, response=False)
                            await self._pace()
                        except asyncio.TimeoutError:
                            continue
            except Exception:
                pass
            finally:
                self._connected.clear()
                self._client = None
                self._fail_pending()
            if not self._closed:
                await asyncio.sleep(2.0)

    async def _pace(self):
        dt = self.min_write_interval - (time.monotonic() - self._last_write)
        if dt > 0:
            await asyncio.sleep(dt)
        self._last_write = time.monotonic()

    async def _enqueue_write(self, packet):
        """Latest-wins: if the mailbox is full, drop the oldest."""
        if self._write_q.full():
            try:
                self._write_q.get_nowait()
            except asyncio.QueueEmpty:
                pass
        self._write_q.put_nowait(packet)

    async def _enqueue_req(self, packet):
        try:
            self._req_q.put_nowait(packet)
        except asyncio.QueueFull:
            pass

    def _on_notify(self, _sender, data: bytearray):
        for addr, payload in self._parser.feed(data):
            with self._pending_lock:
                futures = self._pending.pop(addr, [])
            for fut in futures:
                if not fut.done():
                    fut.set_result(payload)
            if self.on_notify is not None:
                self.on_notify(addr, payload)

    def _fail_pending(self):
        with self._pending_lock:
            pending = self._pending
            self._pending = {}
        for futures in pending.values():
            for fut in futures:
                if not fut.done():
                    fut.set_exception(ConnectionError("BLE disconnected"))

    # ------------------------------------------------------------------
    # synchronous API
    # ------------------------------------------------------------------
    def write(self, addr: int, data):
        """Queue a register write (non-blocking)."""
        if not self._connected.is_set():
            return
        packet = build_write_packet(addr, data)
        asyncio.run_coroutine_threadsafe(
            self._enqueue_write(packet), self._loop)

    def read(self, addr: int, length: int, timeout: float = 1.0):
        """Blocking register read; returns the payload bytes."""
        if not self._connected.is_set():
            raise ConnectionError("BLE not connected")
        fut = concurrent.futures.Future()
        with self._pending_lock:
            self._pending.setdefault(addr, []).append(fut)
        packet = build_read_packet(addr, length)
        asyncio.run_coroutine_threadsafe(
            self._enqueue_req(packet), self._loop)
        try:
            return fut.result(timeout)
        finally:
            with self._pending_lock:
                lst = self._pending.get(addr)
                if lst and fut in lst:
                    lst.remove(fut)

    def request_async(self, addr: int, length: int):
        """Send a read request without waiting for the response.

        The response arrives via the `on_notify` callback (addr-keyed),
        which is enough for periodic state polling.
        """
        if not self._connected.is_set():
            return
        packet = build_read_packet(addr, length)
        asyncio.run_coroutine_threadsafe(
            self._enqueue_req(packet), self._loop)
