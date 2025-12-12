"""Event handling for swarm simulator."""
import pygame
from pygame.locals import *
from config import *

class EventHandler:
    """Handles user input events."""
    
    def __init__(self, simulation, tasks):
        self.simulation = simulation
        self.tasks = tasks
        self.running = True
    
    def handle_events(self):
        """Process all pending events."""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
                return False
            
            # Handle slider events
            for slider in self.simulation.sliders:
                slider.handle(event)
            
            # Handle toggle button events
            if hasattr(self.simulation, 'visualize_fields'):
                self.simulation.visualize_fields.handle(event)
            if hasattr(self.simulation, 'visualize_potential'):
                self.simulation.visualize_potential.handle(event)
            
            # Handle keyboard events
            if event.type == KEYDOWN:
                self._handle_keydown(event)
            
            # Handle mouse events
            if event.type == MOUSEBUTTONDOWN:
                self._handle_mouse(event)
        
        return True
    
    def _handle_keydown(self, event):
        """Handle keyboard input."""
        # Map editor mode
        if event.key == K_m:
            self.simulation.map_editor.toggle()
        elif self.simulation.map_editor.active:
            # Map editor controls
            if event.key == K_t:
                self.simulation.map_editor.toggle_obstacle_type()
            elif event.key == K_EQUALS or event.key == K_PLUS:  # + key
                if self.simulation.map_editor.obstacle_type == "circular":
                    self.simulation.map_editor.increase_circular_size()
                else:
                    self.simulation.map_editor.increase_rect_width()
            elif event.key == K_MINUS:  # - key
                if self.simulation.map_editor.obstacle_type == "circular":
                    self.simulation.map_editor.decrease_circular_size()
                else:
                    self.simulation.map_editor.decrease_rect_width()
            elif event.key == K_q:  # Q for width increase (rectangular)
                if self.simulation.map_editor.obstacle_type == "rectangular":
                    self.simulation.map_editor.increase_rect_width()
            elif event.key == K_a:  # A for width decrease (rectangular)
                if self.simulation.map_editor.obstacle_type == "rectangular":
                    self.simulation.map_editor.decrease_rect_width()
            elif event.key == K_w:  # W for height increase (rectangular)
                if self.simulation.map_editor.obstacle_type == "rectangular":
                    self.simulation.map_editor.increase_rect_height()
            elif event.key == K_x:  # X for height decrease (rectangular) - changed from S to avoid conflict
                if self.simulation.map_editor.obstacle_type == "rectangular":
                    self.simulation.map_editor.decrease_rect_height()
            elif event.key == K_s:  # S to save map
                filename = "map.json"
                if self.simulation.map_editor.save_map(self.simulation.obstacles, filename):
                    print(f"Map saved to {filename}")
            elif event.key == K_l:
                # Load map
                filename = "map.json"
                loaded_obs = self.simulation.map_editor.load_map(filename)
                if loaded_obs:
                    self.simulation.obstacles = loaded_obs
                    print(f"Map loaded from {filename}")
            return  # Don't process other keys in editor mode
        
        # Debug toggles
        if event.key == K_v:
            self.simulation.show_velocity = not self.simulation.show_velocity
        elif event.key == K_s:
            self.simulation.show_sensors = not self.simulation.show_sensors
        elif event.key == K_n:
            self.simulation.show_neighbors = not self.simulation.show_neighbors
        
        # Boid/obstacle management
        elif event.key == K_b:
            self.simulation.add_boid()
        elif event.key == K_o:
            mx, my = pygame.mouse.get_pos()
            self.simulation.add_obstacle(mx, my)
        
        # Parameter save/load
        elif event.key == K_l:
            self._load_params()
        elif event.key == K_p:
            self._save_params()
        
        # Task selection (1-6)
        elif event.key == K_1 and len(TASKS) > 0:
            self._switch_task(0)
        elif event.key == K_2 and len(TASKS) > 1:
            self._switch_task(1)
        elif event.key == K_3 and len(TASKS) > 2:
            self._switch_task(2)
        elif event.key == K_4 and len(TASKS) > 3:
            self._switch_task(3)
        elif event.key == K_5 and len(TASKS) > 4:
            self._switch_task(4)
        elif event.key == K_6 and len(TASKS) > 5:
            self._switch_task(5)
    
    def _handle_mouse(self, event):
        """Handle mouse clicks."""
        mx, my = event.pos
        
        # Map editor mode
        if self.simulation.map_editor.active:
            if event.button == 1:  # Left click - place obstacle
                if mx < SIM_WIDTH:
                    obs = self.simulation.map_editor.create_obstacle(mx, my)
                    self.simulation.obstacles.append(obs)
            elif event.button == 3:  # Right click - remove obstacle
                self.simulation.remove_obstacle_at(mx, my)
            return
        
        if event.button == 1:  # Left click
            # Let task handle the click
            if self.simulation.current_task:
                self.simulation.current_task.handle_mouse_click((mx, my), 1)
        
        elif event.button == 3:  # Right click
            # Remove obstacle
            self.simulation.remove_obstacle_at(mx, my)
    
    def _switch_task(self, index):
        """Switch to a different task."""
        if 0 <= index < len(self.tasks):
            self.simulation.set_task(self.tasks[index])
            self.simulation.task_index = index
    
    def _save_params(self):
        """Save slider parameters to file."""
        self.simulation.save_params()
    
    def _load_params(self):
        """Load slider parameters from file."""
        self.simulation.load_params()

