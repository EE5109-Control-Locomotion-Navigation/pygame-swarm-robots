import pygame

FONT = None

def init_ui_font():
    global FONT
    if FONT is None:
        FONT = pygame.font.SysFont("consolas", 16)

def clamp(x, lo, hi):
    return max(lo, min(x, hi))

class Slider:
    def __init__(self, x, y, w, label, min_val, max_val, start):
        self.rect = pygame.Rect(x, y, w, 20)
        self.min = min_val
        self.max = max_val
        self.value = start
        self.label = label
        self.dragging = False

    def draw(self, surf):
        pygame.draw.rect(surf, (80,80,80), self.rect)
        kx = self.rect.x + (self.value-self.min)/(self.max-self.min)*self.rect.w
        pygame.draw.circle(surf, (220,220,220),(int(kx), self.rect.centery),8)
        txt = FONT.render(f"{self.label}: {self.value:.2f}",True,(230,230,230))
        surf.blit(txt,(self.rect.x, self.rect.y-18))

    def handle(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.dragging = True
        if event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        if event.type == pygame.MOUSEMOTION and self.dragging:
            rel = (event.pos[0] - self.rect.x)/self.rect.w
            rel = clamp(rel,0,1)
            self.value = self.min + rel*(self.max-self.min)

class ToggleButton:
    def __init__(self, x, y, label, start_state=False):
        self.x = x
        self.y = y
        self.label = label
        self.state = start_state
        self.value = start_state  # Alias for compatibility
    
    def draw(self, surf):
        txt = FONT.render(f"{self.label}: {'ON' if self.state else 'OFF'}", True, (230, 230, 230))
        surf.blit(txt, (self.x, self.y))
    
    def toggle(self):
        self.state = not self.state
        self.value = self.state
    
    def handle(self, event):
        """Handle mouse click events to toggle the button."""
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                # Check if click is within button area (approximate)
                mx, my = event.pos
                if self.x <= mx <= self.x + 200 and self.y <= my <= self.y + 20:
                    self.toggle()
