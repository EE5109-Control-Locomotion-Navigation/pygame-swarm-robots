"""Resource class for foraging task."""
import pygame
import random

class Resource:
    """A resource that can be collected by boids."""
    
    def __init__(self, pos):
        self.pos = pygame.Vector2(pos)
        self.collected = False
    
    def draw(self, surf):
        """Draw the resource."""
        color = (200, 200, 50) if not self.collected else (100, 100, 100)
        pygame.draw.circle(surf, color, (int(self.pos.x), int(self.pos.y)), 8)

