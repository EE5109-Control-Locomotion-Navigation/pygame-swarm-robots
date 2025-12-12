"""Foraging task."""
import pygame
import random
from .base_task import BaseTask
from resource import Resource

class Foraging(BaseTask):
    """Task where boids collect resources and return them to a home base."""
    
    def __init__(self):
        super().__init__("Foraging")
        self.resources = []
        self.forage_home = None
        self.sim_width = 0
        self.sim_height = 0
        self.resource_count = 20
    
    def initialize(self, sim_width, sim_height):
        """Initialize resources and home base."""
        self.sim_width = sim_width
        self.sim_height = sim_height
        self.forage_home = pygame.Vector2(100, sim_height // 2)
        self._create_resources()
    
    def _create_resources(self):
        """Create initial resources."""
        self.resources = []
        for _ in range(self.resource_count):
            pos = (
                random.randint(50, self.sim_width - 50),
                random.randint(50, self.sim_height - 50)
            )
            self.resources.append(Resource(pos))
    
    def reset_resources(self):
        """Reset all resources to uncollected state."""
        for r in self.resources:
            r.collected = False
    
    def update_boid(self, boid, boids, obstacles, params, goal_strength=1.5, **kwargs):
        """Apply foraging behavior to boid."""
        # Initialize carrying attribute if not present
        if not hasattr(boid, 'carrying'):
            boid.carrying = None
        
        # Check if boid is carrying a resource
        if boid.carrying is None:
            # Seek nearest uncollected resource
            nearest = None
            min_dist = float('inf')
            
            for r in self.resources:
                if not r.collected:
                    d = (r.pos - boid.pos).length()
                    if d < min_dist:
                        min_dist = d
                        nearest = r
            
            if nearest:
                # Move toward resource
                direction = nearest.pos - boid.pos
                if direction.length() > 0:
                    boid.vel += direction.normalize() * 0.1 * goal_strength
                
                # Collect resource when close enough
                if min_dist < 10:
                    boid.carrying = nearest
                    nearest.collected = True
        else:
            # Move toward home base
            direction = self.forage_home - boid.pos
            if direction.length() > 0:
                boid.vel += direction.normalize() * 0.1 * goal_strength
            
            # Deposit resource when at home
            if (self.forage_home - boid.pos).length() < 10:
                # Reset resource position and mark as uncollected
                boid.carrying.pos = self.forage_home.copy()
                boid.carrying.collected = False
                boid.carrying = None
    
    def handle_mouse_click(self, pos, button, **kwargs):
        """Set home base position on left click."""
        if button == 1:  # Left click
            self.forage_home = pygame.Vector2(pos)
            return {'forage_home': self.forage_home}
        return None
    
    def draw(self, screen, **kwargs):
        """Draw resources and home base."""
        # Draw resources
        for r in self.resources:
            r.draw(screen)
        
        # Draw home base
        if self.forage_home:
            pygame.draw.circle(screen, (50, 255, 50),
                             (int(self.forage_home.x), int(self.forage_home.y)), 12)
    
    def get_help_text(self):
        return "Click left in sim area to move home base"
    
    def get_resources(self):
        """Get list of resources."""
        return self.resources

