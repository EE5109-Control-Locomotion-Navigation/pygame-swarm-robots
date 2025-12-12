"""Object transport task."""
import pygame
from .base_task import BaseTask
from objectTransport import TransportObject

class ObjectTransport(BaseTask):
    """Task where boids cooperatively transport an object to a goal."""
    
    def __init__(self):
        super().__init__("Object Transport")
        self.transport_obj = None
        self.transport_goal = None
        self.sim_width = 0
        self.sim_height = 0
    
    def initialize(self, sim_width, sim_height):
        """Initialize transport object and goal."""
        self.sim_width = sim_width
        self.sim_height = sim_height
        self.transport_obj = TransportObject((sim_width // 2, sim_height // 2), radius=20)
        self.transport_goal = pygame.Vector2(sim_width - 100, sim_height // 2)
    
    def update_boid(self, boid, boids, obstacles, params, **kwargs):
        """Apply boid-object interaction forces."""
        if self.transport_obj is None:
            return
        
        direction_to_obj = self.transport_obj.pos - boid.pos
        dist = direction_to_obj.length()
        if dist < self.transport_obj.r + 50 and dist > 0:
            # Apply force toward object center
            force = direction_to_obj.normalize() * 0.5
            boid.vel += force
    
    def update_task(self, boids, obstacles, damping=0.95, **kwargs):
        """Update transport object with cooperative push."""
        if self.transport_obj is None:
            return
        
        # Update damping from slider
        self.transport_obj.damping = damping
        
        # Apply cooperative push if multiple boids are near
        nearby_boids = [b for b in boids 
                       if (b.pos - self.transport_obj.pos).length() < self.transport_obj.r + 50]
        
        if len(nearby_boids) >= 2:  # Require at least 2 boids to move object
            goal_dir = self.transport_goal - self.transport_obj.pos
            goal_dist = goal_dir.length()
            if goal_dist > 0:
                push_dir = goal_dir.normalize() * 0.3
                self.transport_obj.vel += push_dir
        
        # Update transport object
        self.transport_obj.update()
        
        # Keep object within bounds
        self.transport_obj.pos.x = max(
            self.transport_obj.r, 
            min(self.sim_width - self.transport_obj.r, self.transport_obj.pos.x)
        )
        self.transport_obj.pos.y = max(
            self.transport_obj.r,
            min(self.sim_height - self.transport_obj.r, self.transport_obj.pos.y)
        )
    
    def handle_mouse_click(self, pos, button, **kwargs):
        """Set transport goal on left click."""
        if button == 1 and self.transport_goal:  # Left click
            self.transport_goal = pygame.Vector2(pos)
            return {'transport_goal': self.transport_goal}
        return None
    
    def draw(self, screen, **kwargs):
        """Draw transport object and goal."""
        if self.transport_obj:
            self.transport_obj.draw(screen)
        if self.transport_goal:
            pygame.draw.circle(screen, (50, 255, 50),
                             (int(self.transport_goal.x), int(self.transport_goal.y)), 10)
    
    def get_help_text(self):
        return "Click left in sim area to move transport goal"

