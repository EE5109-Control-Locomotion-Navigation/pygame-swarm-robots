"""Goal-seeking task."""
import pygame
from .base_task import BaseTask

class GoalSeeking(BaseTask):
    """Task where boids seek a goal position."""
    
    def __init__(self):
        super().__init__("Goal-Seeking")
        self.goal_pos = None
    
    def initialize(self, sim_width, sim_height):
        """Initialize goal position."""
        self.goal_pos = pygame.Vector2(sim_width // 2, sim_height // 2)
    
    def update_boid(self, boid, boids, obstacles, params, goal_strength=1.5, **kwargs):
        """Apply goal-seeking force to boid."""
        if self.goal_pos is None:
            return
        
        task_force = (self.goal_pos - boid.pos).normalize() * goal_strength
        boid.vel += task_force * 0.05
    
    def handle_mouse_click(self, pos, button, **kwargs):
        """Set goal position on left click."""
        if button == 1:  # Left click
            self.goal_pos = pygame.Vector2(pos)
            return {'goal_pos': self.goal_pos}
        return None
    
    def draw(self, screen, **kwargs):
        """Draw goal marker."""
        if self.goal_pos:
            pygame.draw.circle(screen, (50, 255, 50), 
                             (int(self.goal_pos.x), int(self.goal_pos.y)), 10)
    
    def get_help_text(self):
        return "Click left in sim area to move goal"

