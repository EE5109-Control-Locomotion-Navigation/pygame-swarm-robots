"""Area coverage task with heatmap."""
import pygame
from .base_task import BaseTask

class AreaCoverage(BaseTask):
    """Task where boids cover the area efficiently."""
    
    def __init__(self):
        super().__init__("Area Coverage")
        self.grid_size = 50
        self.visitation = None
        self.grid_cols = 0
        self.grid_rows = 0
    
    def initialize(self, sim_width, sim_height):
        """Initialize coverage grid."""
        self.grid_cols = sim_width // self.grid_size + 1
        self.grid_rows = sim_height // self.grid_size + 1
        self.visitation = [[0.0 for _ in range(self.grid_rows)] 
                          for _ in range(self.grid_cols)]
    
    def update_task(self, boids, obstacles, **kwargs):
        """Update visitation grid and apply decay."""
        # Update visitation based on boid positions
        for b in boids:
            gx = int(b.pos.x // self.grid_size)
            gy = int(b.pos.y // self.grid_size)
            if 0 <= gx < self.grid_cols and 0 <= gy < self.grid_rows:
                self.visitation[gx][gy] += 1
        
        # Decay for smooth fading
        for i in range(self.grid_cols):
            for j in range(self.grid_rows):
                self.visitation[i][j] *= 0.995
    
    def update_boid(self, boid, boids, obstacles, params, **kwargs):
        """Direct boid to least visited area."""
        if self.visitation is None:
            return
        
        gx = int(boid.pos.x // self.grid_size)
        gy = int(boid.pos.y // self.grid_size)
        min_visits = float('inf')
        target_cell = (gx, gy)
        
        # Find least visited cell in 3x3 neighborhood
        for i in range(max(0, gx-1), min(self.grid_cols, gx+2)):
            for j in range(max(0, gy-1), min(self.grid_rows, gy+2)):
                if self.visitation[i][j] < min_visits:
                    min_visits = self.visitation[i][j]
                    target_cell = (i, j)
        
        target_pos = pygame.Vector2(
            target_cell[0] * self.grid_size + self.grid_size / 2,
            target_cell[1] * self.grid_size + self.grid_size / 2
        )
        task_force = (target_pos - boid.pos).normalize() * 1.5
        boid.vel += task_force * 0.05
    
    def draw_background(self, screen, **kwargs):
        """Draw coverage heatmap as background."""
        if self.visitation is None:
            return
        
        for i in range(self.grid_cols):
            for j in range(self.grid_rows):
                alpha = min(180, int(self.visitation[i][j] * 20))
                if alpha > 0:
                    s = pygame.Surface((self.grid_size, self.grid_size), pygame.SRCALPHA)
                    s.fill((50, 200, 50, alpha))
                    screen.blit(s, (i * self.grid_size, j * self.grid_size))
    
    def draw(self, screen, **kwargs):
        """Draw task overlays (none for area coverage)."""
        pass
    
    def get_help_text(self):
        return "Boids explore and cover the area efficiently"

