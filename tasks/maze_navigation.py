"""Maze navigation task."""
import pygame
import random
from .base_task import BaseTask
from obstacle import Obstacle

# Grid size for path memory (same as area coverage)
GRID_SIZE = 50

class MazeNavigation(BaseTask):
    """Task where boids navigate through a maze toward a goal."""
    
    def __init__(self):
        super().__init__("Maze Navigation")
        self.maze_goal = None
        self.maze_obstacles = []
        self.sim_width = 0
        self.sim_height = 0
        self.path_memory = None
        self.grid_cols = 0
        self.grid_rows = 0
    
    def initialize(self, sim_width, sim_height):
        """Initialize maze goal and create maze obstacles."""
        self.sim_width = sim_width
        self.sim_height = sim_height
        self.maze_goal = pygame.Vector2(sim_width - 100, sim_height // 2)
        self._create_maze()
        
        # Initialize path memory grid
        self.grid_cols = sim_width // GRID_SIZE + 1
        self.grid_rows = sim_height // GRID_SIZE + 1
        self.path_memory = [[0.0 for _ in range(self.grid_rows)] for _ in range(self.grid_cols)]
    
    def _create_maze(self):
        """Create maze walls using rectangular obstacles with more navigable space."""
        self.maze_obstacles = [
            # Horizontal walls - spaced further apart for navigation
            Obstacle(300, 200, 0, width=180, height=15),
            Obstacle(650, 400, 0, width=200, height=15),
            Obstacle(900, 250, 0, width=150, height=15),
            Obstacle(200, 600, 0, width=160, height=15),
            Obstacle(850, 700, 0, width=200, height=15),
            
            # Vertical walls - shorter and with gaps
            Obstacle(400, 100, 0, width=15, height=120),
            Obstacle(700, 320, 0, width=15, height=150),
            Obstacle(500, 520, 0, width=15, height=160),
            Obstacle(1000, 350, 0, width=15, height=200),
        ]
    
    def get_maze_obstacles(self):
        """Get list of maze obstacles."""
        return self.maze_obstacles
    
    def update_task(self, boids, obstacles, **kwargs):
        """Update path memory grid with boid positions and apply decay."""
        if self.path_memory is None:
            return
        
        # Update path memory based on boid positions
        for b in boids:
            gx = int(b.pos.x // GRID_SIZE)
            gy = int(b.pos.y // GRID_SIZE)
            if 0 <= gx < self.grid_cols and 0 <= gy < self.grid_rows:
                self.path_memory[gx][gy] += 0.01  # Accumulate memory
        
        # Decay path memory
        for i in range(self.grid_cols):
            for j in range(self.grid_rows):
                self.path_memory[i][j] *= 0.995
    
    def update_boid(self, boid, boids, obstacles, params, goal_strength=1.5, sensor_len=100, avoid_weight=2.0, max_speed=3.0, **kwargs):
        """Apply maze navigation behavior with path memory bias and scaled obstacle avoidance."""
        if self.maze_goal is None or self.path_memory is None:
            return
        
        # Note: Maze obstacles are now handled by the regular boid.update() with reduced strength
        # This ensures they're avoided but not too aggressively
        
        # Attraction to goal with swarm memory bias
        direction_to_goal = self.maze_goal - boid.pos
        dist = direction_to_goal.length()
        
        if dist > 5:
            # Calculate memory bias from nearby cells
            gx = int(boid.pos.x // GRID_SIZE)
            gy = int(boid.pos.y // GRID_SIZE)
            memory_bias = pygame.Vector2(0, 0)
            
            # Check neighboring cells for memory values
            for i in range(max(0, gx-1), min(self.grid_cols, gx+2)):
                for j in range(max(0, gy-1), min(self.grid_rows, gy+2)):
                    if self.path_memory[i][j] > 0:
                        cell_pos = pygame.Vector2(
                            i * GRID_SIZE + GRID_SIZE / 2,
                            j * GRID_SIZE + GRID_SIZE / 2
                        )
                        memory_bias += (cell_pos - boid.pos) * self.path_memory[i][j]
            
            # Normalize memory bias if it has magnitude
            if memory_bias.length() > 0:
                memory_bias = memory_bias.normalize() * 0.1
            
            # Scale goal force inversely with distance - stronger when closer
            # This allows boids to overcome avoidance when goal is near walls
            distance_factor = min(1.0, 50.0 / max(dist, 5.0))  # Stronger when < 50 pixels away
            goal_force_magnitude = goal_strength * (0.05 + 0.15 * distance_factor)  # 0.05 to 0.2 range
            
            # Combine goal-seeking with memory bias
            goal_force = direction_to_goal.normalize() * goal_force_magnitude
            task_force = goal_force + memory_bias
            boid.vel += task_force
        
        # Small random exploration to help navigate complex mazes
        random_force = pygame.Vector2(
            random.uniform(-0.1, 0.1),
            random.uniform(-0.1, 0.1)
        )
        boid.vel += random_force
    
    def handle_mouse_click(self, pos, button, **kwargs):
        """Set maze goal on left click."""
        if button == 1:  # Left click
            self.maze_goal = pygame.Vector2(pos)
            return {'maze_goal': self.maze_goal}
        return None
    
    def draw(self, screen, **kwargs):
        """Draw maze goal (obstacles are drawn separately by renderer)."""
        # Draw goal
        if self.maze_goal:
            pygame.draw.circle(screen, (50, 255, 50),
                             (int(self.maze_goal.x), int(self.maze_goal.y)), 10)
    
    def get_help_text(self):
        return "Click left in sim area to move maze goal"

