"""Simulation manager for swarm simulator."""
import pygame
from boid import Boid
from obstacle import Obstacle
from ui import Slider, ToggleButton
from config import *

class Simulation:
    """Manages simulation state and updates."""
    
    def __init__(self, screen, font):
        self.screen = screen
        self.font = font
        self.boids = [Boid() for _ in range(INITIAL_BOID_COUNT)]
        self.obstacles = [Obstacle(x, y, r) for x, y, r in INITIAL_OBSTACLES]
        self.sliders = self._create_sliders()
        self.current_task = None
        self.task_index = 0
        
        # Debug toggles
        self.show_velocity = False
        self.show_sensors = False
        self.show_neighbors = False
        
        # Field visualization toggles (positioned after MazeAvoid slider)
        self.visualize_fields = ToggleButton(SIM_WIDTH + 30, 600, "Visualize Fields", False)
        self.visualize_potential = ToggleButton(SIM_WIDTH + 30, 625, "Visualize Potential", False)
        
        # Map editor
        from map_editor import MapEditor
        self.map_editor = MapEditor()
    
    def _create_sliders(self):
        """Create UI sliders."""
        sliders = []
        for config in SLIDER_CONFIGS:
            slider = Slider(
                SIM_WIDTH + 30,
                config["y"],
                240,
                config["label"],
                config["min"],
                config["max"],
                config["start"]
            )
            sliders.append(slider)
        return sliders
    
    def set_task(self, task):
        """Set the current task."""
        self.current_task = task
        if hasattr(task, 'initialize'):
            task.initialize(SIM_WIDTH, HEIGHT)
    
    def get_all_obstacles(self):
        """Get all obstacles including task-specific ones (like maze walls)."""
        all_obs = list(self.obstacles)
        if self.current_task and hasattr(self.current_task, 'get_maze_obstacles'):
            all_obs.extend(self.current_task.get_maze_obstacles())
        return all_obs
    
    def update(self):
        """Update simulation state."""
        # Don't update simulation when in map editor mode
        if self.map_editor.active:
            return
        
        # Get all obstacles (regular + task-specific)
        all_obstacles = self.get_all_obstacles()
        
        # Update task-specific state
        if self.current_task:
            goal_strength = self.sliders[6].value if len(self.sliders) > 6 else 1.5
            damping = self.sliders[7].value if len(self.sliders) > 7 else 0.95
            self.current_task.update_task(
                self.boids, 
                all_obstacles,
                goal_strength=goal_strength,
                damping=damping
            )
        
        # Update boids
        update_params = [s.value for s in self.sliders[:6]]
        
        # For maze navigation, use slider-controlled avoidance strength for maze obstacles
        goal_pos_for_avoidance = None
        if self.current_task and self.current_task.name == "Maze Navigation":
            maze_obstacle_scale = self.sliders[8].value if len(self.sliders) > 8 else 0.05
            # Pass goal position to reduce avoidance when goal is nearby
            if hasattr(self.current_task, 'maze_goal') and self.current_task.maze_goal:
                goal_pos_for_avoidance = self.current_task.maze_goal
        else:
            maze_obstacle_scale = 1.0
        
        for b in self.boids:
            b.update(self.boids, all_obstacles, update_params, maze_obstacle_scale, goal_pos_for_avoidance)
            
            # Apply task-specific behavior
            if self.current_task:
                goal_strength = self.sliders[6].value if len(self.sliders) > 6 else 1.5
                sensor_len = self.sliders[5].value if len(self.sliders) > 5 else 100
                avoid_weight = self.sliders[3].value if len(self.sliders) > 3 else 2.0
                max_speed = self.sliders[4].value if len(self.sliders) > 4 else 3.0
                self.current_task.update_boid(
                    b, 
                    self.boids, 
                    all_obstacles, 
                    update_params,
                    goal_strength=goal_strength,
                    sensor_len=sensor_len,
                    avoid_weight=avoid_weight,
                    max_speed=max_speed
                )
    
    def add_boid(self):
        """Add a new boid."""
        self.boids.append(Boid())
    
    def add_obstacle(self, x, y):
        """Add a new obstacle."""
        if x < SIM_WIDTH:
            self.obstacles.append(Obstacle(x, y, 60))
    
    def remove_obstacle_at(self, x, y):
        """Remove obstacle at position if clicked."""
        click_pos = pygame.Vector2(x, y)
        for obs in self.obstacles:
            if obs.is_rect:
                # Check if click is inside rectangular obstacle
                rect = obs.get_rect()
                if rect.collidepoint(x, y):
                    self.obstacles.remove(obs)
                    return True
            else:
                # Check if click is within circular obstacle
                if click_pos.distance_to(obs.pos) < obs.r:
                    self.obstacles.remove(obs)
                    return True
        return False
    
    def save_params(self):
        """Save slider parameters to file."""
        from config import PARAM_FILE
        import json
        data = {s.label: s.value for s in self.sliders}
        with open(PARAM_FILE, "w") as f:
            json.dump(data, f)
    
    def load_params(self):
        """Load slider parameters from file."""
        from config import PARAM_FILE
        import json
        try:
            with open(PARAM_FILE) as f:
                data = json.load(f)
            for slider in self.sliders:
                if slider.label in data:
                    slider.value = float(data[slider.label])
        except:
            pass

