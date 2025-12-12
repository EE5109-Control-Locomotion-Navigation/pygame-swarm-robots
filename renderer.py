"""Rendering for swarm simulator."""
import pygame
from config import *
from boid import Boid

class Renderer:
    """Handles all rendering operations."""
    
    def __init__(self, screen, font):
        self.screen = screen
        self.font = font
    
    def clear(self):
        """Clear the screen."""
        self.screen.fill(BG_COLOR)
        pygame.draw.rect(self.screen, SIM_BG_COLOR, (0, 0, SIM_WIDTH, HEIGHT))
    
    def draw_simulation(self, simulation):
        """Draw simulation elements."""
        # Draw task-specific background visuals (heatmap, etc.)
        if simulation.current_task and hasattr(simulation.current_task, 'draw_background'):
            simulation.current_task.draw_background(self.screen)
        
        # Draw task-specific obstacles (maze walls, etc.) before regular obstacles
        if simulation.current_task and hasattr(simulation.current_task, 'get_maze_obstacles'):
            for obs in simulation.current_task.get_maze_obstacles():
                obs.draw(self.screen)
        
        # Draw regular obstacles
        for obs in simulation.obstacles:
            obs.draw(self.screen)
        
        # Draw boids (only if not in map editor mode)
        if not simulation.map_editor.active:
            for b in simulation.boids:
                b.draw(
                    self.screen,
                    simulation.show_velocity,
                    simulation.show_sensors,
                    simulation.show_neighbors
                )
        
        # Draw task-specific overlays (goals, etc.)
        if simulation.current_task:
            simulation.current_task.draw(self.screen)
        
        # Draw field visualization if enabled
        if hasattr(simulation, 'visualize_fields') and simulation.visualize_fields.state:
            self._draw_field_visualization(simulation)
        
        # Draw potential field visualization if enabled
        if hasattr(simulation, 'visualize_potential') and simulation.visualize_potential.state:
            self._draw_potential_field(simulation)
        
        # Draw map editor UI overlay
        if simulation.map_editor.active:
            simulation.map_editor.draw_editor_ui(self.screen, self.font)
    
    def _draw_field_visualization(self, simulation):
        """Draw vector fields for goal attraction and obstacle avoidance."""
        if not simulation.current_task:
            return
        
        sensor_len = simulation.sliders[5].value if len(simulation.sliders) > 5 else 100
        
        for b in simulation.boids:
            # Draw goal attraction vector
            task_name = simulation.current_task.name
            if task_name == "Goal-Seeking":
                goal_pos = None
                if hasattr(simulation.current_task, 'goal_pos'):
                    goal_pos = simulation.current_task.goal_pos
                if goal_pos:
                    direction = goal_pos - b.pos
                    if direction.length() > 0:
                        pygame.draw.line(
                            self.screen, (0, 255, 255),
                            b.pos,
                            b.pos + direction.normalize() * 30,
                            2
                        )
            
            elif task_name == "Maze Navigation":
                if hasattr(simulation.current_task, 'maze_goal') and simulation.current_task.maze_goal:
                    direction = simulation.current_task.maze_goal - b.pos
                    if direction.length() > 0:
                        pygame.draw.line(
                            self.screen, (0, 255, 255),
                            b.pos,
                            b.pos + direction.normalize() * 30,
                            2
                        )
            
            elif task_name == "Foraging":
                # Draw foraging vectors
                carrying = getattr(b, 'carrying', None)
                if carrying is None:
                    # Draw line to nearest uncollected resource
                    if hasattr(simulation.current_task, 'get_resources'):
                        resources = simulation.current_task.get_resources()
                        nearest = None
                        min_dist = float('inf')
                        for r in resources:
                            if not r.collected:
                                d = (r.pos - b.pos).length()
                                if d < min_dist:
                                    min_dist = d
                                    nearest = r
                        if nearest:
                            pygame.draw.line(
                                self.screen, (0, 255, 255),
                                b.pos, nearest.pos, 2
                            )
                else:
                    # Draw line to home base
                    if hasattr(simulation.current_task, 'forage_home') and simulation.current_task.forage_home:
                        pygame.draw.line(
                            self.screen, (255, 0, 255),
                            b.pos, simulation.current_task.forage_home, 2
                        )
            
            # Draw avoidance vectors for obstacles
            all_obstacles = simulation.obstacles
            if hasattr(simulation.current_task, 'get_maze_obstacles'):
                all_obstacles = list(all_obstacles) + simulation.current_task.get_maze_obstacles()
            
            for obs in all_obstacles:
                if obs.is_rect:
                    # For rectangular obstacles, find closest point
                    rect = obs.get_rect()
                    closest_x = max(rect.left, min(b.pos.x, rect.right))
                    closest_y = max(rect.top, min(b.pos.y, rect.bottom))
                    closest_point = pygame.Vector2(closest_x, closest_y)
                    dir_vec = b.pos - closest_point
                else:
                    dir_vec = b.pos - obs.pos
                
                if dir_vec.length() < sensor_len and dir_vec.length() > 0:
                    pygame.draw.line(
                        self.screen, (255, 0, 0),
                        b.pos,
                        b.pos + dir_vec.normalize() * 30,
                        2
                    )
    
    def _draw_potential_field(self, simulation):
        """Draw potential field visualization as a grid of vectors."""
        
        # Get all obstacles
        all_obstacles = simulation.obstacles
        if simulation.current_task and hasattr(simulation.current_task, 'get_maze_obstacles'):
            all_obstacles = list(all_obstacles) + simulation.current_task.get_maze_obstacles()
        
        # Get maze obstacle scale from slider and goal position
        goal_pos_for_viz = None
        if simulation.current_task and simulation.current_task.name == "Maze Navigation":
            maze_obstacle_scale = simulation.sliders[8].value if len(simulation.sliders) > 8 else 0.05
            if hasattr(simulation.current_task, 'maze_goal') and simulation.current_task.maze_goal:
                goal_pos_for_viz = simulation.current_task.maze_goal
        else:
            maze_obstacle_scale = 1.0
        
        # Sample grid points
        grid_size = 40  # Sample every 40 pixels
        for x in range(0, SIM_WIDTH, grid_size):
            for y in range(0, HEIGHT, grid_size):
                sample_pos = pygame.Vector2(x, y)
                
                # Create a temporary boid to calculate potential field
                temp_boid = Boid()
                temp_boid.pos = sample_pos
                
                # Calculate potential field force at this point
                force = temp_boid.potential_field(all_obstacles, maze_obstacle_scale, goal_pos_for_viz)
                
                # Draw vector if force is significant
                if force.length() > 0.1:
                    # Scale vector for visibility
                    force_length = min(force.length() * 0.5, grid_size * 0.4)
                    if force_length > 2:
                        end_pos = sample_pos + force.normalize() * force_length
                        
                        # Color based on force strength (red = strong, yellow = weak)
                        force_strength = min(force.length() / 50.0, 1.0)
                        color_r = int(255 * force_strength)
                        color_g = int(255 * (1 - force_strength * 0.5))
                        color_b = 0
                        
                        # Draw arrow
                        pygame.draw.line(
                            self.screen, (color_r, color_g, color_b),
                            sample_pos,
                            end_pos,
                            1
                        )
                        
                        # Draw small circle at sample point
                        pygame.draw.circle(self.screen, (color_r, color_g, color_b), 
                                         (int(sample_pos.x), int(sample_pos.y)), 2)
    
    def draw_ui(self, simulation):
        """Draw UI panel."""
        # UI background
        pygame.draw.rect(self.screen, UI_BG_COLOR, (SIM_WIDTH, 0, UI_WIDTH, HEIGHT))
        
        # Title
        self.screen.blit(
            self.font.render("Swarm Controls", True, TEXT_COLOR),
            (SIM_WIDTH + 20, UI_TITLE_Y)
        )
        
        # Task name
        task_name = simulation.current_task.name if simulation.current_task else "None"
        self.screen.blit(
            self.font.render(f"Task: {task_name} (1-6 to change)", True, TEXT_COLOR_DIM),
            (SIM_WIDTH + 20, UI_TASK_Y)
        )
        
        # Sliders
        for slider in simulation.sliders:
            slider.draw(self.screen)
        
        # Toggle buttons for field visualization
        if hasattr(simulation, 'visualize_fields'):
            simulation.visualize_fields.draw(self.screen)
        if hasattr(simulation, 'visualize_potential'):
            simulation.visualize_potential.draw(self.screen)
        
        # Help text
        self._draw_help_text(simulation)
    
    def _draw_help_text(self, simulation):
        """Draw help text at bottom of UI."""
        y = UI_HELP_START_Y
        lines = [
            "Toggles: v=vel, s=sensors, n=neighbors",
            "B=add boid  O=add obstacle",
            "Right-click obstacle to delete",
            "P=save params  L=load params",
            "M=map editor",
        ]
        
        for line in lines:
            self.screen.blit(
                self.font.render(line, True, TEXT_COLOR_DIM),
                (SIM_WIDTH + 20, y)
            )
            y += UI_HELP_LINE_SPACING
        
        # Task-specific help
        if simulation.current_task:
            help_text = simulation.current_task.get_help_text()
            if help_text:
                self.screen.blit(
                    self.font.render(help_text, True, TEXT_COLOR_DIM),
                    (SIM_WIDTH + 20, y)
                )
    
    def flip(self):
        """Update the display."""
        pygame.display.flip()

