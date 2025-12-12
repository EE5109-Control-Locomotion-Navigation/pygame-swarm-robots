"""Base task class for swarm behaviors."""
import pygame

class BaseTask:
    """Base class for all swarm tasks."""
    
    def __init__(self, name):
        self.name = name
    
    def update_boid(self, boid, boids, obstacles, params, **kwargs):
        """Update a single boid based on task behavior.
        
        Args:
            boid: The boid to update
            boids: List of all boids
            obstacles: List of all obstacles
            params: List of behavior parameters
            **kwargs: Additional task-specific parameters
        """
        pass
    
    def update_task(self, boids, obstacles, **kwargs):
        """Update task-specific state (called once per frame).
        
        Args:
            boids: List of all boids
            obstacles: List of all obstacles
            **kwargs: Additional task-specific parameters
        """
        pass
    
    def handle_mouse_click(self, pos, button, **kwargs):
        """Handle mouse click events for this task.
        
        Args:
            pos: Mouse position (x, y)
            button: Mouse button (1=left, 3=right)
            **kwargs: Additional task-specific parameters
            
        Returns:
            dict: Updated task state or None
        """
        return None
    
    def draw(self, screen, **kwargs):
        """Draw task-specific visuals.
        
        Args:
            screen: Pygame surface to draw on
            **kwargs: Additional task-specific parameters
        """
        pass
    
    def get_help_text(self):
        """Get help text for this task."""
        return ""

