"""General swarm task - basic boid behavior."""
from .base_task import BaseTask

class GeneralSwarm(BaseTask):
    """General swarm task with basic boid behaviors."""
    
    def __init__(self):
        super().__init__("General Swarm")
    
    def get_help_text(self):
        return "Basic swarm behavior with alignment, cohesion, and separation"

