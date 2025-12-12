"""Main entry point for swarm simulator."""
import pygame
from pygame.locals import *
from ui import init_ui_font
from config import WIDTH, HEIGHT, SIM_WIDTH
from simulation import Simulation
from event_handler import EventHandler
from renderer import Renderer
from tasks import GeneralSwarm, GoalSeeking, AreaCoverage, ObjectTransport, MazeNavigation, Foraging

def main():
    """Main simulation loop."""
    # Initialize pygame
    pygame.init()
    init_ui_font()
    
    # Create display
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Swarm Simulator - Task Menu")
    font = pygame.font.SysFont("consolas", 16)
    
    # Initialize tasks
    tasks = [
        GeneralSwarm(),
        GoalSeeking(),
        AreaCoverage(),
        ObjectTransport(),
        MazeNavigation(),
        Foraging(),
    ]
    
    # Initialize simulation
    simulation = Simulation(screen, font)
    simulation.set_task(tasks[0])  # Start with General Swarm
    
    # Initialize event handler
    event_handler = EventHandler(simulation, tasks)
    
    # Initialize renderer
    renderer = Renderer(screen, font)
    
    # Load saved parameters
    simulation.load_params()
    
    # Main loop
    clock = pygame.time.Clock()
    while event_handler.running:
        dt = clock.tick(60)
        
        # Handle events
        if not event_handler.handle_events():
            break
        
        # Update simulation
        simulation.update()
        
        # Render
        renderer.clear()
        renderer.draw_simulation(simulation)
        renderer.draw_ui(simulation)
        renderer.flip()
    
    # Save parameters on exit
    simulation.save_params()
    pygame.quit()

if __name__ == "__main__":
    main()
