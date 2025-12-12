# Swarm Simulator

A modular Python-based swarm robotics simulator featuring multiple task types, interactive controls, and real-time visualization.

## Features

### Task Types

1. **General Swarm** - Basic flocking behavior with alignment, cohesion, and separation
2. **Goal-Seeking** - Boids navigate toward a user-defined goal position
3. **Area Coverage** - Swarm efficiently explores and covers the simulation area with heatmap visualization
4. **Object Transport** - Cooperative transport of objects requiring multiple boids
5. **Maze Navigation** - Navigate through mazes with path memory and adaptive obstacle avoidance
6. **Foraging** - Collect resources and return them to a home base

### Interactive Features

- **Real-time Parameter Control** - Adjustable sliders for behavior parameters
- **Map Editor** - Create and save custom obstacle maps as JSON files
- **Field Visualization** - Visualize goal attraction and obstacle avoidance vectors
- **Potential Field Visualization** - Grid-based visualization of obstacle repulsion fields
- **Debug Modes** - Toggle velocity vectors, sensor visualization, and neighbor detection

## Requirements

- Python 3.8+
- pygame

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd EE5109
```

2. Create a virtual environment (recommended):
```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install pygame
```

## Usage

Run the simulator:
```bash
python swarmSim.py
```

### Controls

#### Task Selection
- **1-6** - Switch between tasks (General Swarm, Goal-Seeking, Area Coverage, Object Transport, Maze Navigation, Foraging)

#### Debug Toggles
- **V** - Toggle velocity vectors
- **S** - Toggle sensor visualization
- **N** - Toggle neighbor detection circles

#### Simulation Control
- **B** - Add a new boid
- **O** - Add obstacle at mouse position
- **Right-click** - Remove obstacle at mouse position

#### Map Editor (Press M to enter)
- **M** - Toggle map editor mode
- **T** - Toggle between circular and rectangular obstacles
- **Left-click** - Place obstacle
- **Right-click** - Remove obstacle
- **+/-** - Adjust circular obstacle size
- **Q/A** - Adjust rectangular obstacle width
- **W/X** - Adjust rectangular obstacle height
- **S** - Save map to `map.json`
- **L** - Load map from `map.json`

#### Parameters
- **P** - Save slider parameters to `params.json`
- **L** - Load slider parameters from `params.json`

#### Visualization
- **Visualize Fields** toggle - Show goal attraction and avoidance vectors
- **Visualize Potential** toggle - Show potential field grid

### Task-Specific Controls

#### Goal-Seeking / Maze Navigation
- **Left-click** in simulation area - Move goal position

#### Object Transport
- **Left-click** in simulation area - Move transport goal

#### Foraging
- **Left-click** in simulation area - Move home base

## Project Structure

```
EE5109/
├── swarmSim.py          # Main entry point
├── simulation.py        # Simulation state management
├── event_handler.py     # Input handling
├── renderer.py          # Rendering system
├── config.py            # Configuration settings
├── map_editor.py        # Map creation and editing
├── boid.py              # Boid behavior and physics
├── obstacle.py          # Obstacle representation
├── resource.py          # Resource for foraging
├── objectTransport.py   # Transport object
├── ui.py                # UI components (sliders, buttons)
├── tasks/               # Task implementations
│   ├── base_task.py
│   ├── general_swarm.py
│   ├── goal_seeking.py
│   ├── area_coverage.py
│   ├── object_transport.py
│   ├── maze_navigation.py
│   └── foraging.py
└── backup/              # Backup files
```

## Parameters

### Behavior Sliders

- **Alignment** (0-5) - Tendency to align with neighbors
- **Cohesion** (0-5) - Tendency to move toward neighbors
- **Separation** (0-5) - Tendency to avoid neighbors
- **Avoid** (0-5) - Obstacle avoidance strength
- **MaxSpeed** (1-8) - Maximum boid velocity
- **SensorLen** (20-200) - Sensor range for obstacle detection
- **GoalStrength** (0-5) - Goal-seeking force strength
- **Damping** (0.8-1.0) - Object transport damping factor
- **MazeAvoid** (0.0-0.2) - Maze obstacle avoidance strength (Maze Navigation only)

## File Formats

### Map Files (`map.json`)
```json
{
  "obstacles": [
    {
      "x": 500.0,
      "y": 300.0,
      "r": 60.0,
      "is_rect": false
    },
    {
      "x": 300.0,
      "y": 200.0,
      "r": 0.0,
      "width": 200.0,
      "height": 20.0,
      "is_rect": true
    }
  ]
}
```

### Parameter Files (`params.json`)
Saves all slider values for easy restoration of settings.

## Architecture

The simulator uses a modular architecture:

- **Tasks** - Each task is a self-contained class inheriting from `BaseTask`
- **Simulation** - Manages boids, obstacles, and task state
- **Renderer** - Handles all drawing operations
- **Event Handler** - Processes user input
- **Map Editor** - Separate system for creating custom maps

This design makes it easy to add new tasks or modify existing behavior.

## License

MIT License - see LICENSE file for details

## Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

