"""Configuration settings for the swarm simulator."""
import pygame

# Display settings
WIDTH, HEIGHT = 1600, 900
SIM_WIDTH = 1100
UI_WIDTH = WIDTH - SIM_WIDTH

# Colors
BG_COLOR = (25, 25, 30)
SIM_BG_COLOR = (40, 40, 50)
UI_BG_COLOR = (55, 55, 65)
TEXT_COLOR = (230, 230, 230)
TEXT_COLOR_DIM = (220, 220, 220)

# Simulation settings
INITIAL_BOID_COUNT = 60
INITIAL_OBSTACLES = [
    (500, 300, 60),
    (830, 550, 80)
]

# Slider settings
SLIDER_CONFIGS = [
    {"label": "Alignment", "min": 0, "max": 5, "start": 1.0, "y": 60},
    {"label": "Cohesion", "min": 0, "max": 5, "start": 1.0, "y": 120},
    {"label": "Separation", "min": 0, "max": 5, "start": 1.2, "y": 180},
    {"label": "Avoid", "min": 0, "max": 5, "start": 2.0, "y": 240},
    {"label": "MaxSpeed", "min": 1, "max": 8, "start": 3.0, "y": 330},
    {"label": "SensorLen", "min": 20, "max": 200, "start": 100, "y": 390},
    {"label": "GoalStrength", "min": 0, "max": 5, "start": 1.5, "y": 450},
    {"label": "Damping", "min": 0.8, "max": 1.0, "start": 0.95, "y": 510},
    {"label": "MazeAvoid", "min": 0.0, "max": 0.2, "start": 0.05, "y": 570},
]

# UI settings
UI_TITLE_Y = 10
UI_TASK_Y = 22
UI_HELP_START_Y = 680
UI_HELP_LINE_SPACING = 25

# File settings
PARAM_FILE = "params.json"

# Task list
TASKS = ["General Swarm", "Goal-Seeking", "Area Coverage", "Object Transport", 
         "Maze Navigation", "Foraging"]

