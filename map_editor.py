"""Map editor for creating and saving obstacle maps."""
import pygame
import json
from obstacle import Obstacle
from config import SIM_WIDTH, HEIGHT, PARAM_FILE

class MapEditor:
    """Map editor for placing obstacles and saving maps."""
    
    def __init__(self):
        self.active = False
        self.obstacle_type = "circular"  # "circular" or "rectangular"
        self.obstacle_size = 60
        self.obstacle_size_min = 10
        self.obstacle_size_max = 200
        self.rect_width = 200
        self.rect_height = 20
        self.rect_size_min = 10
        self.rect_size_max = 500
    
    def toggle(self):
        """Toggle map editor on/off."""
        self.active = not self.active
        return self.active
    
    def set_obstacle_type(self, obs_type):
        """Set obstacle type: 'circular' or 'rectangular'."""
        if obs_type in ["circular", "rectangular"]:
            self.obstacle_type = obs_type
    
    def toggle_obstacle_type(self):
        """Toggle between circular and rectangular."""
        self.obstacle_type = "rectangular" if self.obstacle_type == "circular" else "circular"
    
    def increase_circular_size(self, step=5):
        """Increase circular obstacle size."""
        self.obstacle_size = min(self.obstacle_size + step, self.obstacle_size_max)
    
    def decrease_circular_size(self, step=5):
        """Decrease circular obstacle size."""
        self.obstacle_size = max(self.obstacle_size - step, self.obstacle_size_min)
    
    def increase_rect_width(self, step=10):
        """Increase rectangular obstacle width."""
        self.rect_width = min(self.rect_width + step, self.rect_size_max)
    
    def decrease_rect_width(self, step=10):
        """Decrease rectangular obstacle width."""
        self.rect_width = max(self.rect_width - step, self.rect_size_min)
    
    def increase_rect_height(self, step=10):
        """Increase rectangular obstacle height."""
        self.rect_height = min(self.rect_height + step, self.rect_size_max)
    
    def decrease_rect_height(self, step=10):
        """Decrease rectangular obstacle height."""
        self.rect_height = max(self.rect_height - step, self.rect_size_min)
    
    def create_obstacle(self, x, y):
        """Create an obstacle at the given position."""
        if self.obstacle_type == "circular":
            return Obstacle(x, y, self.obstacle_size)
        else:
            return Obstacle(x, y, 0, width=self.rect_width, height=self.rect_height)
    
    def save_map(self, obstacles, filename="map.json"):
        """Save obstacles to a JSON file."""
        map_data = {
            "obstacles": []
        }
        
        for obs in obstacles:
            obs_data = {
                "x": float(obs.pos.x),
                "y": float(obs.pos.y),
                "r": float(obs.r),
            }
            if obs.is_rect:
                obs_data["width"] = float(obs.width)
                obs_data["height"] = float(obs.height)
                obs_data["is_rect"] = True
            else:
                obs_data["is_rect"] = False
            
            map_data["obstacles"].append(obs_data)
        
        try:
            with open(filename, "w") as f:
                json.dump(map_data, f, indent=2)
            return True
        except Exception as e:
            print(f"Error saving map: {e}")
            return False
    
    def load_map(self, filename="map.json"):
        """Load obstacles from a JSON file."""
        try:
            with open(filename, "r") as f:
                map_data = json.load(f)
            
            obstacles = []
            for obs_data in map_data.get("obstacles", []):
                if obs_data.get("is_rect", False):
                    obs = Obstacle(
                        obs_data["x"],
                        obs_data["y"],
                        obs_data.get("r", 0),
                        width=obs_data.get("width", 200),
                        height=obs_data.get("height", 20)
                    )
                else:
                    obs = Obstacle(
                        obs_data["x"],
                        obs_data["y"],
                        obs_data.get("r", 60)
                    )
                obstacles.append(obs)
            
            return obstacles
        except FileNotFoundError:
            print(f"Map file {filename} not found")
            return []
        except Exception as e:
            print(f"Error loading map: {e}")
            return []
    
    def draw_editor_ui(self, screen, font):
        """Draw map editor UI overlay."""
        if not self.active:
            return
        
        # Draw semi-transparent overlay
        overlay = pygame.Surface((SIM_WIDTH, HEIGHT), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 128))
        screen.blit(overlay, (0, 0))
        
        # Draw editor instructions
        y = 20
        if self.obstacle_type == "circular":
            instructions = [
                "MAP EDITOR MODE",
                f"Type: {self.obstacle_type}",
                "Left-click: Place obstacle",
                "Right-click: Remove obstacle",
                "T: Toggle obstacle type",
                "+/-: Increase/Decrease size",
                "S: Save map",
                "L: Load map",
                "M: Exit editor",
                f"Size: {self.obstacle_size}"
            ]
        else:
            instructions = [
                "MAP EDITOR MODE",
                f"Type: {self.obstacle_type}",
                "Left-click: Place obstacle",
                "Right-click: Remove obstacle",
                "T: Toggle obstacle type",
                "Q/A: Width +/-",
                "W/X: Height +/-",
                "S: Save map",
                "L: Load map",
                "M: Exit editor",
                f"Size: {self.rect_width}x{self.rect_height}"
            ]
        
        for line in instructions:
            text = font.render(line, True, (255, 255, 255))
            screen.blit(text, (20, y))
            y += 25

