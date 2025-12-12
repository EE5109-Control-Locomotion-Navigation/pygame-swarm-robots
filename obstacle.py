import pygame

class Obstacle:
    def __init__(self, x, y, r, width=None, height=None):
        self.pos = pygame.Vector2(x, y)
        self.r = r
        # For rectangular obstacles (maze walls)
        self.width = width if width is not None else r * 2
        self.height = height if height is not None else r * 2
        self.is_rect = (width is not None or height is not None)
    
    def get_rect(self):
        """Get pygame Rect for this obstacle."""
        if self.is_rect:
            return pygame.Rect(
                int(self.pos.x - self.width / 2),
                int(self.pos.y - self.height / 2),
                int(self.width),
                int(self.height)
            )
        else:
            # Circular obstacle - return bounding rect
            side = self.r * 2
            return pygame.Rect(
                int(self.pos.x - self.r),
                int(self.pos.y - self.r),
                side,
                side
            )

    def draw(self, surf):
        if self.is_rect:
            # Draw rectangular wall
            rect = self.get_rect()
            pygame.draw.rect(surf, (120, 50, 50), rect)
        else:
            # Draw circle for circular obstacles
            pygame.draw.circle(surf, (120, 50, 50), 
                            (int(self.pos.x), int(self.pos.y)), int(self.r))