import pygame

# Object Transport task
class TransportObject:
    def __init__(self, pos, radius=20, damping=0.95):
        self.pos = pygame.Vector2(pos)
        self.vel = pygame.Vector2(0,0)
        self.r = radius
        self.damping = damping

    def update(self):
        # Simple physics: position update
        self.pos += self.vel
        # Apply damping
        self.vel *= self.damping

    def draw(self, surf):
        pygame.draw.circle(surf, (200,100,50), (int(self.pos.x), int(self.pos.y)), self.r)
