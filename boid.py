import pygame, math, random
from obstacle import Obstacle

class Boid:
    def __init__(self):
        self.pos = pygame.Vector2(random.uniform(0,1100), random.uniform(0,900))
        ang = random.uniform(0, math.tau)
        self.vel = pygame.Vector2(math.cos(ang), math.sin(ang)) * 2
        self.sensor_angles = [0, 35, -35]
        self.sensor_length = 100
        self.max_speed = 3

    def potential_field(self, obstacles, maze_obstacle_scale=1.0, goal_pos=None, goal_influence_range=100):
        """Calculate avoidance force from obstacles.
        
        Args:
            obstacles: List of obstacles to avoid
            maze_obstacle_scale: Scaling factor for maze obstacles (0.0-1.0)
            goal_pos: Optional goal position to reduce avoidance when goal is nearby
            goal_influence_range: Distance within which goal reduces avoidance
        """
        force = pygame.Vector2()
        
        # Check if goal is nearby - if so, reduce avoidance
        goal_reduction = 1.0
        if goal_pos is not None:
            goal_dist = (self.pos - goal_pos).length()
            if goal_dist < goal_influence_range:
                # Reduce avoidance when goal is close (allows reaching goals near walls)
                goal_reduction = 0.3 + 0.7 * (goal_dist / goal_influence_range)  # 0.3 to 1.0
        
        for obs in obstacles:
            # Check if this is a maze obstacle (rectangular with small or zero radius)
            is_maze_obstacle = obs.is_rect and obs.r == 0
            
            if obs.is_rect:
                # For rectangular obstacles, find closest point on rectangle
                rect = obs.get_rect()
                closest_x = max(rect.left, min(self.pos.x, rect.right))
                closest_y = max(rect.top, min(self.pos.y, rect.bottom))
                closest_point = pygame.Vector2(closest_x, closest_y)
                dvec = self.pos - closest_point
            else:
                # For circular obstacles
                dvec = self.pos - obs.pos
            
            dist = dvec.length()
            # Use appropriate radius for distance calculation
            effective_r = obs.r if not obs.is_rect else max(obs.width, obs.height) / 2
            sd = dist - effective_r
            
            # Reduced range for maze obstacles to allow closer approach
            max_range = 80 if is_maze_obstacle else 150
            
            if sd < max_range:
                if sd < 1: sd = 1
                if dvec.length() > 0:
                    dvec.normalize_ip()
                    # Apply scaling for maze obstacles with gentler falloff
                    if is_maze_obstacle:
                        # Much gentler force for maze obstacles
                        base_force = (200/(sd**1.5)) * maze_obstacle_scale  # Softer falloff (1.5 instead of 2)
                    else:
                        base_force = (1800/(sd**2))
                    # Apply goal-based reduction
                    force += dvec * base_force * goal_reduction
        return force

    def update(self, boids, obstacles, params, maze_obstacle_scale=1.0, goal_pos=None):
        align_w, coh_w, sep_w, obs_w, max_speed, sensor_len = params
        self.max_speed = max_speed
        self.sensor_length = sensor_len

        alignment = pygame.Vector2()
        cohesion = pygame.Vector2()
        separation = pygame.Vector2()
        total = 0
        NEIGH = 65
        SEP = 25

        for other in boids:
            if other is self: continue
            d = self.pos.distance_to(other.pos)
            if d < NEIGH:
                alignment += other.vel
                cohesion += other.pos
                total += 1
                if d < SEP:
                    separation += (self.pos - other.pos) / d

        if total > 0:
            alignment = (alignment/total).normalize()*self.max_speed - self.vel
            cohesion = ((cohesion/total) - self.pos) * 0.01
            separation *= 0.5

        avoid = self.potential_field(obstacles, maze_obstacle_scale, goal_pos) * obs_w

        self.vel += alignment*align_w*0.05
        self.vel += cohesion*coh_w
        self.vel += separation*sep_w
        self.vel += avoid

        if self.vel.length() > self.max_speed:
            self.vel.scale_to_length(self.max_speed)

        self.pos += self.vel
        if self.pos.x < 0: self.pos.x = 1100
        if self.pos.x > 1100: self.pos.x = 0
        if self.pos.y < 0: self.pos.y = 900
        if self.pos.y > 900: self.pos.y = 0

    def draw(self, surf, dbg_vel, dbg_sensors, dbg_neighbors):
        # Draw triangle pointing in direction of travel
        if self.vel.length() > 0:
            # Get velocity direction (normalized)
            vel_dir = self.vel.normalize()
            
            # Triangle dimensions
            size = 8  # Size of the triangle
            # Points relative to center: front point, and two back points
            front_point = pygame.Vector2(size, 0)
            back_left = pygame.Vector2(-size * 0.6, -size * 0.5)
            back_right = pygame.Vector2(-size * 0.6, size * 0.5)
            
            # Calculate rotation angle in degrees
            angle = math.degrees(math.atan2(vel_dir.y, vel_dir.x))
            
            # Rotate points based on velocity direction
            front = front_point.rotate(angle) + self.pos
            back_l = back_left.rotate(angle) + self.pos
            back_r = back_right.rotate(angle) + self.pos
            
            # Draw filled triangle
            pygame.draw.polygon(surf, (255, 220, 120), [
                (int(front.x), int(front.y)),
                (int(back_l.x), int(back_l.y)),
                (int(back_r.x), int(back_r.y))
            ])
        else:
            # If velocity is zero, draw a small circle
            pygame.draw.circle(surf, (255, 220, 120), (int(self.pos.x), int(self.pos.y)), 5)

        if dbg_vel:
            e = self.pos + self.vel*10
            pygame.draw.line(surf, (80,150,255), self.pos, e, 2)

        if dbg_sensors and self.vel.length()>0:
            for ang in self.sensor_angles:
                d = self.vel.normalize().rotate(ang)
                end = self.pos + d*self.sensor_length
                pygame.draw.line(surf, (255,80,140), self.pos, end, 1)

        if dbg_neighbors:
            pygame.draw.circle(surf, (100,100,255),(int(self.pos.x),int(self.pos.y)),60,1)
