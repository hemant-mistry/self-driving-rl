import pygame
import math



class Car:
    def __init__(self, x, y, font):
        self.x = x
        self.y = y
        self.width = 50
        self.height = 30
        self.speed = 0
        self.heading = 0 # in degrees
        self.max_speed = 5
        self.acceleration = 0.2
        self.turn_speed = 5 # degrees per frame
        self.font = font

    def _get_corners(self, x=None, y=None, heading=None):
        """Return the four corner points (world coords) of the car."""
        if x is None: x = self.x
        if y is None: y = self.y
        if heading is None: heading = self.heading

        w = self.width
        h = self.height
        rad = math.radians(heading)
        cos_t = math.cos(rad)
        sin_t = math.sin(rad)

        local = [
            (-w/2, -h/2),
            (w/2, -h/2),
            (w/2, h/2),
            (-w/2, h/2)
        ]

        world = []
        for px, py in local:
            rx = px * cos_t - py * sin_t + x
            ry = px * sin_t + py * cos_t + y
            world.append((rx, ry))

        return world


    def _point_in_polygon(self, px, py, poly):
        """Ray casting point-in-polygon. poly is a list of (x,y)."""
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            intersect = ((yi > py) != (yj > py)) and \
                        (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
            if intersect:
                inside = not inside
            j = i
        return inside


    def _all_corners_inside_polygon(self, poly):
        corners = self._get_corners()
        for cx, cy in corners:
            if not self._point_in_polygon(cx, cy, poly):
                return False
        return True

    def update(self, keys, road_polygon):
        # store previous pose for potential revert
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_heading = self.heading

        # steering
        if keys[pygame.K_LEFT]:
            self.heading -= self.turn_speed
        if keys[pygame.K_RIGHT]:
            self.heading += self.turn_speed

        # throttle/ brake
        if keys[pygame.K_UP]:
            self.speed += self.acceleration
        elif keys[pygame.K_DOWN]:
            self.speed -= self.acceleration
        else:
            self.speed *= 0.95  # friction / slow down

        # clamp speed
        if self.speed > self.max_speed:
            self.speed = self.max_speed
        if self.speed < -self.max_speed / 2:
            self.speed = -self.max_speed / 2

        # tentative move
        rad = math.radians(self.heading)
        dx = self.speed * math.cos(rad)
        dy = self.speed * math.sin(rad)

        self.x += dx
        self.y += dy

        # collision check: if any corner outside road polygon, revert and damp speed
        if not self._all_corners_inside_polygon(road_polygon):
            # simple collision response: revert position and reduce speed
            self.x = self.prev_x
            self.y = self.prev_y
            self.heading = self.prev_heading
            self.speed *= -0.2


    def rl_update(self, action, road_polygon):
        # store previous pose for potential revert
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_heading = self.heading
        self.collided = False

        if action == 3:
            self.heading -= self.turn_speed
        elif action == 4:
            self.heading += self.turn_speed
        
        if action == 1:
            self.speed += self.acceleration
        elif action == 2:
            self.speed -= self.acceleration
        else:
            self.speed *= 0.95

        # clamp speed
        if self.speed > self.max_speed:
            self.speed = self.max_speed
        if self.speed < -self.max_speed / 2:
            self.speed = -self.max_speed / 2

        # tentative move
        rad = math.radians(self.heading)
        dx = self.speed * math.cos(rad)
        dy = self.speed * math.sin(rad)

        self.x += dx
        self.y += dy

        # collision check: if any corner outside road polygon, revert and damp speed
        if not self._all_corners_inside_polygon(road_polygon):
            # simple collision response: revert position and reduce speed
            self.x = self.prev_x
            self.y = self.prev_y
            self.heading = self.prev_heading
            self.speed *= -0.2
            self.collided = True




    def draw_car(self, screen, x, y, heading, width, height, color=(0,0,255), debug = True):
        # define car corners relative to center

        ''' (-w/2, -h/2)      (w/2, -h/2)
           +----------------+
           |                |
           |                |
           +----------------+
        (-w/2, h/2)       (w/2, h/2) '''
    
        points = [
            (-width/2, -height/2),
            (width/2, -height/2),
            (width/2, height/2),
            (-width/2, height/2)
        ]

        # rotate points
        rad = math.radians(heading)
        rotated_points = []
        cos_theta = math.cos(rad)
        sin_theta = math.sin(rad)
        for px, py in points:
            rx = px*math.cos(rad) - py*math.sin(rad)+x
            ry = px*math.sin(rad) + py*math.cos(rad)+y
            rotated_points.append((rx,ry))
    
        pygame.draw.polygon(screen, color, rotated_points)

        if debug:
            # draw center
            pygame.draw.circle(screen, (0,255,0), (int(x), int(y)), 5)

            # draw vectors to all corners
            for rx, ry in rotated_points:
                pygame.draw.line(screen, (255,0,0), (x, y), (rx, ry), 2)

            # highlight first corner projections
            rx, ry = rotated_points[0]
            pygame.draw.line(screen, (0,255,255), (x, y), (rx, y), 2)  # cosθ horizontal
            pygame.draw.line(screen, (255,255,0), (rx, y), (rx, ry), 2)  # sinθ vertical

            # show text
            self.draw_text(screen, f"Heading: {heading:.1f}°", 10, 10)
            self.draw_text(screen, f"cosθ: {cos_theta:.3f}", 10, 30)
            self.draw_text(screen, f"sinθ: {sin_theta:.3f}", 10, 50)
            self.draw_text(screen, f"Corner0: ({rx:.1f}, {ry:.1f})", 10, 70)

        
        
    def draw(self, screen):
        self.draw_car(screen, self.x, self.y, self.heading, self.width, self.height, debug=True)


    def draw_text(self, screen, text, x, y, color=(255,255,255)):
        img = self.font.render(text, True, color)
        screen.blit(img, (x, y))