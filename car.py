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


    def update(self, keys):
        # steering
        if keys[pygame.K_LEFT]:
            self.heading-=self.turn_speed
        if keys[pygame.K_RIGHT]:
            self.heading+=self.turn_speed

        # throttle/ brake
        if keys[pygame.K_UP]:
            self.speed+=self.acceleration
        elif keys[pygame.K_DOWN]:
            self.speed-=self.acceleration
        else:
            self.speed*=0.95 #friction /slow down

        
        # clamp speed
        if self.speed>self.max_speed:
            self.speed = self.max_speed
        
        if self.speed< -self.max_speed/2:
            self.speed = -self.max_speed/2

        
        #update position using heading
        rad = math.radians(self.heading)
        self.x += self.speed * math.cos(rad)
        self.y += self.speed * math.sin(rad)



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