import pygame
import sys
import math
from car import Car
import random
from collections import defaultdict
pygame.init()

# -----------------------------
# CONFIG
# -----------------------------
WIDTH, HEIGHT = 1000, 700
ROAD_WIDTH = 120
MARGIN = 80

BLACK = (0, 0, 0)
ROAD_COLOR = (60, 60, 60)
WHITE = (255, 255, 255)
GREEN = (0, 200, 0)
RED = (200, 0, 0)

Q = defaultdict(lambda: [0.0] * 5)
alpha = 0.1     # learning rate
gamma = 0.95    # discount factor
epsilon = 0.2   # exploration rate
# -----------------------------
# RAW TRACK (LOGICAL SPACE)
# -----------------------------
raw_centerline = [
    (0, 0),
    (0, 200),
    (120, 340),
    (320, 360),
    (520, 330),
    (680, 200),
    (680, 0),
]

# -----------------------------
# UTILITIES
# -----------------------------
def smooth_path(points, samples=20):
    smooth = []
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        for t in range(samples):
            a = t / samples
            smooth.append((
                x1 * (1 - a) + x2 * a,
                y1 * (1 - a) + y2 * a
            ))
    smooth.append(points[-1])
    return smooth


def fit_centerline_to_screen(points, w, h, margin):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    track_w = max_x - min_x
    track_h = max_y - min_y

    scale = min(
        (w - 2 * margin) / track_w,
        (h - 2 * margin) / track_h
    )

    cx = (min_x + max_x) / 2
    cy = (min_y + max_y) / 2

    screen_cx = w / 2
    screen_cy = h / 2

    fitted = []
    for x, y in points:
        x = (x - cx) * scale + screen_cx
        y = (y - cy) * scale + screen_cy
        fitted.append((x, y))

    return fitted

def normalize_angle_deg(angle):
    """ Normalize angle to [-180, 180] degrees. """
    while angle>180:
        angle-=360
    while angle<-180:
        angle+=360
    return angle

def compute_heading_error(car_x, car_y, car_heading_deg, centerline):
    """
    Returns heading error in degress between car heading
    and direction of the closest centerline segment
    """

    min_dist = float("inf")
    best_p1 = None
    best_p2 = None

    # find closest centerline segment (via midpoint)
    for i in range(len(centerline)-1):
        x1,y1 = centerline[i]
        x2,y2 = centerline[i+1]

        mx = (x1+x2)/2
        my = (y1+y2)/2

        dx = car_x-mx
        dy = car_y-my

        dist = math.hypot(dx,dy)

        if dist<min_dist:
            min_dist = dist
            best_p1 = (x1,y1)
            best_p2 = (x2,y2)

    # Direction of the road segment
    dx = best_p2[0]-best_p1[0]
    dy = best_p2[1]-best_p1[1]

    road_angle_rad = math.atan2(dy,dx)
    road_angle_rad = math.degrees(road_angle_rad)

    # Heading error
    heading_error = car_heading_deg - road_angle_rad
    heading_error = normalize_angle_deg(heading_error)

    return heading_error

def distance_point_to_segment(px, py, x1, y1, x2, y2):
    """
    Returns the shortest distance from point P(px,py)
    to the line segment (x1,y1)-(x2,y2)
    """

    dx = x2-x1
    dy = y2-y1
    
    if(dx == 0 and dy==0):
        # segment is a point
        return math.hypot(px-x1, py-y1)
    
    # Projection factor t
    t = ((px-x1)*dx + (py-y1)*dy)/(dx*dx + dy*dy)
    t = max(0.0, min(1.0, t)) # Clamp to segment

    closest_x = x1 + t*dx
    closest_y = y1 + t*dy

    return math.hypot(px-closest_x, py-closest_y)

def compute_distance_to_centerline(car_x, car_y, centerline):
    min_dist = float("inf")
    for i in range(len(centerline)-1):
        x1,y1 = centerline[i]
        x2,y2 = centerline[i+1]

        dist = distance_point_to_segment(car_x, car_y, x1,y1, x2,y2)
        if(dist<min_dist):
            min_dist = dist

    return min_dist


def discretize_state(speed, heading_error, distance):
    # Speed
    if abs(speed) < 0.5:
        s = 0      # STOPPED
    elif abs(speed) < 2.5:
        s = 1      # SLOW
    else:
        s = 2      # FAST

    # Heading
    if heading_error < -20:
        h = 0      # HARD_LEFT
    elif heading_error < -5:
        h = 1      # LEFT
    elif heading_error < 5:
        h = 2      # STRAIGHT
    elif heading_error < 20:
        h = 3      # RIGHT
    else:
        h = 4      # HARD_RIGHT

    # Distance
    if distance < 10:
        d = 0      # CENTER
    elif distance < 30:
        d = 1      # OFF
    else:
        d = 2      # FAR

    return (s, h, d)




# -----------------------------
# BUILD FINAL CENTERLINE
# -----------------------------
centerline = smooth_path(raw_centerline, samples=20)
centerline = fit_centerline_to_screen(centerline, WIDTH, HEIGHT, MARGIN)

# -----------------------------
# ROAD DRAWING
# -----------------------------
def draw_road(screen, centerline, width):
    half = width / 2
    left_edge = []
    right_edge = []
    normals = []

    for i in range(len(centerline) - 1):
        x1, y1 = centerline[i]
        x2, y2 = centerline[i + 1]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length == 0:
            dx, dy = 1, 0
            length = 1
        dx /= length
        dy /= length
        normals.append((-dy, dx))

    for i in range(len(centerline)):
        x, y = centerline[i]
        if i == 0:
            nx, ny = normals[0]
        elif i == len(centerline) - 1:
            nx, ny = normals[-1]
        else:
            nx = normals[i - 1][0] + normals[i][0]
            ny = normals[i - 1][1] + normals[i][1]
            length = math.hypot(nx, ny)
            if length == 0:
                nx, ny = normals[i]
            else:
                nx /= length
                ny /= length

        left_edge.append((x + nx * half, y + ny * half))
        right_edge.append((x - nx * half, y - ny * half))

    road_polygon = left_edge + right_edge[::-1]

    pygame.draw.polygon(screen, ROAD_COLOR, road_polygon)
    pygame.draw.lines(screen, WHITE, False, left_edge, 4)
    pygame.draw.lines(screen, WHITE, False, right_edge, 4)

    return left_edge, right_edge, road_polygon


# -----------------------------
# START / FINISH
# -----------------------------
def draw_gate(screen, p1, p2, color, label):
    pygame.draw.line(screen, color, p1, p2, 6)
    font = pygame.font.SysFont("Arial", 20)
    text = font.render(label, True, color)
    mx = (p1[0] + p2[0]) / 2
    my = (p1[1] + p2[1]) / 2
    screen.blit(text, (mx - 30, my - 30))


# -----------------------------
# PYGAME SETUP
# -----------------------------
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("RL Car Simulation")

clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)

car = Car(centerline[0][0], centerline[0][1] + 40, font=font)

# Initialize state before loop
heading_error = compute_heading_error(car.x, car.y, car.heading, centerline)
distance_to_center = compute_distance_to_centerline(car.x, car.y, centerline)
state = discretize_state(car.speed, heading_error, distance_to_center)

# Define the goal
finish_line_point = centerline[-1]

prev_dist_to_finish = math.hypot(car.x - finish_line_point[0], car.y-finish_line_point[1])

# -----------------------------
# MAIN LOOP
# -----------------------------
running = True
while running:
    screen.fill(BLACK)

    left_edge, right_edge, road_polygon = draw_road(screen, centerline, ROAD_WIDTH)

    # Debug centerline (remove later for RL)
    pygame.draw.lines(screen, (100, 100, 255), False, centerline, 1)

    # Start & finish
    draw_gate(screen, left_edge[3], right_edge[3], GREEN, "START")
    draw_gate(screen, left_edge[-4], right_edge[-4], RED, "FINISH")

    #keys = pygame.key.get_pressed()
    if random.random() < epsilon:
        action = random.randint(0, 4)   # explore
    else:
        action = max(range(5), key=lambda a: Q[state][a])  # exploit

    prev_state = state

    car.rl_update(action=action, road_polygon=road_polygon)   # <-- pass road polygon for collision checks
    #car.update(keys, road_polygon)
    heading_error = compute_heading_error(car.x,car.y,car.heading,centerline)
    text = font.render(f"Heading error: {heading_error:.1f}°", True, (255, 255, 0))
    screen.blit(text, (10, 90))

    distance_to_center = compute_distance_to_centerline(car.x,car.y,centerline)
    text = font.render(f"Distance to centerline: {distance_to_center:.1f}",True,(255, 200, 0))
    screen.blit(text, (10, 120))
    
    next_state = discretize_state(car.speed,heading_error,distance_to_center)
    text = font.render(f"State: {next_state}", True, (0, 255, 255))
    screen.blit(text, (10, 150))

    curr_dist_to_finish = math.hypot(car.x - finish_line_point[0], car.y-finish_line_point[1])
    progress = prev_dist_to_finish - curr_dist_to_finish

    reward = 0.0
    reward+=progress*2.0
    if car.speed>0:
        reward+=0.1
    else:
        reward-=0.1

    reward-=0.05*abs(heading_error)
    reward-=3.0*distance_to_center
    if car.collided:
        reward-=10
    prev_dist_to_finish = curr_dist_to_finish
    best_next = max(Q[next_state])

    Q[prev_state][action] += alpha * (
        reward + gamma * best_next - Q[prev_state][action]
        )
    
    # # Decay epsilon
    # if epsilon > 0.01:
    #     epsilon *= 0.995  # Slowly reduce randomness
    car.draw(screen)    

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
