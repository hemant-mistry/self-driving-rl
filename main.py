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

Q = defaultdict(lambda: [0.0] * 9)
alpha = 0.1     # learning rate
gamma = 0.95    # discount factor
epsilon = 0.4   # exploration rate
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

def compute_future_heading_error(car_x, car_y, car_heading, centerline, lookahead=10):
    min_idx = 0
    min_dist = float("inf")
    for i in range(len(centerline)):
        d = math.hypot(car_x-centerline[i][0], car_y-centerline[i][1])
        if d < min_dist:
            min_dist = d
            min_idx = i

    i2 = min(min_idx + lookahead, len(centerline)-1)
    dx = centerline[i2][0] - centerline[min_idx][0]
    dy = centerline[i2][1] - centerline[min_idx][1]
    road_angle = math.degrees(math.atan2(dy, dx))
    return normalize_angle_deg(car_heading - road_angle)


def discretize_state(speed, heading_error, distance, future_heading_error):
    # Speed
    if abs(speed) < 0.5:
        s = 0      # STOPPED
    elif abs(speed) < 2.5:
        s = 1      # SLOW
    else:
        s = 2      # FAST

    # Heading (current)
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

    # Future heading error (lookahead) discretized into 3 bins
    # Negative => road turns left ahead; positive => road turns right ahead
    if future_heading_error < -10:
        fh = 0    # FUTURE_LEFT
    elif future_heading_error > 10:
        fh = 2    # FUTURE_RIGHT
    else:
        fh = 1    # FUTURE_STRAIGHT

    # Always return a tuple (so Q dict keys are consistent)
    return (s, h, d, fh)





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
future_heading_error = compute_future_heading_error(car_heading=car.heading, car_x=car.x, car_y=car.y, centerline=centerline)

state = discretize_state(car.speed, heading_error, distance_to_center,future_heading_error)

# Define the goal
finish_line_point = centerline[-4]

prev_dist_to_finish = math.hypot(car.x - finish_line_point[0], car.y-finish_line_point[1])

# -----------------------------
# MAIN LOOP
# -----------------------------
running = True
tries = 0;
current_lap_clean = True
lap_times = []
best_lap = None
lap_start_time = pygame.time.get_ticks()

while running:
    screen.fill(BLACK)

    left_edge, right_edge, road_polygon = draw_road(screen, centerline, ROAD_WIDTH)

    # Debug centerline (remove later for RL)
    pygame.draw.lines(screen, (100, 100, 255), False, centerline, 1)

    # Start & finish
    draw_gate(screen, left_edge[3], right_edge[3], GREEN, "START")
    draw_gate(screen, left_edge[-4], right_edge[-4], RED, "FINISH")

    #  Lap timings
    current_time = (pygame.time.get_ticks() - lap_start_time) / 1000.0
    lap_text = font.render(f"Lap Time: {current_time:.1f}s", True, (200, 200, 255))
    lap_text_x = WIDTH - lap_text.get_width() - MARGIN
    screen.blit(lap_text, (lap_text_x, 50))

    if best_lap is not None:
        best_text = font.render(f"Best Lap: {best_lap:.2f}s", True, (100, 255, 100))
        best_text_x = WIDTH - best_text.get_width() - MARGIN
        screen.blit(best_text, (best_text_x, 70))

    #keys = pygame.key.get_pressed()
    if random.random() < epsilon:
        action = random.randint(0,8)   # explore
    else:
        action = max(range(9), key=lambda a: Q[state][a])  # exploit

    prev_state = state

    car.rl_update(action=action, road_polygon=road_polygon)   # <-- pass road polygon for collision checks
    #car.update(keys, road_polygon)
    heading_error = compute_heading_error(car.x,car.y,car.heading,centerline)
    text = font.render(f"Heading error: {heading_error:.1f}°", True, (255, 255, 0))
    screen.blit(text, (10, 90))

    distance_to_center = compute_distance_to_centerline(car.x,car.y,centerline)
    text = font.render(f"Distance to centerline: {distance_to_center:.1f}",True,(255, 200, 0))
    screen.blit(text, (10, 120))
    
    future_heading_error = compute_future_heading_error(car.x, car.y, car.heading, centerline)
    next_state = discretize_state(car.speed,heading_error,distance_to_center, future_heading_error)
    text = font.render(f"State: {next_state}", True, (0, 255, 255))
    screen.blit(text, (10, 150))

    tries_text = f"Total Tries: {tries}"
    text_surface = font.render(tries_text, True, (255, 255, 255))
    tries_x = WIDTH - text_surface.get_width() - MARGIN
    screen.blit(text_surface, (tries_x, 10))

    curr_dist_to_finish = math.hypot(car.x - finish_line_point[0], car.y-finish_line_point[1])
    progress = prev_dist_to_finish - curr_dist_to_finish

    reward = 0.0
    reward += max(progress, 0) * 5.0
    
    if progress < 0:
        reward -= 1.0


    if car.speed>0:
        reward+=0.1
    else:
        reward-=0.1

    reward -= abs(heading_error) / 90.0
    reward -= min(distance_to_center / (ROAD_WIDTH/2), 1.0)
    if car.collided:
        reward-=10
        current_lap_clean = False

    prev_dist_to_finish = curr_dist_to_finish
    best_next = max(Q[next_state])

    Q[prev_state][action] += alpha * (
        reward + gamma * best_next - Q[prev_state][action]
        )
    

    state = next_state
    if curr_dist_to_finish < 40:
        print(f"Lap Finished!")
        lap_end_time = pygame.time.get_ticks()
        lap_time_sec = (lap_end_time-lap_start_time)/1000.0
        lap_times.append(lap_time_sec)

        if best_lap is None or best_lap>lap_time_sec:
            best_lap = lap_time_sec
        
        print(f"Lap {tries + 1} finished in {lap_time_sec:.2f}s | Best: {best_lap:.2f}s")

        # Extra reward for finishing
        Q[prev_state][action] += alpha * (100 + gamma * 0 - Q[prev_state][action])

        tries += 1

        # Reset Physics: place the car exactly where it was spawned initially
        car.x = centerline[0][0]
        car.y = centerline[0][1] + 40   # <-- IMPORTANT: same offset used at initial creation
        car.speed = 0.0


        # Align heading with the first centerline segment so it faces the track
        dx0 = centerline[1][0] - centerline[0][0]
        dy0 = centerline[1][1] - centerline[0][1]
        car.heading = math.degrees(math.atan2(dy0, dx0))

        car.collided = False
        current_lap_clean = True

        # Reset Distance Tracker
        prev_dist_to_finish = math.hypot(car.x - finish_line_point[0], car.y - finish_line_point[1])

        # Reset the brain state too
        heading_error = compute_heading_error(car.x, car.y, car.heading, centerline)
        distance_to_center = compute_distance_to_centerline(car.x, car.y, centerline)
        future_heading_error = compute_future_heading_error(car.x, car.y, car.heading, centerline)
        state = discretize_state(car.speed, heading_error, distance_to_center, future_heading_error)
        prev_state = state

        # Reset the timer
        lap_start_time = pygame.time.get_ticks()


    # Decay epsilon
    epsilon = max(0.02, epsilon * 0.95)
    car.draw(screen)    

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
