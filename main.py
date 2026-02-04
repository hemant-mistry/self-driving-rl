import pygame
import sys
import math
from car import Car
import random
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
    action = random.randint(0, 4)

    car.rl_update(action=action, road_polygon=road_polygon)   # <-- pass road polygon for collision checks
    car.draw(screen)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
