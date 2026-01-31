import pygame
import sys

from car import Car

pygame.init()

font = pygame.font.SysFont("Arial", 18)
WIDTH = 800
HEIGHT = 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("RL Car Simulation")

BLACK = (0,0,0)
BLUE = (0,0,255)

clock = pygame.time.Clock()
running = True

car = Car(WIDTH//2,HEIGHT//2, font=font)

while running:
    screen.fill(BLACK)

    keys = pygame.key.get_pressed()
    car.update(keys)
    car.draw(screen)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(60)


pygame.quit()
sys.exit()