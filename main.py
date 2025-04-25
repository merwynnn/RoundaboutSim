import pygame
from Simulator import Simulator
from Constants import *
from Road import Road, RoadExtremity
from Car import Car
from Intersections import *
import sys
print("start")
# Pygame setup
pygame.init()

win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Roundabout Simulator")

clock = pygame.time.Clock()

simulator = Simulator(win)


while True:
    dt = clock.tick(60) # Delta time in milliseconds
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_d:
                DEBUG = not DEBUG
                simulator.debug = DEBUG
            elif event.key == pygame.K_1:
                intersections_0 = [ClassicRoundabout((400, 400), 100, [Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)])]

                ext1 = RoadExtremity((0, HEIGHT//2), spawn_cars=True, spawn_cars_timer=60)
                ext2 = RoadExtremity((WIDTH, HEIGHT//2), spawn_cars=True, spawn_cars_timer=60)
                ext_3 = RoadExtremity((WIDTH//2, 0), spawn_cars=True, spawn_cars_timer=60)
                ext_4 = RoadExtremity((WIDTH//2, HEIGHT), spawn_cars=True, spawn_cars_timer=60)

                roads_0 = [Road(ext1, intersections_0[0].exits[0]), Road(intersections_0[0].exits[1], ext2), Road(ext_3, intersections_0[0].exits[2]), Road(ext_4, intersections_0[0].exits[3])]

                road_extremity_spawners_0 = [ext1, ext2, ext_3, ext_4]
                simulator.initialize(intersections_0, roads_0, road_extremity_spawners_0)
            elif event.key == pygame.K_2:
                ext1_1 = RoadExtremity((0, HEIGHT//2), spawn_cars=True, spawn_cars_timer=60)
                ext2_1 = RoadExtremity((WIDTH, HEIGHT//2), spawn_cars=False, spawn_cars_timer=60)
                roads_1 = [Road(ext1_1, ext2_1)]

                road_extremity_spawners_1 = [ext1_1, ext2_1]
                simulator.initialize([], roads_1, road_extremity_spawners_1)


    if simulator:
        simulator.update(dt, events)
    pygame.display.update()
