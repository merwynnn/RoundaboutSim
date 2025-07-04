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

car_flow_rate = 120

def create_grid_setup(n, m):
    intersections = []
    roads = []
    road_extremity_spawners = []

    fixed_road_length = 500
    roundabout_radius = 100
    # Spacing between the centers of adjacent roundabouts
    spacing_between_centers = fixed_road_length + 2 * roundabout_radius

    # Create intersections
    for i in range(n):
        for j in range(m):
            # Calculate position for the center of the roundabout
            pos_x = (j * spacing_between_centers) + spacing_between_centers
            pos_y = (i * spacing_between_centers) + spacing_between_centers
            pos = (int(pos_x), int(pos_y))
            intersections.append(ClassicRoundabout(pos, roundabout_radius, [Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)]))

    # Create roads connecting intersections
    for i in range(n):
        for j in range(m):
            current_intersection_index = i * m + j
            current_intersection = intersections[current_intersection_index]

            # Connect to intersection on the right
            if j < m - 1:
                right_intersection_index = i * m + (j + 1)
                right_intersection = intersections[right_intersection_index]
                roads.append(Road(current_intersection.exits[1], right_intersection.exits[0])) # Connect right exit to left entry
                roads.append(Road(right_intersection.exits[0], current_intersection.exits[1])) # Connect left exit to right entry

            # Connect to intersection below
            if i < n - 1:
                below_intersection_index = (i + 1) * m + j
                below_intersection = intersections[below_intersection_index]
                roads.append(Road(current_intersection.exits[3], below_intersection.exits[2])) # Connect down exit to up entry
                roads.append(Road(below_intersection.exits[2], current_intersection.exits[3])) # Connect up exit to down entry

    # Create road extremities at the edges of the grid
    # Top edge
    for j in range(m):
        center_x_col_j = (j * spacing_between_centers) + spacing_between_centers
        first_row_roundabout_up_exit_y = ((0 * spacing_between_centers) + spacing_between_centers) - roundabout_radius
        spawner_y = first_row_roundabout_up_exit_y - fixed_road_length
        ext_pos = (int(center_x_col_j), int(spawner_y))
        ext = RoadExtremity(ext_pos, spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(ext, intersections[j].exits[2]))

    # Bottom edge
    for j in range(m):
        center_x_col_j = (j * spacing_between_centers) + spacing_between_centers
        last_row_idx = n - 1
        last_row_roundabout_down_exit_y = ((last_row_idx * spacing_between_centers) + spacing_between_centers) + roundabout_radius
        spawner_y = last_row_roundabout_down_exit_y + fixed_road_length
        ext_pos = (int(center_x_col_j), int(spawner_y))
        ext = RoadExtremity(ext_pos, spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(intersections[last_row_idx * m + j].exits[3], ext))

    # Left edge
    for i in range(n):
        center_y_row_i = (i * spacing_between_centers) + spacing_between_centers
        first_col_idx = 0
        first_col_roundabout_left_exit_x = ((first_col_idx * spacing_between_centers) + spacing_between_centers) - roundabout_radius
        spawner_x = first_col_roundabout_left_exit_x - fixed_road_length
        ext_pos = (int(spawner_x), int(center_y_row_i))
        ext = RoadExtremity(ext_pos, spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(ext, intersections[i * m + first_col_idx].exits[0]))

    # Right edge
    for i in range(n):
        center_y_row_i = (i * spacing_between_centers) + spacing_between_centers
        last_col_idx = m - 1
        last_col_roundabout_right_exit_x = ((last_col_idx * spacing_between_centers) + spacing_between_centers) + roundabout_radius
        spawner_x = last_col_roundabout_right_exit_x + fixed_road_length
        ext_pos = (int(spawner_x), int(center_y_row_i))
        ext = RoadExtremity(ext_pos, spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(intersections[i * m + last_col_idx].exits[1], ext))


    return intersections, roads, road_extremity_spawners


time_multiplier = 1
while True:
    dt = clock.tick(60) * time_multiplier # Delta time in milliseconds
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RIGHT:
                if time_multiplier == 1:
                    time_multiplier = 2
                elif time_multiplier == 2:
                    time_multiplier = 4
                else:
                    time_multiplier = 1
            elif event.key == pygame.K_d:
                DEBUG = not DEBUG
                simulator.debug = DEBUG
            elif event.key == pygame.K_1:
                intersections_0 = [ClassicRoundabout((400, 400), 100, [Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)]),]

                ext1 = RoadExtremity((0, HEIGHT//2), spawn_cars=True, spawn_cars_timer=car_flow_rate)
                ext2 = RoadExtremity((WIDTH*2, HEIGHT//2), spawn_cars=True, spawn_cars_timer=car_flow_rate)
                ext_3 = RoadExtremity((WIDTH//2, 0), spawn_cars=True, spawn_cars_timer=car_flow_rate)
                ext_4 = RoadExtremity((WIDTH//2, HEIGHT), spawn_cars=True, spawn_cars_timer=car_flow_rate)

                roads_0 = [Road(ext1, intersections_0[0].exits[0]), Road(ext2, intersections_0[0].exits[1]), Road(ext_3, intersections_0[0].exits[2]), Road(ext_4, intersections_0[0].exits[3])]

                road_extremity_spawners_0 = [ext1, ext2, ext_3, ext_4]
                simulator.initialize(intersections_0, roads_0, road_extremity_spawners_0)
            elif event.key == pygame.K_2:
                ext1_1 = RoadExtremity((0, HEIGHT//2), spawn_cars=True, spawn_cars_timer=car_flow_rate)
                ext2_1 = RoadExtremity((WIDTH, HEIGHT//2), spawn_cars=False, spawn_cars_timer=car_flow_rate)
                roads_1 = [Road(ext1_1, ext2_1)]

                road_extremity_spawners_1 = [ext1_1, ext2_1]
                simulator.initialize([], roads_1, road_extremity_spawners_1)
            elif event.key == pygame.K_3:
                n_rows = 2
                m_cols = 2
                intersections_3, roads_3, road_extremity_spawners_3 = create_grid_setup(n_rows, m_cols)
                simulator.initialize(intersections_3, roads_3, road_extremity_spawners_3)


    if simulator:
        simulator.update(dt, events)

    pygame.display.update()
