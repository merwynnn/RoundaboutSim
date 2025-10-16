import pygame
from Simulator import Simulator
from Constants import *
from Road import Road, RoadExtremity
from Car import Car
from Intersections import *
import sys
import matplotlib.pyplot as plt
print("start")
# Pygame setup
pygame.init()

win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Roundabout Simulator")

clock = pygame.time.Clock()

# Font for FPS display
font = pygame.font.Font(None, 30)

simulator = Simulator(win)

car_flow_rate = 120

PAUSE = False

def create_grid_setup(n, m):
    RoadExtremity.next_id = 0
    intersections = []
    roads = []
    road_extremity_spawners = []

    fixed_road_length = 80
    roundabout_radius = 17.5
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
            #intersections.append(RedLightIntersection(pos, [Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)], size=roundabout_radius*2))

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

tick = 0

while True:
    tick+=1
    if tick % 10000 == 0:
        print(f"  ... tick {tick}")
    dt = DT * time_multiplier
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            # Plotting
            plt.figure(figsize=(10, 5))

            plt.subplot(1, 2, 1)
            plt.plot(simulator.exit_flow_rate_history)
            plt.title("Real Entry Flow Rate vs. Time")
            plt.xlabel("Time (ticks)")
            plt.ylabel("Flow Rate")

            plt.subplot(1, 2, 2)
            plt.plot(simulator.car_density_history)
            plt.title("Car Density vs. Time")
            plt.xlabel("Time (ticks)")
            plt.ylabel("Car Density")

            plt.tight_layout()
            plt.show()

            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                PAUSE = not PAUSE
            if event.key == pygame.K_RIGHT:
                if time_multiplier == 1:
                    time_multiplier = 2
                elif time_multiplier == 2:
                    time_multiplier = 3
                else:
                    time_multiplier = 1
            elif event.key == pygame.K_d:
                DEBUG = not DEBUG
                simulator.debug = DEBUG
            elif event.key == pygame.K_1:
                n_rows = 2
                m_cols = 2
                intersections_3, roads_3, road_extremity_spawners_3 = create_grid_setup(n_rows, m_cols)
                simulator.initialize(intersections_3, roads_3, road_extremity_spawners_3, config_file='configs/flow_config_1.xlsx', spawn_intervall_multiplier=0.05)
                tick = 0
    if PAUSE:
        continue

    if simulator:
        simulator.update(dt, events)

    # Display FPS
    clock.tick()
    fps = clock.get_fps()
    fps_text = font.render(f"FPS: {int(fps)}", True, (255, 255, 255)) # White color
    win.blit(fps_text, (10, 10)) # Position at top-left

    pygame.display.update()
