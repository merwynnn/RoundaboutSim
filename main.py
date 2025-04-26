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
    grid_size_x = WIDTH / (m + 1)
    grid_size_y = HEIGHT / (n + 1)

    # Create intersections
    for i in range(n):
        for j in range(m):
            pos = (int((j + 1) * grid_size_x), int((i + 1) * grid_size_y))
            # Assuming a simple intersection type for now, adjust as needed
            intersections.append(ClassicRoundabout(pos, 70, [Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)]))

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
        ext = RoadExtremity((int((j + 1) * grid_size_x), 0), spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(ext, intersections[j].exits[2])) # Connect extremity to top entry of first row

    # Bottom edge
    for j in range(m):
        ext = RoadExtremity((int((j + 1) * grid_size_x), HEIGHT), spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(intersections[(n-1)*m + j].exits[3], ext)) # Connect bottom exit of last row to extremity

    # Left edge
    for i in range(n):
        ext = RoadExtremity((0, int((i + 1) * grid_size_y)), spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(ext, intersections[i*m].exits[0])) # Connect extremity to left entry of first column

    # Right edge
    for i in range(n):
        ext = RoadExtremity((WIDTH, int((i + 1) * grid_size_y)), spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext)
        roads.append(Road(intersections[i*m + (m-1)].exits[1], ext)) # Connect right exit of last column to extremity


    return intersections, roads, road_extremity_spawners


# Font for debug text
font = pygame.font.Font(None, 24)

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

                ext1 = RoadExtremity((0, HEIGHT//2), spawn_cars=True, spawn_cars_timer=car_flow_rate)
                ext2 = RoadExtremity((WIDTH, HEIGHT//2), spawn_cars=True, spawn_cars_timer=car_flow_rate)
                ext_3 = RoadExtremity((WIDTH//2, 0), spawn_cars=True, spawn_cars_timer=car_flow_rate)
                ext_4 = RoadExtremity((WIDTH//2, HEIGHT), spawn_cars=False, spawn_cars_timer=car_flow_rate)

                roads_0 = [Road(ext1, intersections_0[0].exits[0]), Road(intersections_0[0].exits[1], ext2), Road(ext_3, intersections_0[0].exits[2]), Road(ext_4, intersections_0[0].exits[3])]

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

        # Draw debug info for selected car
        if simulator.selected_car and simulator.debug:
            car = simulator.selected_car
            debug_info = [
                f"Status: {car.status}",
                f"Speed: {car.speed:.2f}",
            ]
            if car.status == "APPROACHING" and car.current_target_extremity.intersection:
                 can_enter = car.current_target_extremity.intersection.can_car_enter(car.current_target_extremity)
                 debug_info.append(f"Can Enter: {can_enter}")
            else:
                 debug_info.append(f"Can Enter: None")


            debug_rect_width = 200
            debug_rect_height = 80
            debug_rect_x = WIDTH - debug_rect_width - 10
            debug_rect_y = 10

            pygame.draw.rect(win, (200, 200, 200), (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height))
            pygame.draw.rect(win, (0, 0, 0), (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height), 2)

            for i, line in enumerate(debug_info):
                text_surface = font.render(line, True, (0, 0, 0))
                win.blit(text_surface, (debug_rect_x + 10, debug_rect_y + 10 + i * 20))


    pygame.display.update()
