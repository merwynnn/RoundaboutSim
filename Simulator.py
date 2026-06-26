import pygame
from pygame import Vector2 as Vec2
import sys
import random  # For random spawning
from collections import deque
from Constants import *
from Road import Road, RoadExtremity
from Car import Car
from Camera import Camera  # Added
from Intersections import *
import os


class Simulator:
    def __init__(self, win=None, use_gui=True, on_car_spawned=lambda car: None):
        Simulator._instance = self
        self.use_gui = use_gui
        self.win = win if self.use_gui else None

        self.camera = Camera(WIDTH, HEIGHT)

        # Preload car images once
        self.preloaded_car_images = self._preload_car_images()

        self.initialized = False

        self.flow_manager = None

        self.debug = False
        self.selected_car = None  # Track the currently selected car

        self.render_as_rect = False

        self.total_cars_spawned_count = 0
        self.total_cars_exited = 0

        self.flow_rate_time_delta = 120
        self.cars_exited_tick_during_delta = []
        self.last_exit_flow_rate = 0
        self.last_exited_car_tick = 0

        self.total_ticks = 0
        self.completion_times = []

        self.car_density_history = []
        self.exit_flow_rate_history = []

        self.energy_consumption = 0

        self.on_car_spawned = on_car_spawned  # can be set externally for custom behavior on car spawn
    def get_average_completion_time(self):
        if not self.completion_times:
            return 0
        return sum(self.completion_times) / len(self.completion_times)
    def get_car_density(self):
        return len(self.cars)
    def get_mean_car_density(self):
        if not self.car_density_history:
            return 0
        return sum(self.car_density_history) / len(self.car_density_history)
    def initialize(self,
                   intersections=None,
                   roads=None,
                   road_extremity_spawners=None,
                   car_spawn_interval=60.0,
                   road_extremity_exits=None):
        self.total_ticks = 0
        self.completion_times = []

        self.car_density_history = []
        self.real_entry_flow_rate_history = []

        self.intersections = intersections if intersections is not None else []
        self.roads = roads if roads is not None else []
        self.road_extremity_spawners = road_extremity_spawners if road_extremity_spawners is not None else []
        self.road_extremity_exits = road_extremity_exits if road_extremity_exits is not None else []

        for spawner in self.road_extremity_spawners:
            spawner.spawn_cars_timer = car_spawn_interval

        for road in self.roads:
            road.start_extremity.road = road
            road.end_extremity.road = road

        # Collect all extremities for pathfinding reference later if needed
        self.all_extremities = set()
        for road in self.roads:
            self.all_extremities.add(road.start_extremity)
            self.all_extremities.add(road.end_extremity)

        self.camera.position = Vec2(
            self.intersections[0].pos if self.intersections else self.roads[0].
            start_extremity.pos if self.roads else Vec2(0, 0))

        self._neighbor_map = None

        self.cars = []

        self.initialized = True

        self.energy_consumption = 0

    def update(self, dt, events):
        if not self.initialized:
            return

        self.total_ticks += 1

        # Update simulation logic
        for road in self.roads:
            road.update(dt)
        for intersection in self.intersections:
            intersection.update(dt)
        for car in list(self.cars):
            car.move(dt)
            # Computes energy consumption
            self.energy_consumption += CAR_WEIGHT * max(car.acceleration, 0) * car.speed * dt  

        for road_extremity in self.road_extremity_spawners:
            road_extremity.update(dt)

        if not self.use_gui:
            return

        # --- Handle GUI (events, rendering) ---
        #...

        for road in self.roads:
            road.draw(self.win)

        for intersection in self.intersections:
            intersection.draw(self.win)

        # Use a copy of the list for iteration if cars can be removed during the loop
        for car in self.cars:
            car.draw(self.win)

        if self.selected_car and self.debug:
            self.draw_debug_panel()

        if self.debug:
            self.draw_global_debug_panel()

    def draw_debug_panel(self):
        car = self.selected_car
        debug_info = [
            f"Acceleration: {car.acceleration:.3f}",
            f"Target Speed: {car.target_speed*3.6:.2f}",
            f"Dist Obstacle: {car.check_front()[0]:.2f}",
            f"Can Enter: {car.can_enter_intersection}", f"Energy Consumption: {self.energy_consumption:.2f}"
        ]

        debug_rect_width = 200
        debug_rect_height = 180
        debug_rect_x = WIDTH - debug_rect_width - 10
        debug_rect_y = 10

        pygame.draw.rect(
            self.win, (200, 200, 200),
            (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height))
        pygame.draw.rect(
            self.win, (0, 0, 0),
            (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height),
            2)

        for i, line in enumerate(debug_info):
            text_surface = font.render(line, True, (0, 0, 0))
            self.win.blit(text_surface,
                          (debug_rect_x + 10, debug_rect_y + 10 + i * 20))

    def draw_global_debug_panel(self):
        mean_exit_flow_rate = (sum(self.exit_flow_rate_history) /
                               len(self.exit_flow_rate_history)) if len(
                                   self.exit_flow_rate_history) > 0 else 0
        debug_info = [
            f"Total time: {self.total_ticks*DT:.1f}s",
            f"Mean speed: {sum(car.speed for car in self.cars)/len(self.cars)*3.6:.2f} km/h" if self.cars else "Mean speed: N/A",
            f"Average completion time: {self.get_average_completion_time():.2f}s",
        ]

        debug_rect_width = 300
        debug_rect_height = 120
        debug_rect_x = 10
        debug_rect_y = 10

        pygame.draw.rect(
            self.win, (200, 200, 200),
            (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height))
        pygame.draw.rect(
            self.win, (0, 0, 0),
            (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height),
            2)

        for i, line in enumerate(debug_info):
            text_surface = font.render(line, True, (0, 0, 0))
            self.win.blit(text_surface,
                          (debug_rect_x + 10, debug_rect_y + 10 + i * 20))

    def generate_path(self, start_extremity, end_extremity):
        path = [
            start_extremity,
            start_extremity.get_other_extremity(),
            end_extremity.get_other_extremity(), end_extremity
        ]

        return path

    def car_reached_destination(self, car):
        if car in self.cars:
            self.total_cars_exited += 1
            self.cars_exited_tick_during_delta.append(self.total_ticks)
            self.cars.remove(car)

            #Computes car crossing time
            lifetime_ticks = self.total_ticks - car.creation_tick
            lifetime_seconds = lifetime_ticks * DT
            self.completion_times.append(lifetime_seconds)

    def spawn_car(self, start_extremity):
        self.total_cars_spawned_count += 1

        # Makes sure only the furthest exit is selected
        end_extremity = self.road_extremity_exits[0] if (self.road_extremity_exits[0].pos - start_extremity.pos).length() < (self.road_extremity_exits[1].pos - start_extremity.pos).length() else self.road_extremity_exits[1]

        # Generate the path for the new car
        path = self.generate_path(start_extremity, end_extremity)


        if path:  # Only spawn if a path exists
            car_img = random.choice(
                self.preloaded_car_images
            ) if self.use_gui and self.preloaded_car_images else None
            new_car = Car(path, self.total_ticks, car_img)

            self.cars.append(new_car)

            self.on_car_spawned(new_car)  # Call the callback for any additional setup

            return new_car
        else:
            print(
                f"Warning: Could not generate path for new car from {start_extremity.pos} to {end_extremity.pos}. Car not spawned."
            )

    def spawn_car_at_position(self, position, direction, intersection,
                              target_position, target_index):
        self.total_cars_spawned_count += 1

        car_img = random.choice(
            self.preloaded_car_images
        ) if self.use_gui and self.preloaded_car_images else None
        new_car = Car([],
                      self.total_ticks,
                      car_img,
                      target_position=target_position, id = len(self.cars))
        new_car.current_target_index = target_index

        new_car.pos = position
        new_car.dir = direction.normalize()

        dummy_extremity = RoadExtremity(position)
        dummy_extremity.intersection = intersection

        new_car.last_extremity = dummy_extremity  # Assign to an arbitrary exit for now
        new_car.status = "IN_RING_ROAD"

        self.cars.append(new_car)

        self.on_car_spawned(new_car)  # Call the callback for any additional setup

        return new_car
