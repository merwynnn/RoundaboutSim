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
    _instance = None

    def _preload_car_images(self):
        """Preload and scale all car images once. Returns a list of surfaces."""
        assets_dir = "Assets"
        images = []
        if os.path.isdir(assets_dir):
            for fname in os.listdir(assets_dir):
                fpath = os.path.join(assets_dir, fname)
                try:
                    img = pygame.image.load(fpath)
                    img = pygame.transform.scale(img, (40, 40))
                    images.append(img)
                except Exception as e:
                    print(f"Warning: Could not load car image {fpath}: {e}")
        if not images:
            # Fallback: create a dummy surface
            img = pygame.Surface((int(40 / 8), int(40 / 8)))
            img.fill((255, 0, 0))
            images.append(img)
        return images

    def __init__(self, win=None, use_gui=True):
        Simulator._instance = self
        self.use_gui = use_gui
        self.win = win if self.use_gui else None

        from Constants import WIDTH, HEIGHT  # Make sure WIDTH and HEIGHT are imported
        self.camera = Camera(WIDTH, HEIGHT)  # Added
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
        self.car_lifetimes = []

        self.car_density_history = []
        self.exit_flow_rate_history = []

        self.energy_consumption = 0

    def get_average_car_lifetime(self):
        if not self.car_lifetimes:
            return 0
        return sum(self.car_lifetimes) / len(self.car_lifetimes)

    def get_car_density(self):
        return len(self.cars)

    def get_entry_flow_rate(self):
        total_interval = 0
        spawner_count = 0
        for spawner in self.road_extremity_spawners:
            if spawner.spawn_cars:
                total_interval += spawner.spawn_cars_timer
                spawner_count += 1
        if spawner_count == 0:
            return 0
        avg_interval = total_interval / spawner_count
        return 1 / avg_interval if avg_interval > 0 else 0

    def get_real_entry_flow_rate(self):
        return self.total_cars_spawned_count / self.total_ticks

    def get_exit_flow_rate(self):
        return self.last_exit_flow_rate

    def get_mean_car_density(self):
        if not self.car_density_history:
            return 0
        return sum(self.car_density_history) / len(self.car_density_history)

    def get_mean_real_entry_flow_rate(self):
        if not self.real_entry_flow_rate_history:
            return 0
        return sum(self.real_entry_flow_rate_history) / len(
            self.real_entry_flow_rate_history)

    def initialize(self,
                   intersections=None,
                   roads=None,
                   road_extremity_spawners=None,
                   car_spawn_interval=60.0,
                   road_extremity_exits=None):
        print("init")
        self.total_ticks = 0
        self.car_lifetimes = []

        self.car_density_history = []
        self.real_entry_flow_rate_history = []

        # --- Your existing setup code ---
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

    @classmethod
    def get_instance(cls):
        return cls._instance

    def update(self, dt, events):
        if not self.initialized:
            return

        self.total_ticks += 1

        self.car_density_history.append(self.get_car_density())
        if self.total_ticks > 0:
            self.real_entry_flow_rate_history.append(self.get_exit_flow_rate())

        if self.total_ticks % 20 == 0:  # Every 20 ticks
            for i in range(len(self.cars_exited_tick_during_delta)):
                if self.cars_exited_tick_during_delta[
                        0] < self.total_ticks - self.flow_rate_time_delta:
                    self.cars_exited_tick_during_delta.pop(0)
                else:
                    break

            self.last_exit_flow_rate = len(
                self.cars_exited_tick_during_delta) / self.flow_rate_time_delta
            self.exit_flow_rate_history.append(self.last_exit_flow_rate)

        # Update simulation logic
        for road in self.roads:
            road.update(dt)
        for intersection in self.intersections:
            intersection.update(dt)
        for car in list(self.cars):
            car.move(dt)
            self.energy_consumption += CAR_WEIGHT * max(car.acceleration, 0) * car.speed * dt  # energy consumption

        for road_extremity in self.road_extremity_spawners:
            road_extremity.update(dt)

        if not self.use_gui:
            return

        # --- Handle GUI (events, rendering) ---
        for event in events:
            self.camera.handle_event(event)  # Added
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_d:
                    self.debug = not self.debug  # Toggle debug mode
                if event.key == pygame.K_r:
                    self.render_as_rect = not self.render_as_rect
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    world_mouse_pos = self.camera.screen_to_world(
                        pygame.math.Vector2(event.pos))  # Added
                    clicked_car = None
                    for car in self.cars:
                        if car.handle_click(
                                world_mouse_pos
                        ):  # Modified to use world_mouse_pos
                            clicked_car = car
                            break  # Found a clicked car, no need to check others

                    # Deselect the previously selected car
                    if self.selected_car:
                        self.selected_car.selected = False

                    # Select the clicked car if any
                    self.selected_car = clicked_car
                    if self.selected_car:
                        self.selected_car.selected = True

        self.win.fill(BACKGROUND_COLOR)

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
            f"Exit Flow Rate: {self.last_exit_flow_rate*360:.3f}",
            f"Car Density: {self.get_car_density()}",
            f"Mean Car Density: {self.get_mean_car_density():.2f}",
            f"Mean Exit Flow Rate: {mean_exit_flow_rate}",
            f"Total time: {self.total_ticks*0.1:.1f}s",
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
        # Check if the car object exists in the list before attempting removal
        if car in self.cars:
            self.total_cars_exited += 1
            self.cars_exited_tick_during_delta.append(self.total_ticks)
            print(
                f"Car exited.Total energy consumption mean: {self.energy_consumption/self.total_cars_exited:.2f}"
            )

            self.cars.remove(car)
            lifetime = self.total_ticks - car.creation_tick
            self.car_lifetimes.append(lifetime)
        # Optional: Clean up the car object if necessary (Python's garbage collector usually handles this)
        # del car
        # print("Car reached destination and was removed.") # For debugging

    def spawn_car(self, start_extremity):
        self.total_cars_spawned_count += 1

        # Randomly select an end extremity from the list of exits
        end_extremity = random.choice(self.road_extremity_exits)

        # Generate the path for the new car
        path = self.generate_path(start_extremity, end_extremity)

        if path:  # Only spawn if a path exists
            car_img = random.choice(
                self.preloaded_car_images
            ) if self.use_gui and self.preloaded_car_images else None
            new_car = Car(path, self.total_ticks, car_img)
            # It seems the Car class already calls generate_path internally,
            # ensure it uses the passed end_extremity or remove the internal call.
            # Let's assume Car uses the provided final_target_extremity.
            self.cars.append(new_car)

            return new_car
            # print(f"Spawned car from {start_extremity.pos} to {end_extremity.pos}") # For debugging
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
        new_car.status = "INTERSECTION"

        self.cars.append(new_car)

        return new_car
