import pygame
from pygame import Vector2 as Vec2
import sys
import random # For random spawning
from collections import deque
import time
import matplotlib.pyplot as plt
from Constants import *
from Road import Road, RoadExtremity
from Car import Car
from Camera import Camera # Added
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
            img = pygame.Surface((40, 40))
            img.fill((255, 0, 0))
            images.append(img)
        return images

    def __init__(self, win=None):
        Simulator._instance = self
        self.win = win
        from Constants import WIDTH, HEIGHT # Make sure WIDTH and HEIGHT are imported
        self.camera = Camera(WIDTH, HEIGHT) # Added
        self.initialized = False

        # Preload car images once
        self.preloaded_car_images = self._preload_car_images()

        self.debug = False
        self.selected_car = None # Track the currently selected car

        # Congestion tracking
        self.total_cars_spawned_count = 0
        self.total_cars_congested_macro_count = 0
        self.global_congestion_factor = 0.0

        # Font for debug text
        self.font = pygame.font.Font(None, 24)

        self.render_as_rect = False

        # Data recording
        self.car_count_data = []
        self.start_time = time.time()
        self.last_record_time = 0
        self.record_interval = 0.5 # seconds

    def initialize(self, intersections=None, roads=None, road_extremity_spawners=None):
        # --- Your existing setup code ---
        self.intersections = intersections if intersections is not None else []
        self.roads = roads if roads is not None else []
        self.road_extremity_spawners = road_extremity_spawners if road_extremity_spawners is not None else []
        for road_extremity_spawner in self.road_extremity_spawners:
            road_extremity_spawner.simulator = self

        for road in self.roads:
            road.start_extremity.road = road
            road.end_extremity.road = road

        # Collect all extremities for pathfinding reference later if needed
        self.all_extremities = set()
        for road in self.roads:
             self.all_extremities.add(road.start_extremity)
             self.all_extremities.add(road.end_extremity)
        # --- End of existing setup ---


        self._neighbor_map = None

        # Precompute neighbor map for fast pathfinding
        self._build_neighbor_map()

        self.cars = []

        self.initialized = True
    @classmethod
    def get_instance(cls):
        return cls._instance

    def _build_neighbor_map(self):
        """
        Precompute a mapping from each RoadExtremity to its set of neighbor RoadExtremities.
        This should be called after roads and intersections are set up.
        """
        neighbor_map = dict()
        # Collect all extremities
        all_extremities = set()
        for road in self.roads:
            all_extremities.add(road.start_extremity)
            all_extremities.add(road.end_extremity)
        for intersection in self.intersections:
            for exit_extremity in intersection.exits:
                all_extremities.add(exit_extremity)
        # Build neighbors
        for ext in all_extremities:
            neighbors = set()
            # Across the road
            if ext.road:
                other = ext.get_other_extremity()
                if other:
                    neighbors.add(other)
            # Across the intersection
            if ext.intersection:
                for neighbor in ext.intersection.exits:
                    if neighbor is not ext:
                        neighbors.add(neighbor)
            neighbor_map[ext] = neighbors
        self._neighbor_map = neighbor_map
        self._all_extremities = all_extremities

    def rebuild_neighbor_map(self):
        """Public method to rebuild neighbors if roads/intersections change."""
        self._build_neighbor_map()

    def update(self, dt, events):
        if not self.initialized:
            return
        # --- Your existing run loop ---
        for event in events:
            self.camera.handle_event(event) # Added
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_d:
                    self.debug = not self.debug # Toggle debug mode
                if event.key == pygame.K_r:
                    self.render_as_rect = not self.render_as_rect
                if event.key == pygame.K_p:
                    self.plot_car_count()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # Left click
                    world_mouse_pos = self.camera.screen_to_world(pygame.math.Vector2(event.pos)) # Added
                    clicked_car = None
                    for car in self.cars:
                        if car.handle_click(world_mouse_pos): # Modified to use world_mouse_pos
                            clicked_car = car
                            break # Found a clicked car, no need to check others

                    # Deselect the previously selected car
                    if self.selected_car:
                        self.selected_car.selected = False

                    # Select the clicked car if any
                    self.selected_car = clicked_car
                    if self.selected_car:
                        self.selected_car.selected = True


        self.win.fill(BACKGROUND_COLOR)

        for road in self.roads:
            road.update(dt)
            road.draw(self.win)


        for intersection in self.intersections:
            intersection.update(dt)
            intersection.draw(self.win)

        # Use a copy of the list for iteration if cars can be removed during the loop
        for car in list(self.cars):
            car.move(dt)
            car.draw(self.win)

        for road_extremity in self.road_extremity_spawners:
            road_extremity.update(dt)

        # Record car count data
        current_time = time.time()
        if current_time - self.last_record_time > self.record_interval:
            elapsed_time = current_time - self.start_time
            self.car_count_data.append((elapsed_time, len(self.cars)))
            self.last_record_time = current_time

        if self.selected_car and self.debug:
            self.draw_debug_panel()


    def draw_debug_panel(self):
        car = self.selected_car
        debug_info = [
            f"Status: {car.status}",
            f"Speed: {car.speed:.2f}",
            f"Acceleration: {car.acceleration:.3f}",
            f"Max Speed: {car.max_speed}",
            f"Dist Obstacle: {car.check_front()[0]:.2f}",
            f"Dist Exit: {car.distance_on_exit_road}",
            f"Num Cars: {len(self.cars)}",
            f"Can Enter: {car.can_enter_intersection}",
        ]
        if car.current_target_extremity.intersection:
                can_enter = car.current_target_extremity.intersection.can_car_enter(car.current_target_extremity)
                debug_info.append(f"Can Enter Int: {can_enter}")
        else:
                debug_info.append(f"Can Enter Int: N/A")

        # Add global congestion factor to debug panel (or a general info panel)
        debug_info.append(f"Global Congestion: {self.global_congestion_factor:.2f}")


        debug_rect_width = 200
        debug_rect_height = 200 # Increased height for new line
        debug_rect_x = WIDTH - debug_rect_width - 10
        debug_rect_y = 10

        pygame.draw.rect(self.win, (200, 200, 200), (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height))
        pygame.draw.rect(self.win, (0, 0, 0), (debug_rect_x, debug_rect_y, debug_rect_width, debug_rect_height), 2)

        for i, line in enumerate(debug_info):
            text_surface = self.font.render(line, True, (0, 0, 0))
            self.win.blit(text_surface, (debug_rect_x + 10, debug_rect_y + 10 + i * 20))

    def generate_path(self, start_extremity, end_extremity):
        """
        Fast BFS using precomputed neighbor map. Finds shortest path (fewest hops) between extremities.
        Adjusts path for intersection U-turns.
        """
        neighbor_map = self._neighbor_map
        visited = set()
        previous_nodes = dict()
        queue = deque()
        queue.append(start_extremity)
        visited.add(start_extremity)
        found = False
        while queue:
            current_node = queue.popleft()
            if current_node == end_extremity:
                found = True
                break
            for neighbor in neighbor_map.get(current_node, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    previous_nodes[neighbor] = current_node
                    queue.append(neighbor)
        # Reconstruct path
        path_nodes = []
        current = end_extremity
        if not found and current != start_extremity:
            print(f"Warning: Path not found from start {id(start_extremity)} to end {id(end_extremity)}")
            return []
        while current is not None:
            path_nodes.append(current)
            next_node_in_recon = previous_nodes.get(current)
            if next_node_in_recon in path_nodes:
                print(f"Error: Path reconstruction cycle detected near node id={id(current)}")
                return []
            current = next_node_in_recon
        if not path_nodes or path_nodes[-1] != start_extremity:
            print(f"Warning: Path reconstruction failed for start {id(start_extremity)} to end {id(end_extremity)}")
            if path_nodes and path_nodes[-1] != start_extremity:
                print(f"  -> Path ends at {id(path_nodes[-1])} instead of {id(start_extremity)}")
            return []
        ordered_nodes = path_nodes[::-1]
        # Correct Path for Intersection U-Turns (same as before)
        final_path_corrected = []
        if not ordered_nodes:
            return []
        final_path_corrected.append(ordered_nodes[0])
        i = 0
        while i < len(ordered_nodes) - 1:
            prev_node = ordered_nodes[i]
            curr_node = ordered_nodes[i+1]
            final_path_corrected.append(curr_node)
            if i + 2 < len(ordered_nodes):
                next_node = ordered_nodes[i+2]
                if next_node == prev_node and curr_node.intersection is not None:
                    final_path_corrected.append(curr_node)
                    i += 1
            i += 1
        return final_path_corrected


    def car_reached_destination(self, car):
        import time # For current time
        actual_travel_time = time.time() - car.start_time
        min_travel_time = car.calculate_min_travel_time()

        if min_travel_time > 0: # Avoid division by zero if path was instantaneous or invalid
            if actual_travel_time > min_travel_time * MACRO_CONGESTION_THRESHOLD:
                self.total_cars_congested_macro_count += 1
                # print(f"Car {id(car)} was MACRO congested. Actual: {actual_travel_time:.2f}s, Min: {min_travel_time:.2f}s")

        if self.total_cars_spawned_count > 0 : # total_cars_spawned_count is incremented in spawn_car before this can be an issue for a just spawned car
             self.global_congestion_factor = self.total_cars_congested_macro_count / self.total_cars_spawned_count


        # Check if the car object exists in the list before attempting removal
        if car in self.cars:
            self.cars.remove(car)
        # Optional: Clean up the car object if necessary (Python's garbage collector usually handles this)
        # del car
        # print("Car reached destination and was removed.") # For debugging


    def plot_car_count(self):
        if not self.car_count_data:
            print("No data to plot.")
            return

        times, car_counts = zip(*self.car_count_data)
        
        plt.figure()
        plt.plot(times, car_counts)
        plt.xlabel("Time (s)")
        plt.ylabel("Number of Cars")
        plt.title("Number of Cars Over Time")
        plt.grid(True)
        plt.show()

    def spawn_car(self, start_extremity):
        self.total_cars_spawned_count +=1
        # Choose a random end extremity that is NOT the start extremity or on the same road
        possible_ends = [
            ext for ext in self.road_extremity_spawners if ext != start_extremity
        ]

        if not possible_ends:
             # Fallback or specific logic if no suitable random end found
             # For the simple case, let's use the predefined ext2 if available
             end_extremity = self.roads[1].end_extremity if len(self.roads) > 1 else None
             # print("Warning: No random end extremity found, using default.")
             if not end_extremity or end_extremity == start_extremity:
                  print("Error: Cannot determine a valid end extremity for spawning car.")
                  return # Don't spawn if no valid end
        else:
             end_extremity = random.choice(possible_ends)


        # Generate the path for the new car
        path = self.generate_path(start_extremity, end_extremity)

        if path: # Only spawn if a path exists
            car_img = random.choice(self.preloaded_car_images)
            new_car = Car(path, car_img)
            # It seems the Car class already calls generate_path internally,
            # ensure it uses the passed end_extremity or remove the internal call.
            # Let's assume Car uses the provided final_target_extremity.
            self.cars.append(new_car)
            return new_car
            # print(f"Spawned car from {start_extremity.pos} to {end_extremity.pos}") # For debugging
        else:
            print(f"Warning: Could not generate path for new car from {start_extremity.pos} to {end_extremity.pos}. Car not spawned.")
