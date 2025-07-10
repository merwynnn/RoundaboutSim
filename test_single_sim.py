from Simulator import Simulator
from Constants import * # Make sure Vec2 is available if not already from other imports
from Road import Road, RoadExtremity
from Intersections import * # Make sure ClassicRoundabout and other intersection types are imported
import numpy as np
import os

# --- Simulation Setup (Copied from main.py) ---
# Ensure Vec2 is correctly imported or defined if it's part of Constants or another module
# If Vec2 comes from pygame, it might be an issue for non-GUI runs if not handled carefully.
# For this test, we assume Simulator handles Pygame initialization conditionally.
# If pygame.Vector2 is used directly in create_grid_setup, we need:
try:
    from pygame import Vector2 as Vec2
except ImportError:
    # Fallback or error if pygame is not available and Vec2 is needed here.
    # This script is intended for non-GUI testing, but setup might still use Vec2.
    print("Warning: Pygame not found, Vec2 might be unavailable for setup if used directly.")
    # A simple Vec2 placeholder if absolutely necessary for setup structure,
    # but ideally, setup logic avoids direct Pygame dependencies if run without GUI.
    class Vec2:
        def __init__(self, x, y): self.x = x; self.y = y
        def __add__(self, other): return Vec2(self.x + other.x, self.y + other.y)
        def __sub__(self, other): return Vec2(self.x - other.x, self.y - other.y)
        def __mul__(self, scalar): return Vec2(self.x * scalar, self.y * scalar)
        def length(self): return (self.x**2 + self.y**2)**0.5
        def normalize(self):
            l = self.length()
            if l == 0: return Vec2(0,0)
            return Vec2(self.x/l, self.y/l)
        def rotate(self, angle_degrees): return self # Placeholder
        def angle_to(self, other): return 0 # Placeholder


def create_grid_setup(n, m, car_flow_rate):
    RoadExtremity.next_id = 0 # Reset IDs for consistent runs
    intersections = []
    roads = []
    road_extremity_spawners = []

    fixed_road_length = 500
    roundabout_radius = 100
    spacing_between_centers = fixed_road_length + 2 * roundabout_radius

    for i in range(n):
        for j in range(m):
            pos_x = (j * spacing_between_centers) + spacing_between_centers
            pos_y = (i * spacing_between_centers) + spacing_between_centers
            pos = (int(pos_x), int(pos_y))
            # Assuming ClassicRoundabout is defined and imported
            intersections.append(ClassicRoundabout(pos, roundabout_radius, [Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)]))

    for i in range(n):
        for j in range(m):
            current_intersection_index = i * m + j
            current_intersection = intersections[current_intersection_index]
            if j < m - 1:
                right_intersection_index = i * m + (j + 1)
                right_intersection = intersections[right_intersection_index]
                roads.append(Road(current_intersection.exits[1], right_intersection.exits[0]))
                roads.append(Road(right_intersection.exits[0], current_intersection.exits[1]))
            if i < n - 1:
                below_intersection_index = (i + 1) * m + j
                below_intersection = intersections[below_intersection_index]
                roads.append(Road(current_intersection.exits[3], below_intersection.exits[2]))
                roads.append(Road(below_intersection.exits[2], current_intersection.exits[3]))

    # Edge spawners
    for j in range(m):
        center_x_col_j = (j * spacing_between_centers) + spacing_between_centers
        ext_pos_top = (int(center_x_col_j), int(((0 * spacing_between_centers) + spacing_between_centers) - roundabout_radius - fixed_road_length))
        ext_top = RoadExtremity(ext_pos_top, spawn_cars=True, spawn_cars_timer=car_flow_rate) # car_flow_rate used as placeholder
        road_extremity_spawners.append(ext_top)
        roads.append(Road(ext_top, intersections[j].exits[2]))

        ext_pos_bottom = (int(center_x_col_j), int((((n - 1) * spacing_between_centers) + spacing_between_centers) + roundabout_radius + fixed_road_length))
        ext_bottom = RoadExtremity(ext_pos_bottom, spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext_bottom)
        roads.append(Road(intersections[(n - 1) * m + j].exits[3], ext_bottom))

    for i in range(n):
        center_y_row_i = (i * spacing_between_centers) + spacing_between_centers
        ext_pos_left = (int((((0 * spacing_between_centers) + spacing_between_centers) - roundabout_radius) - fixed_road_length), int(center_y_row_i))
        ext_left = RoadExtremity(ext_pos_left, spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext_left)
        roads.append(Road(ext_left, intersections[i * m].exits[0]))

        ext_pos_right = (int((((m - 1) * spacing_between_centers) + spacing_between_centers) + roundabout_radius + fixed_road_length), int(center_y_row_i))
        ext_right = RoadExtremity(ext_pos_right, spawn_cars=True, spawn_cars_timer=car_flow_rate)
        road_extremity_spawners.append(ext_right)
        roads.append(Road(intersections[i * m + (m - 1)].exits[1], ext_right))

    return intersections, roads, road_extremity_spawners

# --- Adapted Simulation Runner ---
def run_single_test_simulation(multiplier, config_file_path):
    print(f"--- Test Simulation ---")
    print(f"Multiplier: {multiplier:.2f}, Config: {config_file_path}")

    # Parameters for the test simulation
    n_rows_test = 1 # Smaller grid for faster testing
    m_cols_test = 1
    initial_car_flow_rate = 120 # As in main.py, but actual flow is modified by FlowManager + multiplier

    # Create a new simulator instance for this test run
    # Important for multiprocessing later: each process should have its own simulator
    test_simulator = Simulator(win=None, use_gui=False)

    intersections, roads, road_extremity_spawners = create_grid_setup(n_rows_test, m_cols_test, initial_car_flow_rate)

    # Initialize the simulator
    # The spawn_intervall_multiplier in FlowManager will use the 'multiplier' argument
    test_simulator.initialize(intersections, roads, road_extremity_spawners,
                              config_file=config_file_path,
                              spawn_intervall_multiplier=multiplier)

    max_ticks_test = 2000  # Reduced ticks for faster test
    stability_ticks_test = 200 # Reduced stability check window
    car_counts = []
    # last_car_counts = [] # Not strictly needed for basic test output, can be added for stability logic

    print(f"Running simulation for {max_ticks_test} ticks...")
    for tick in range(max_ticks_test):
        if tick % 500 == 0: # Print progress less frequently
            print(f"  ... tick {tick}/{max_ticks_test}")

        test_simulator.update(dt=15, events=[]) # dt=15 as in main

        if tick % 100 == 0: # Sample car count
            current_car_count = test_simulator.get_car_density()
            car_counts.append(current_car_count)
            # Basic stability/jam check (optional for simple test, can be refined)
            # if len(last_car_counts) > stability_ticks_test:
            #     last_car_counts.pop(0)
            #     if np.std(last_car_counts) < 1.0 and current_car_count > (n_rows_test * m_cols_test * 10): # Arbitrary jam threshold
            #         print(f"Potential jam/stability detected at tick {tick}.")
            #         break
            # last_car_counts.append(current_car_count)


    exit_flow_rate = test_simulator.get_exit_flow_rate()
    mean_density = np.mean(car_counts) if car_counts else 0

    print(f"--- Test Simulation Results ---")
    print(f"  Total Ticks Run: {test_simulator.total_ticks}")
    print(f"  Total Cars Spawned: {test_simulator.total_cars_spawned_count}")
    print(f"  Total Cars Exited: {test_simulator.total_cars_exited}")
    print(f"  Calculated Exit Flow Rate: {exit_flow_rate:.4f} (cars/tick)")
    print(f"  Mean Car Density: {mean_density:.4f} (cars)")
    print(f"  Final Car Count: {test_simulator.get_car_density()}")

    return mean_density, exit_flow_rate

# --- Main execution for the test script ---
if __name__ == "__main__":
    # Test parameters
    test_spawn_multiplier = 1.0  # A single multiplier for testing
    # Use the root flow_config.xlsx file as requested
    test_config_file = "flow_config.xlsx"

    if not os.path.exists(test_config_file):
        print(f"ERROR: Test config file '{test_config_file}' not found in the root directory.")
        print("Please ensure 'flow_config.xlsx' is present in the repository root.")
    else:
        run_single_test_simulation(test_spawn_multiplier, test_config_file)
        print("\nTest script finished.")
        # Example of how to check Numba's JIT compilation status if interested
        # from numba.core.runtime import rtsys
        # print(f"Numba JIT functions compiled: {rtsys.get_ptds_count()}")
        # Note: This Numba status check might be advanced or require specific Numba versions.
        # A simpler check is just to ensure it runs without Numba-related errors.
        print("If Numba is working, you should not see Numba compilation errors during the first run.")
        print("Subsequent runs should be faster for Numba-jitted functions due to caching.")

print("test_single_sim.py loaded")
# Ensure necessary modules like FlowManager, Car, etc., are correctly imported by Simulator
# and other components. This script assumes the standard project structure.
