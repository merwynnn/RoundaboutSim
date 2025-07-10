from Simulator import Simulator
from Constants import *
from Road import Road, RoadExtremity
from Intersections import *
import sys
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation # Not used directly in this version
import os
import multiprocessing # Added for multiprocessing

# --- Simulation Setup ---
# use_gui is implicitly False for multiprocessing script.
# Simulator instances will be created with use_gui=False in each process.

# Need to ensure Vec2 is available. Assuming it's part of Constants or handled by Simulator's Pygame init.
# If create_grid_setup directly uses pygame.Vector2, it might need adjustment or
# ensure pygame is importable in the fork context (usually it is).
try:
    from pygame import Vector2 as Vec2
except ImportError:
    print("Warning: Pygame not found, Vec2 might be unavailable for setup if used directly by create_grid_setup.")
    # Basic Vec2 placeholder if create_grid_setup absolutely needs it and pygame isn't available
    # This is unlikely to be hit if Simulator handles Pygame init.
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


def create_grid_setup(n, m, car_flow_rate):
    RoadExtremity.next_id = 0
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
        ext_top = RoadExtremity(ext_pos_top, spawn_cars=True, spawn_cars_timer=car_flow_rate)
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

# --- Experiment Parameters ---
n_rows = 2
m_cols = 2

# --- Matplotlib Setup ---
plt.ion()
fig, ax = plt.subplots()
ax.set_ylabel("Flow Rate (Cars Exiting per tick)")
ax.set_xlabel("Mean Density (cars)")
ax.set_title("Global Flow Rate vs. Mean Density for Different Configurations")
ax.grid(True)
ax.set_xlim(0, 200) # Adjust as needed
ax.set_ylim(0, 0.2) # Adjust as needed

# Removed global simulator instance: simulator = Simulator(win=None, use_gui=use_gui)

def run_simulation_for_multiplier(args):
    """
    Worker function for multiprocessing.
    Each call to this function runs one simulation instance in a separate process.
    """
    multiplier, config_file_path, n_rows_sim, m_cols_sim = args
    
    # Process ID for logging, helps distinguish outputs from different processes
    pid = os.getpid()
    print(f"[PID {pid}] Testing with multiplier: {multiplier:.2f} and config: {config_file_path}")

    # Each process creates its own Simulator instance. use_gui is False.
    local_simulator = Simulator(win=None, use_gui=False)

    # Reset RoadExtremity ID counter for this process to ensure consistency if objects are re-created.
    # This is important because each process will call create_grid_setup independently.
    RoadExtremity.next_id = 0

    intersections, roads, road_extremity_spawners = create_grid_setup(n_rows_sim, m_cols_sim, car_flow_rate=120) # Using passed n_rows, m_cols
    local_simulator.initialize(intersections, roads, road_extremity_spawners,
                               config_file=config_file_path,
                               spawn_intervall_multiplier=multiplier)

    # Simulation parameters from original script
    max_ticks = 20000  # Original: 20000
    stability_ticks = 1000 # Original: 1000
    car_counts = []
    last_car_counts = []

    traffic_jam = False
    
    for tick in range(max_ticks):
        if tick % 5000 == 0: # Reduce print frequency for parallel runs
            print(f"  [PID {pid}] ... tick {tick}/{max_ticks}")
        local_simulator.update(dt=15, events=[]) # Use local_simulator
        
        if tick % 500 == 0: # Sample car count as before
            current_car_count = local_simulator.get_car_density() # Use local_simulator
            car_counts.append(current_car_count)
            last_car_counts.append(current_car_count) # For stability check
            if len(last_car_counts) > stability_ticks / 500: # Adjusted for sampling rate
                last_car_counts.pop(0)
                if np.std(last_car_counts) < 1.5 and len(car_counts) > (stability_ticks / 500): # Ensure enough samples for std dev
                    print(f"  [PID {pid}] System stable after {tick} ticks with car count std dev = {np.std(last_car_counts):.2f}")
                    traffic_jam = True
                    break
    
    if traffic_jam:
        print(f"  [PID {pid}] Traffic jam detected.")
        # Return config_file_path and multiplier to associate results later
        return config_file_path, multiplier, local_simulator.get_car_density(), 0

    # else: # Not needed, print is conditional
    #     print(f"  [PID {pid}] No traffic jam detected.")
    
    exit_flow_rate = local_simulator.get_exit_flow_rate() # Use local_simulator
    mean_density = np.mean(car_counts) if car_counts else 0

    print(f"  [PID {pid}] -> Flow Rate: {exit_flow_rate:.4f}, Mean Density: {mean_density:.4f} for {config_file_path} mult {multiplier}")
    # Return config_file_path and multiplier to associate results
    return config_file_path, multiplier, mean_density, exit_flow_rate


# --- Main Experiment Loop ---
if __name__ == "__main__":
    # Moved spawn_multipliers and config_files loading inside main block
    spawn_multipliers = np.arange(1.5, 0, -0.15)
    config_files_names = [f for f in os.listdir('configs') if f.endswith('.xlsx')]
    
    # Prepare list of arguments for each simulation task
    tasks = []
    for config_name in config_files_names:
        config_path = os.path.join('configs', config_name)
        for mult in spawn_multipliers:
            tasks.append((mult, config_path, n_rows, m_cols)) # Pass n_rows, m_cols

    print(f"Total simulation tasks to run: {len(tasks)}")

    # Use multiprocessing Pool
    # Adjust number of processes as needed, None usually defaults to cpu_count()
    num_processes = multiprocessing.cpu_count()
    print(f"Starting simulations with {num_processes} processes...")

    results_from_pool = []
    # Using try/finally to ensure pool is closed
    pool = multiprocessing.Pool(processes=num_processes)
    try:
        results_from_pool = pool.map(run_simulation_for_multiplier, tasks)
    finally:
        pool.close()
        pool.join()

    print("\nAll simulations finished. Aggregating and plotting results...")

    # --- Process and Plot Results ---
    # Reorganize results for plotting
    # results_by_config will store: {config_file_path: [(mean_density, exit_flow_rate), ...]}
    # Order by multiplier for correct line plotting.
    results_by_config = {}
    for res_config_path, res_multiplier, res_mean_density, res_exit_flow in results_from_pool:
        if res_config_path not in results_by_config:
            results_by_config[res_config_path] = []
        results_by_config[res_config_path].append({'multiplier': res_multiplier,
                                                   'density': res_mean_density,
                                                   'flow': res_exit_flow})

    # Plotting
    colors = plt.cm.jet(np.linspace(0, 1, len(config_files_names)))

    for i, config_name in enumerate(config_files_names):
        config_path_key = os.path.join('configs', config_name)
        if config_path_key in results_by_config:

            # Sort data by multiplier to ensure lines are plotted correctly
            sorted_data = sorted(results_by_config[config_path_key], key=lambda x: x['multiplier'], reverse=True)

            plot_densities = [item['density'] for item in sorted_data if item['flow'] > 0 or item['density'] > 0]
            plot_flows = [item['flow'] for item in sorted_data if item['flow'] > 0 or item['density'] > 0]

            if plot_densities and plot_flows: # Only plot if there's valid data
                # Original plot used exit_flow_rates on x-axis and mean_densities on y-axis.
                # Let's match that: ax.plot(exit_flow_rates, mean_densities, ...)
                # So, x-axis is flow, y-axis is density.
                ax.plot(plot_flows, plot_densities, marker='o', linestyle='-', color=colors[i], label=f'Config: {config_name}')
        else:
            print(f"No results found for config: {config_name}")


    print("Plotting complete.")
    plt.ioff() # Turn off interactive mode
    plt.legend()
    # Ensure the plot file name matches the original or is updated if needed
    plt.savefig("flow_vs_density_plot_all_configs_mp.png")
    print("Plot saved to flow_vs_density_plot_all_configs_mp.png")
    # plt.show() # Typically not used in automated scripts, but can be uncommented for interactive display
