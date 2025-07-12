import multiprocessing
from Simulator import Simulator
from Constants import *
from Road import Road, RoadExtremity
from Intersections import *
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os # Import os module

# --- Simulation Setup ---
use_gui = False  # Set to False for batch processing

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

def run_simulation_for_multiplier(args):
    multiplier, config_file_path, n_rows, m_cols, use_gui = args
    print(f"Testing with multiplier: {multiplier:.2f} and config: {config_file_path}")
    
    simulator = Simulator(win=None, use_gui=use_gui)
    intersections, roads, road_extremity_spawners = create_grid_setup(n_rows, m_cols, car_flow_rate=120)
    simulator.initialize(intersections, roads, road_extremity_spawners, config_file=config_file_path, spawn_intervall_multiplier=multiplier)

    max_ticks = 5000
    stability_ticks = 1000
    car_counts = []
    last_car_counts = []

    flow_rates = []

    traffic_jam = False
    
    for tick in range(max_ticks):
        if tick % 3000 == 0:
            print(f"  ... tick {tick}/{max_ticks}")
        simulator.update(dt=15, events=[])
        
        if tick % 100 == 0:
            current_car_count = simulator.get_car_density()
            car_counts.append(current_car_count)
            flow_rate = simulator.get_real_entry_flow_rate()
            flow_rates.append(flow_rate)
            last_car_counts.append(current_car_count)
            if len(last_car_counts) > stability_ticks:
                last_car_counts.pop(0)
                if np.std(last_car_counts) < 1.5:
                    print(f"System stable after {tick} ticks with car count std dev = {np.std(last_car_counts):.2f}")
                    traffic_jam = True
                    break
    
    if traffic_jam:
        print(f"Traffic jam detected.")
        return simulator.get_car_density(), 0

    else:
        print(f"No traffic jam detected.")
    mean_flow_rate = (np.mean(flow_rates) if flow_rates else 0)*60*60
    mean_exit_flow_rate = (sum(simulator.exit_flow_rate_history)/len(simulator.exit_flow_rate_history))*60*60
    mean_density = np.mean(car_counts) if car_counts else 0

    flow_rate = simulator.get_real_entry_flow_rate()*60*60
    
    print(f"  -> Mean Entry Flow Rate: {flow_rate:.4f}, Mean Exit Flow Rate: {mean_exit_flow_rate:.4f}, Flow Rate: {flow_rate:.4f}, Mean Density: {mean_density:.4f}")
    return mean_density, mean_exit_flow_rate

def main():
    # --- Experiment Parameters ---
    n_rows = 2
    m_cols = 2

    # --- Matplotlib Setup ---
    fig, ax = plt.subplots()
    ax.set_ylabel("Flow Rate (Cars Exiting per tick)")
    ax.set_xlabel("Mean Density (cars)")
    ax.set_title("Global Flow Rate vs. Mean Density for Different Configurations")
    ax.grid(True)
    ax.set_xlim(0, 200) # Adjust as needed
    ax.set_ylim(0, 400) # Adjust as needed

    # --- Main Experiment Loop ---
    spawn_multipliers = list(np.arange(5, 2, -1)) + list(np.arange(2, 1, -0.5)) + list(np.arange(1, 0.3, -0.1)) + list(np.arange(0.3, 0.1, -0.05)) + list(np.arange(0.1, 0.01, -0.01))
    #spawn_multipliers = [1, 2, 3]

    # config_files = [f for f in os.listdir('configs') if f.endswith('.xlsx')]
    config_files = ['flow_config_1.xlsx'] # Use a single config for testing
    colors = plt.cm.jet(np.linspace(0, 1, len(config_files))) # Generate distinct colors

    try:
        for i, config_file in enumerate(config_files):
            config_file_path = os.path.join('configs', config_file)
            config_file_path = "flow_config.xlsx"
            print(f"Processing config file: {config_file_path}")

            pool_args = [(multiplier, config_file_path, n_rows, m_cols, use_gui) for multiplier in spawn_multipliers]

            # Use all available CPU cores
            with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
                results = pool.map(run_simulation_for_multiplier, pool_args)

            # Filter out cases where simulation might have failed or resulted in no cars
            valid_results_with_multipliers = []
            for res_idx, res in enumerate(results):
                if res and (res[0] > 0 or res[1] > 0):
                    valid_results_with_multipliers.append((spawn_multipliers[res_idx], res))

            if not valid_results_with_multipliers:
                continue

            # Unpack for plotting
            valid_multipliers = [item[0] for item in valid_results_with_multipliers]
            mean_densities = [item[1][0] for item in valid_results_with_multipliers]
            exit_flow_rates = [item[1][1] for item in valid_results_with_multipliers]

            # Plotting
            ax.plot(mean_densities, exit_flow_rates, marker='o', linestyle='-', color=colors[i], label=f'Config: {config_file}')
            for j, multiplier in enumerate(valid_multipliers):
                ax.annotate(f'{multiplier:.2f}', (mean_densities[j], exit_flow_rates[j]), xytext=(5, 5), textcoords='offset points')

        print("Experiment finished.")
        plt.legend()
        plt.savefig("flow_vs_density_plot_with_multipliers.png")
        plt.show()
    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt, terminating processes.")
        sys.exit(0)

if __name__ == '__main__':
    # This is important for multiprocessing to work correctly on all platforms
    multiprocessing.freeze_support()
    main()
