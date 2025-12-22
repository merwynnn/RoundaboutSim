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
import csv # Import csv module

# --- Simulation Setup ---
use_gui = False  # Set to False for batch processing

def create_grid_setup(n, m):
    RoadExtremity.next_id = 0
    intersections = []
    roads = []
    road_extremity_spawners = []

    fixed_road_length = 80
    roundabout_radius = 17.5
    # Spacing between the centers of adjacent roundabouts
    spacing_between_centers = fixed_road_length + 2 * roundabout_radius

    car_flow_rate = 120


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


def run_simulation_for_multiplier(args):
    multiplier, config_file_path, n_rows, m_cols, use_gui = args
    print(f"Testing with multiplier: {multiplier:.2f} and config: {config_file_path}")
    
    simulator = Simulator(win=None, use_gui=use_gui)
    intersections, roads, road_extremity_spawners = create_grid_setup(n_rows, m_cols,)
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
        simulator.update(dt=DT, events=[])
        
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
    average_car_lifetime = simulator.get_average_car_lifetime() if simulator.car_lifetimes else 0

    flow_rate = simulator.get_real_entry_flow_rate()*60*60
    
    print(f"  -> Mean Entry Flow Rate: {flow_rate:.4f}, Mean Exit Flow Rate: {mean_exit_flow_rate:.4f}, Flow Rate: {flow_rate:.4f}, Mean Density: {mean_density:.4f}, Average Car Lifetime: {average_car_lifetime:.4f}")
    return mean_density, mean_exit_flow_rate, average_car_lifetime

def main():
    # --- Experiment Parameters ---
    n_rows = 2
    m_cols = 2

    # --- Matplotlib Setup ---
    fig, ax1 = plt.subplots(figsize=(10, 6)) # Use ax1 for flow rate
    ax2 = ax1.twinx() # Create a second y-axis for car lifetime

    ax1.set_ylabel("Flow Rate (Cars Exiting per tick)", color='tab:blue')
    ax1.set_xlabel("Mean Density (cars)")
    ax1.set_title("Global Flow Rate and Average Car Lifetime vs. Mean Density")
    ax1.grid(True)
    ax1.set_xlim(0, 200) # Adjust as needed
    ax1.set_ylim(0, 400) # Adjust as needed
    ax1.tick_params(axis='y', labelcolor='tab:blue')

    ax2.set_ylabel("Average Car Lifetime (ticks)", color='tab:red')
    ax2.set_ylim(0, 10000) # Adjust as needed for car lifetime
    ax2.tick_params(axis='y', labelcolor='tab:red')

    # --- Main Experiment Loop ---
    spawn_multipliers = list(np.arange(5, 2, -1)) + list(np.arange(2, 1, -0.5)) + list(np.arange(1, 0.3, -0.1)) + list(np.arange(0.3, 0.1, -0.05)) + list(np.arange(0.1, 0.01, -0.01))
    #spawn_multipliers = [1, 2, 3]

    config_files = [f for f in os.listdir('configs') if f.endswith('.xlsx')]
    config_files = ['flow_config_1.xlsx'] # Use a single config for testing
    colors = plt.cm.jet(np.linspace(0, 1, len(config_files))) # Generate distinct colors

    all_simulation_results = [] # List to store all results

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
                if res and (res[0] > 0 or res[1] > 0 or res[2] > 0): # Check all returned values
                    valid_results_with_multipliers.append((spawn_multipliers[res_idx], res))
                    # Store results for saving to file
                    all_simulation_results.append({
                        'config_file': config_file,
                        'multiplier': spawn_multipliers[res_idx],
                        'mean_density': res[0],
                        'exit_flow_rate': res[1],
                        'average_car_lifetime': res[2] # Store average car lifetime
                    })

            if not valid_results_with_multipliers:
                continue

            # Unpack for plotting
            valid_multipliers = [item[0] for item in valid_results_with_multipliers]
            mean_densities = [item[1][0] for item in valid_results_with_multipliers]
            exit_flow_rates = [item[1][1] for item in valid_results_with_multipliers]
            average_car_lifetimes = [item[1][2] for item in valid_results_with_multipliers] # Unpack average car lifetime

            # Plotting on ax1 (Flow Rate)
            ax1.plot(mean_densities, exit_flow_rates, marker='o', linestyle='-', color='tab:blue', label=f'Flow Rate - Config: {config_file}')
            for j, multiplier in enumerate(valid_multipliers):
                ax1.annotate(f'{multiplier:.2f}', (mean_densities[j], exit_flow_rates[j]), xytext=(5, 5), textcoords='offset points', color='tab:blue')

            # Plotting on ax2 (Average Car Lifetime)
            ax2.plot(mean_densities, average_car_lifetimes, marker='x', linestyle='--', color='tab:red', label=f'Avg Lifetime - Config: {config_file}')
            for j, multiplier in enumerate(valid_multipliers):
                ax2.annotate(f'{multiplier:.2f}', (mean_densities[j], average_car_lifetimes[j]), xytext=(5, -15), textcoords='offset points', color='tab:red')


        print("Experiment finished.")
        fig.legend(loc="upper left", bbox_to_anchor=(0.1, 0.9)) # Adjust legend position
        plt.savefig("flow_lifetime_vs_density_plot.png") # New filename for the combined plot
        plt.show()

        # Save all simulation results to a CSV file
        results_file_path = "simulation_results.csv"
        with open(results_file_path, 'w', newline='') as csvfile:
            fieldnames = ['config_file', 'multiplier', 'mean_density', 'exit_flow_rate', 'average_car_lifetime'] # Add new field
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for row in all_simulation_results:
                writer.writerow(row)
        print(f"Simulation results saved to {results_file_path}")

    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt, terminating processes.")
        sys.exit(0)

if __name__ == '__main__':
    # This is important for multiprocessing to work correctly on all platforms
    multiprocessing.freeze_support()
    main()
