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

simulator = Simulator(win=None, use_gui=use_gui)

def run_simulation_for_multiplier(multiplier, config_file_path):
    print(f"Testing with multiplier: {multiplier:.2f} and config: {config_file_path}")
    
    intersections, roads, road_extremity_spawners = create_grid_setup(n_rows, m_cols, car_flow_rate=120)
    simulator.initialize(intersections, roads, road_extremity_spawners, config_file=config_file_path, spawn_intervall_multiplier=multiplier)

    max_ticks = 20000
    stability_ticks = 1000
    car_counts = []
    last_car_counts = []

    traffic_jam = False
    
    for tick in range(max_ticks):
        if tick % 1000 == 0:
            print(f"  ... tick {tick}/{max_ticks}")
        simulator.update(dt=15, events=[])
        
        if tick % 500 == 0:
            print(f"  ... tick {tick}/{max_ticks}")
            current_car_count = simulator.get_car_density()
            car_counts.append(current_car_count)
            last_car_counts.append(current_car_count)
            if len(last_car_counts) > stability_ticks:
                last_car_counts.pop(0)
                # Check if the car count has stabilized by checking if the standard deviation
                # of recent car counts is below a threshold.
                if np.std(last_car_counts) < 1.5:
                    print(f"System stable after {tick} ticks with car count std dev = {np.std(last_car_counts):.2f}")
                    traffic_jam = True
                    break
    
    if traffic_jam:
        print(f"Traffic jam detected.")
        return simulator.get_car_density(), 0

    else:
        print(f"No traffic jam detected.")
    exit_flow_rate = simulator.get_exit_flow_rate()
    mean_density = np.mean(car_counts)
    
    print(f"  -> Flow Rate: {exit_flow_rate:.4f}, Mean Density: {mean_density:.4f}")
    return mean_density, exit_flow_rate

# --- Main Experiment Loop ---
spawn_multipliers = np.arange(1.5, 0, -0.15)

config_files = [f for f in os.listdir('configs') if f.endswith('.xlsx')]
colors = plt.cm.jet(np.linspace(0, 1, len(config_files))) # Generate distinct colors

for i, config_file in enumerate(config_files):
    config_file_path = os.path.join('configs', config_file)
    exit_flow_rates = []
    mean_densities = []
    
    for multiplier in spawn_multipliers:
        exit_flow, mean_density = run_simulation_for_multiplier(multiplier, config_file_path)
        
        if exit_flow > 0 or mean_density > 0:
            exit_flow_rates.append(exit_flow)
            mean_densities.append(mean_density)

            ax.plot(exit_flow_rates, mean_densities, marker='o', linestyle='-', color=colors[i], label=f'Config: {config_file}')
            fig.canvas.draw()
            fig.canvas.flush_events()

print("Experiment finished.")
plt.ioff()
plt.legend() # Add legend to distinguish lines
plt.savefig("flow_vs_density_plot_all_configs.png") # Save with a new name
plt.show()
