from datetime import datetime

import pygame
import numpy as np
from Simulator import Simulator
from Constants import *
from Road import Road, RoadExtremity
from Car import Car
from Intersections import *
import sys
import matplotlib.pyplot as plt
from multiprocessing import Pool



print("start")
# Pygame setup
pygame.init()


win = pygame.display.set_mode((WIDTH, HEIGHT)) if RENDER else None
pygame.display.set_caption("Roundabout Simulator")

clock = pygame.time.Clock()

# Font for FPS display
font = pygame.font.Font(None, 30)



def create_ring_road_setup(n):
    fixed_road_length = 50

    directions = [Vec2(1, 0).rotate(i * 360 / n) for i in range(n)]
    ring_road = ClassicRoundabout((0, 0), ROUNDABOUT_RADIUS, directions)

    # Lists to hold the road extremities and roads
    road_extremity_spawners = []
    road_extremity_exits = []
    roads = []

    # Create road extremities and roads
    for i in range(n):

        inner_extremity = ring_road.exits[i]

        # Calculate the position of the road extremity

        if i % 2 == 0:
            # rotate vector 90 degrees to get the direction of the road extremity
            dir = directions[i].rotate(90)
            outer_extremity_pos = inner_extremity.pos + fixed_road_length * dir

            # Create the road extremity
            outer_extremity = RoadExtremity(outer_extremity_pos,
                                            spawn_cars=True)
            road_extremity_spawners.append(outer_extremity)
        else:
            dir = directions[i].rotate(-90)
            outer_extremity_pos = inner_extremity.pos + fixed_road_length * dir

            outer_extremity = RoadExtremity(outer_extremity_pos,
                                            spawn_cars=False)
            road_extremity_exits.append(outer_extremity)

        # Create the road
        road = Road(outer_extremity, inner_extremity)
        roads.append(road)
    intersections = [ring_road]

    return intersections, roads, road_extremity_spawners, road_extremity_exits

"""
intersections, roads, road_extremity_spawners, road_extremity_exits = create_ring_road_setup(
        8)
    simulator.initialize(intersections,
                        roads,
                        road_extremity_spawners,
                        car_spawn_interval=car_spawn_interval,
                        road_extremity_exits=road_extremity_exits)
"""






def start_simulation_with_parameters(alpha, beta, gamma, n, pred_ok, vp_max, degraded_mode=False, optimized_car_rate=1):
    print(f"-----------Starting simulation with parameters: alpha={alpha}, beta={beta}, gamma={gamma}, n={n}, pred_ok={pred_ok}, vp_max={vp_max:.2f}----------------")



    def on_car_spawned(car):
        if degraded_mode:
            if RING_ROAD:

                optimize = random.random() < optimized_car_rate
                if optimize:
                    car.alpha = ALPHA_FLUID
                    car.beta = BETA_FLUID
                    car.gamma = GAMMA_FLUID
                else:
                    car.alpha = ALPHA_CONGESTED
                    car.beta = BETA_CONGESTED
                    car.gamma = GAMMA_CONGESTED
                
        else:
            car.alpha = alpha
            car.beta = beta
            car.gamma = gamma

        if not RING_ROAD:
            car.speed = car.max_speed / 2  # Start at half of max speed to avoid initial congestion

    simulator = Simulator(win, use_gui=RENDER, on_car_spawned=on_car_spawned)

    if RING_ROAD:
        intersections, roads, road_extremity_spawners, road_extremity_exits = create_ring_road_setup(4)
    else:
        intersections = [
                            ClassicRoundabout((0, 0), ROUNDABOUT_RADIUS, [])
                        ]

        roads = []

        road_extremity_spawners = []

        road_extremity_exits = []



    simulator.initialize(intersections,
                            roads,
                            road_extremity_spawners,
                            road_extremity_exits=road_extremity_exits,
                            car_spawn_interval=CAR_SPAWN_INTERVAL,)  

    if not RING_ROAD:
        intersections[0].spawn_evenly_spaced_cars(n)

    optimized_car = 0

    if degraded_mode and not RING_ROAD:
        num_cars = len(simulator.cars)
        target_optimized = int(num_cars * optimized_car_rate)

        # Create a set of indices to be optimized
        if target_optimized > 0:
            # This spreads target_optimized indices across the range of num_cars
            optimized_indices = {int(k * num_cars / target_optimized) for k in range(target_optimized)}
        else:
            optimized_indices = set()

        for i, car in enumerate(simulator.cars):
            if i in optimized_indices:
                car.alpha, car.beta, car.gamma = ALPHA_FLUID, BETA_FLUID, GAMMA_FLUID
                optimized_car += 1
            else:
                car.alpha, car.beta, car.gamma = ALPHA_CONGESTED, BETA_CONGESTED, GAMMA_CONGESTED

        print(f"Degraded mode with {optimized_car} optimized cars out of {NUMBER_OF_CARS} total cars ({optimized_car_rate*100:.1f}%)")


    time_multiplier = 1

    tick = 0

    total_time = 0

    max_acceleration = 0
    min_acceleration = 0

    congested = False

    PAUSE = False

    while True:

        events = pygame.event.get()

        for event in events:
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    PAUSE = not PAUSE
                    print("PAUSE" if PAUSE else "RESUME")

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    time_multiplier = min(2.0, time_multiplier * 2)
                elif event.key == pygame.K_DOWN:
                    time_multiplier = max(0.01, time_multiplier / 2)


        if PAUSE:
            continue

        tick += 1
        dt = DT * time_multiplier 
        total_time += dt

        #if 5<= total_time <= 7:
        #    simulator.cars[0].speed = 0

        if simulator:
            simulator.update(dt, events)

        for car in simulator.cars:
            if car.acceleration > max_acceleration:
                max_acceleration = car.acceleration
            if car.acceleration < min_acceleration:
                min_acceleration = car.acceleration
        
        congested_cars = 0
        for car in simulator.cars:
            """if car.next_car:
                if (car.pos - car.next_car.pos).length() < 5:
                    print(f"Car at {car.pos} is too close to the next car at {car.next_car.pos} with speed {car.speed:.2f}")
                    print(f"Max acceleration: {max_acceleration:.2f}, Min acceleration: {min_acceleration:.2f}")
                    return False, simulator.energy_consumption/total_time, max_acceleration, min_acceleration"""

            if car.speed < 2 and car.status == "INTERSECTION":
                #congested = True
                congested_cars += 1

        if len(simulator.cars) > 0:
            if congested_cars/len(simulator.cars) > 0.1:        # if more than 10% of the cars are moving very slowly (<2m/s) in the circular road, we consider the trafic congested
                #print("congested")
                congested = True

            """if car.speed < 0:
                print(f"Wrong way : Max acceleration: {max_acceleration:.2f}, Min acceleration: {min_acceleration:.2f}")
                return False, simulator.energy_consumption, max_acceleration, min_acceleration"""

        if total_time >= MAX_SIMULATION_TIME:
            print(congested)
            print(f"Max acceleration: {max_acceleration:.2f}, Min acceleration: {min_acceleration:.2f}")
            return not congested, simulator.energy_consumption, max_acceleration, min_acceleration
        
        if tick % 8000 == 0:
            print(f"time: {total_time}")



        # Display FPS
        if RENDER:
            clock.tick()
            fps = clock.get_fps()
            fps_text = font.render(f"FPS: {int(fps)}", True,

                                (255, 255, 255))  # White color
            win.blit(fps_text, (10, 10))  # Position at top-left

            pred_text = font.render(f"Prediction: {'OK' if pred_ok else 'NOT OK'}, Max VP: {vp_max}", True, (255, 255, 255))
            win.blit(pred_text, (10, 40))

            pygame.display.update()

def predict(alpha, beta, gamma, n):

    A = np.zeros((n, n))

    for i in range(n - 1):
        A[i, i] = -1
        A[i, i + 1] = 1

    A[-1, 0] = 1
    A[-1, -1] = -1

    B = np.zeros((2*n, 2*n))
    B[:n,n:] = np.eye(n)
    B[n:,:n] = alpha*A
    B[n:,n:] = -beta*np.eye(n)+gamma*A
    valeurs_propres, vecteurs_propres = np.linalg.eig(B)

    print("Valeurs propres :", valeurs_propres)

    # On cherche à savoir si toutes valeurs_propres ont une partie réelle négative ou nulle (stabilité)
    if np.any(valeurs_propres.real - EPSILON > 0):     # On soustrait un petit epsilon pour éviter les problèmes de précision numérique (valeurs propres nulles pouvant être légèrement positives à cause de la précision)
        return False, valeurs_propres
    return True, valeurs_propres


import numpy as np
import matplotlib.pyplot as plt

def plot_stability_map(alpha, n, resolution=20):
    gammas = np.linspace(GAMMA_INTERVAL[0], GAMMA_INTERVAL[1], resolution)
    betas  = np.linspace(BETA_INTERVAL[0], BETA_INTERVAL[1], resolution)
    
    energy_grid = np.zeros((resolution, resolution))
    accelerations_max_grid = np.zeros((resolution, resolution))
    accelerations_min_grid = np.zeros((resolution, resolution))



    # 1. Create one single large figure
    plt.figure(figsize=(18, 10)) 

    inputs_parallels = [(alpha, beta, gamma, n, False, 0, False, 0.25) for gamma in gammas for beta in betas]

    if RENDER:
        results = []
        for g_idx, gamma in enumerate(gammas):
            for b_idx, beta in enumerate(betas):
                pred_ok, vp = predict(alpha, beta, gamma, n)
                sim_ok, energy_consumption, max_deceleration, min_deceleration = start_simulation_with_parameters(alpha, beta, gamma, n, pred_ok, np.max(vp)- EPSILON )
                results.append((sim_ok, energy_consumption, max_deceleration, min_deceleration))
    else:
        with Pool(2) as pool:
            results = pool.starmap(start_simulation_with_parameters, inputs_parallels)

    # --- Data Processing and Plot 1 (Stability Scatter) ---
    plt.subplot(2, 3, 1) # Position 1
    i = 0
    for g_idx, gamma in enumerate(gammas):
        for b_idx, beta in enumerate(betas):
            sim_ok, energy_consumption, max_deceleration, min_deceleration = results[i]
            pred_ok, vp = predict(alpha, beta, gamma, n)
            
            energy_grid[g_idx, b_idx] = energy_consumption
            accelerations_max_grid[g_idx, b_idx] = max_deceleration
            accelerations_min_grid[g_idx, b_idx] = min_deceleration

            color  = 'blue' if sim_ok else 'red'
            marker = 'o' if pred_ok else 'x'
            plt.scatter(beta, gamma, c=color, marker=marker, s=40)
            i += 1
            

    current_date =  datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    with open(f"Data/energy_grid_{current_date}.txt", "w") as f:
        f.write(f"Energy grid for alpha={alpha}, gammas={GAMMA_INTERVAL}, betas={BETA_INTERVAL}\n, n={n}\n, resolution={resolution}\n")
        f.write(str(energy_grid))



    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title(f'Stability Map (alpha={alpha})')

    # --- Plot 2: Energy Heatmap ---
    plt.subplot(2, 3, 2) # Position 2
    im1 = plt.imshow(np.log10(energy_grid), extent=[betas[0], betas[-1], gammas[0], gammas[-1]], 
                    origin='lower', aspect='auto', cmap='viridis')
    plt.colorbar(im1, label='Log10 Energy')
    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title('log(Energy Consumption) in Joule')

    # --- Plot 3: Energy vs Beta Line Plot ---
    plt.subplot(2, 3, 3) # Position 3
    im1 = plt.imshow(energy_grid, extent=[betas[0], betas[-1], gammas[0], gammas[-1]], 
                    origin='lower', aspect='auto', cmap='viridis')
    plt.colorbar(im1, label='Energy')
    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title('Energy Consumption in Joule')

    # --- Plot 4: Max Acceleration Heatmap ---
    plt.subplot(2, 3, 4) # Position 4
    im2 = plt.imshow(accelerations_max_grid, extent=[betas[0], betas[-1], gammas[0], gammas[-1]], 
                    origin='lower', aspect='auto', cmap='magma')
    plt.colorbar(im2, label='Max Accel')
    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title('Max Acceleration')

    # --- Plot 5: Max Deceleration Heatmap ---
    plt.subplot(2, 3, 5) # Position 5
    im3 = plt.imshow(-accelerations_min_grid, extent=[betas[0], betas[-1], gammas[0], gammas[-1]], 
                    origin='lower', aspect='auto', cmap='plasma')
    plt.colorbar(im3, label='Max Decel')
    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title('Max Deceleration')

    # Final layout adjustments and single show call
    plt.tight_layout()
    plt.show()


def plot_car_optimization_percentage_map(n, resolution=10):
    optimized_car_rates = np.linspace(0, 1, resolution)
    results = []
    for rate in optimized_car_rates:
        sim_ok, energy_consumption, max_deceleration, min_deceleration = start_simulation_with_parameters(ALPHA_INTERVAL[0], BETA_INTERVAL[0], GAMMA_INTERVAL[0], n, False, 0, degraded_mode=True, optimized_car_rate=rate)
        results.append((sim_ok, energy_consumption, max_deceleration, min_deceleration))
        print(f"Optimized car rate: {rate:.2f}, Simulation OK: {sim_ok}, Energy: {energy_consumption:.2f}, Max Decel: {max_deceleration:.2f}, Min Decel: {min_deceleration:.2f}")

    sim_status = [int(r[0]) for r in results]
    energy_vals = [r[1] for r in results]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.subplots_adjust(hspace=0.3) # Give them some breathing room

    # --- Plot 1: Simulation Success vs Rate ---
    ax1.step(optimized_car_rates, sim_status, where='post', color='teal', linewidth=2)
    ax1.fill_between(optimized_car_rates, sim_status, step="post", alpha=0.2, color='teal')
    ax1.set_yticks([0, 1])
    ax1.set_yticklabels(['FAIL', 'OK'])
    ax1.set_ylabel('Simulation Status', fontweight='bold')
    ax1.set_title('Simulation Success vs. Optimized Car Rate', fontsize=14)
    ax1.grid(axis='x', linestyle='--', alpha=0.7)

    # --- Plot 2: Energy Consumption vs Rate ---
    ax2.plot(optimized_car_rates, energy_vals, color='firebrick', marker='o', markersize=4, linestyle='-')
    ax2.set_ylabel('Energy Consumption', fontweight='bold')
    ax2.set_xlabel('Optimized Car Rate', fontweight='bold')
    ax2.set_title('Energy Consumption vs. Optimized Car Rate', fontsize=14)
    ax2.grid(True, linestyle='--', alpha=0.7)

    # Optional: Highlight the "Fail" zones on the energy plot for context
    for i in range(len(sim_status)):
        if sim_status[i] == 0:
            ax2.axvspan(optimized_car_rates[i], optimized_car_rates[min(i+1, len(sim_status)-1)], 
                        color='gray', alpha=0.1)

    plt.show()



# Exemple d'appel

if __name__ == '__main__':
    if DEGRADED_MODE:
        plot_car_optimization_percentage_map(n=NUMBER_OF_CARS, resolution=RESOLUTION)
    else:
        plot_stability_map(alpha=ALPHA_INTERVAL[0], n=NUMBER_OF_CARS, resolution=RESOLUTION)

