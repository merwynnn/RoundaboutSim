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

render = False

win = pygame.display.set_mode((WIDTH, HEIGHT)) if render else None
pygame.display.set_caption("Roundabout Simulator")

clock = pygame.time.Clock()

# Font for FPS display
font = pygame.font.Font(None, 30)


car_spawn_interval = 0.2



def create_ring_road_setup(n):
    fixed_road_length = 100
    radius = 300

    directions = [Vec2(1, 0).rotate(i * 360 / n) for i in range(n)]
    ring_road = ClassicRoundabout((0, 0), radius, directions)

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

alpha_interval = [0.01, 0.1]

beta_interval = [0.01, 0.1]

gamma_interval = [0.1, 1.0]




def start_simulation_with_parameters(alpha, beta, gamma, n, pred_ok, vp_max):
    simulator = Simulator(win, use_gui=render)


    intersections = [
                        ClassicRoundabout((0, 0), ROUNDABOUT_RADIUS, [])
                    ]

    roads = []

    road_extremity_spawners = []

    simulator.initialize(intersections,
                            roads,
                            road_extremity_spawners,
                            car_spawn_interval=car_spawn_interval)  

    intersections[0].spawn_evenly_spaced_cars(n)

    for car in simulator.cars:
        car.alpha = alpha
        car.beta = beta
        car.gamma = gamma

    time_multiplier = 0.1

    tick = 0

    total_time = 0

    max_acceleration = 0
    min_acceleration = 0

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
        
        if total_time >= 15:
            for car in simulator.cars:
                if car.next_car:
                    if (car.pos - car.next_car.pos).length() < 5:
                        print(f"Car at {car.pos} is too close to the next car at {car.next_car.pos} with speed {car.speed:.2f}")
                        print(f"Max acceleration: {max_acceleration:.2f}, Min acceleration: {min_acceleration:.2f}")
                        return False, simulator.energy_consumption/total_time, max_acceleration, min_acceleration
                if car.speed < 2.5:
                    print(f"Max acceleration: {max_acceleration:.2f}, Min acceleration: {min_acceleration:.2f}")
                    return False, simulator.energy_consumption/total_time, max_acceleration, min_acceleration

        if total_time >= MAX_SIMULATION_TIME:
            print(f"Max acceleration: {max_acceleration:.2f}, Min acceleration: {min_acceleration:.2f}")
            return True, simulator.energy_consumption/total_time, max_acceleration, min_acceleration
        
        if tick % 8000 == 0:
            print(f"time: {total_time}")



        # Display FPS
        if render:
            clock.tick()
            fps = clock.get_fps()
            fps_text = font.render(f"FPS: {int(fps)}", True,

                                (255, 255, 255))  # White color
            win.blit(fps_text, (10, 10))  # Position at top-left

            pred_text = font.render(f"Prediction: {'OK' if pred_ok else 'NOT OK'}, Max VP: {vp_max:.2f}", True, (255, 255, 255))
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

    if np.any(valeurs_propres.real - 1e-6 > 0):
        return False, valeurs_propres
    return True, valeurs_propres


import numpy as np
import matplotlib.pyplot as plt

def plot_stability_map(alpha, n, resolution=20):
    gammas = np.linspace(gamma_interval[0], gamma_interval[1], resolution)
    betas  = np.linspace(beta_interval[0], beta_interval[1], resolution)
    
    # Initialisation d'une grille pour stocker l'énergie
    energy_grid = np.zeros((resolution, resolution))

    valeurs_propres_max = []
    puissances = []

    accelerations_max_grid = np.zeros((resolution, resolution))
    accelerations_min_grid = np.zeros((resolution, resolution))
    
    # Premier Graphique : Stabilité (Scatter plot)
    plt.figure(figsize=(14, 6))
    
    plt.subplot(1, 2, 1) # 1 ligne, 2 colonnes, index 1
    inputs_parallels = [(alpha, beta, gamma, n, False, 0) for gamma in gammas for beta in betas]


    
    if render:
        results = []
        for g_idx, gamma in enumerate(gammas):
            for b_idx, beta in enumerate(betas):
                pred_ok, vp = predict(alpha, beta, gamma, n)
                sim_ok, energy_consumption, max_deceleration, min_deceleration = start_simulation_with_parameters(alpha, beta, gamma, n, pred_ok, np.max(vp))
                results.append((sim_ok, energy_consumption, max_deceleration, min_deceleration))

    else:
        with Pool(2) as pool:
            results = pool.starmap(start_simulation_with_parameters, inputs_parallels)

    plt.subplot(1, 2, 1)
    i = 0
    for g_idx, gamma in enumerate(gammas):
        for b_idx, beta in enumerate(betas):
            sim_ok, energy_consumption, max_deceleration, min_deceleration = results[i]
            pred_ok, vp = predict(alpha, beta, gamma, n)
            
            # Stockage de l'énergie pour le heatmap
            energy_grid[g_idx, b_idx] = energy_consumption
            valeurs_propres_max.append(np.max(vp))
            puissances.append(energy_consumption)

            accelerations_max_grid[g_idx, b_idx] = max_deceleration
            accelerations_min_grid[g_idx, b_idx] = min_deceleration


            # Affichage Stabilité
            color  = 'blue' if pred_ok  else 'red'
            marker = 'o'    if pred_ok else 'x'
            plt.scatter(beta, gamma, c=color, marker=marker, s=60)
            i += 1
        
    print(accelerations_max_grid, accelerations_min_grid)

    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title(f'Carte de stabilité — alpha={alpha}')

    # Deuxième Graphique : Heatmap de l'Énergie
    plt.subplot(1, 2, 2) # index 2
    # On utilise 'origin=lower' pour que l'axe Y corresponde aux gammas croissants vers le haut
    im = plt.imshow(np.log10(energy_grid), extent=[betas[0], betas[-1], gammas[0], gammas[-1]], 
                    origin='lower', aspect='auto', cmap='viridis')
    
    plt.colorbar(im, label='Energy Consumption')
    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title(f'Consommation Énergétique — alpha={alpha}')



    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(7, 6))
    plt.plot(betas, np.log10(energy_grid[0]), label='Energy Consumption')
    plt.xlabel('beta')
    plt.ylabel('Energy Consumption')
    plt.title(f'Energy Consumption vs beta for alpha={alpha} and gamma={gammas[0]}')
    plt.legend()
    plt.show()

    plt.figure(figsize=(14, 6))

    plt.subplot(1, 2, 1)
    im = plt.imshow(accelerations_max_grid, extent=[betas[0], betas[-1], gammas[0], gammas[-1]], 
                    origin='lower', aspect='auto', cmap='viridis')
    
    plt.colorbar(im, label='Max Acceleration')
    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title(f'Max Acceleration — alpha={alpha}')

    plt.subplot(1, 2, 2)
    im = plt.imshow(-accelerations_min_grid, extent=[betas[0], betas[-1], gammas[0], gammas[-1]], 
                    origin='lower', aspect='auto', cmap='viridis')
    plt.colorbar(im, label='Min Acceleration')
    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title(f'Max Deceleration — alpha={alpha}')
    plt.show()

    


# Exemple d'appel

if __name__ == '__main__':
    plot_stability_map(alpha=0.02, n=10, resolution=4)

