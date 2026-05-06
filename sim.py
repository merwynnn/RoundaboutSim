import pygame
import numpy as np
from Simulator import Simulator
from Constants import *
from Road import Road, RoadExtremity
from Car import Car
from Intersections import *
import sys
import matplotlib.pyplot as plt

print("start")
# Pygame setup
pygame.init()

render = True

win = pygame.display.set_mode((WIDTH, HEIGHT)) if render else None
pygame.display.set_caption("Roundabout Simulator")

clock = pygame.time.Clock()

# Font for FPS display
font = pygame.font.Font(None, 30)

simulator = Simulator(win, use_gui=render)

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

beta_interval = [0.1, 0.5]

gamma_interval = [0.3, 1.0]




def start_simulation_with_parameters(alpha, beta, gamma, n, pred_ok):

    intersections = [
                        ClassicRoundabout((0, 0), ROUNDABOUT_RADIUS * 5, [])
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

    end_simulation = False

    time_multiplier = 1

    tick = 0

    total_time = 0

    PAUSE = False

    while not end_simulation:
        

        events = pygame.event.get()

        for event in events:
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    PAUSE = not PAUSE



        if PAUSE:
            continue

        tick += 1
        dt = DT * time_multiplier 
        total_time += dt

        if 5<= total_time <= 7:
            simulator.cars[0].speed = 0

        if simulator:
            simulator.update(dt, events)
        
        if total_time >= 15:
            for car in simulator.cars:
                if car.speed < 2.5:
                    end_simulation = True
                    return False

        if total_time >= 500:
            end_simulation = True

            return True
        
        if tick % 100 == 0:
            print(f"time: {total_time}")



        # Display FPS
        if render:
            clock.tick()
            fps = clock.get_fps()
            fps_text = font.render(f"FPS: {int(fps)}", True,

                                (255, 255, 255))  # White color
            win.blit(fps_text, (10, 10))  # Position at top-left

            pred_text = font.render(f"Pred: {'OK' if pred_ok else 'Not OK'}", True, (255, 255, 255))
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
    print(B)
    B[n:,:n] = alpha*A
    print(B)
    B[n:,n:] = -beta*np.eye(n)+gamma*A
    print(B)
    valeurs_propres, vecteurs_propres = np.linalg.eig(B)

    print("Valeurs propres :", valeurs_propres)

    if np.any(valeurs_propres.real > 0):
        return False
    return True


def plot_stability_map(alpha, n, resolution=20):
    gammas = np.linspace(gamma_interval[0], gamma_interval[1], resolution)
    betas  = np.linspace(beta_interval[0], beta_interval[1], resolution)
    i = 0
    for gamma in gammas:
        for beta in betas:
            print(f"Sim {i}/{resolution**2}, alpha={alpha}, beta={beta}, gamma={gamma}")
            
            pred_ok = predict(alpha, beta, gamma, n)
            sim_ok  = start_simulation_with_parameters(alpha, beta, gamma, n, pred_ok)
            

            color  = 'blue' if sim_ok  else 'red'
            marker = 'o'    if pred_ok else 'x'
            plt.scatter(beta, gamma, c=color, marker=marker, s=60)

            i+=1

    plt.xlabel('beta')
    plt.ylabel('gamma')
    plt.title(f'Carte de stabilité — alpha={alpha}, n={n}')
    plt.tight_layout()
    plt.show()

plot_stability_map(alpha=0.1, n=32, resolution=3)

