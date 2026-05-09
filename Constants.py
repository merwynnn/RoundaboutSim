# Constants
import pygame


WIDTH, HEIGHT = 1000, 800
BACKGROUND_COLOR = (16, 117, 38)   #(13, 12, 66)
ROAD_COLOR = (113, 112, 113)
LANE_WIDTH = 3.125
STRIPE_WIDTH = 0.375

BASE_CAR_LENGTH = 35 # Base size for car images at zoom level 1.0 in pixel
BASE_CAR_WIDTH = 20  # Base width for car images at zoom level 1.0 in pixel
REAL_CAR_LENGTH = 4.36 # in meters
REAL_CAR_WIDTH = 2.5  # in meters
CAR_WEIGHT = 1500  # in kg

NUMBER_OF_CARS = 36 #40

ROUNDABOUT_RADIUS = 90 #320  # in meters, 90

MAX_SIMULATION_TIME = 600  # in seconds

MAX_SPEED = 70 / 3.6  # m/s, convert from km/h

TARGET_DISTANCE = 19


ALPHA_INTERVAL = [0.02, 0.02]

BETA_INTERVAL = [0.01, 0.5]

GAMMA_INTERVAL = [0.01, 0.5]    

"""
# Configuration "confort" non stable

ALPHA_INTERVAL = [0.02, 0.02]
BETA_INTERVAL = [0.1, 0.1]
GAMMA_INTERVAL = [0.1, 0.1]

# Configuration "confort" stable
ALPHA_INTERVAL = [0.02, 0.02]
BETA_INTERVAL = [0.12, 0.12]
GAMMA_INTERVAL = [0.2, 0.12]
"""


RENDER = False  # Whether to render the simulation or not. Set to False for faster testing without visualization.   

RING_ROAD = False   # Whether to use a ring road setup or a circular road setup.

RESOLUTION = 25


##### Degraded Mode Parameters #####

DEGRADED_MODE = True  

ALPHA_FLUID = 0.02
BETA_FLUID = 0.1
GAMMA_FLUID = 0.1

ALPHA_CONGESTED = 0.02      
BETA_CONGESTED = 0.01       # 0.12
GAMMA_CONGESTED = 0.1733    #0.2


ROUNDABOUT_RESOLUTION = 80  # Number of points to represent the roundabout circle


# pixel per meter at zoom level 1.0
PIXEL_PER_METER = BASE_CAR_LENGTH / REAL_CAR_LENGTH

DT = 0.01  # seconds per simulation step

EPSILON = 1e-4

CAR_SPAWN_INTERVAL = 3 # seconds between car spawns


DEBUG = False

pygame.font.init()
font = pygame.font.Font(None, 24)
font_medium = pygame.font.Font(None, 32)

def to_pixel(value):
    return value * PIXEL_PER_METER

def to_world(value):
    return value / PIXEL_PER_METER


# --- Paramètres globaux ---
TAU = 0.3              # s, constante EMA
DT = 0.1                 # s, pas de simulation
V_FREE = 20.0 / 3.6      # m/s, trafic libre
V_CONG = 18.0 / 3.6      # m/s, trafic congestionné
DV_UP = 10.0 / 3.6       # m/s, seuil de variation pour front amont du trafic (ralentissement)
DV_DOWN = 10.0 / 3.6     # m/s, seuil de variation pour front aval du trafic (accélération)
T_MIN_STATE = 0.1        # s, durée minimale dans un état pour éviter "chattering"

# états possibles
FREE = "FREE"
UP = "UPSTREAM_FRONT"    # approche d'un bouchon (front amont)
JAM = "CONGESTED"
BOTTLENECK = "BOTTLENECK"
DOWN = "DOWNSTREAM_FRONT"

PRIORITY = [DOWN, BOTTLENECK, JAM, UP, FREE]
