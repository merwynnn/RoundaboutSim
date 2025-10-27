# Constants
import pygame


WIDTH, HEIGHT = 1000, 800
BACKGROUND_COLOR = (0, 150, 0)
ROAD_COLOR = (113, 112, 113)
LANE_WIDTH = 3.125
STRIPE_WIDTH = 0.375

BASE_CAR_LENGTH = 35 # Base size for car images at zoom level 1.0
BASE_CAR_WIDTH = 20  # Base width for car images at zoom level 1.0
REAL_CAR_LENGTH = 4.36 # in meters
REAL_CAR_WIDTH = 2.5  # in meters

ROUNDABOUT_RADIUS = 17.5  # in meters

# pixel per meter at zoom level 1.0
PIXEL_PER_METER = BASE_CAR_LENGTH / REAL_CAR_LENGTH

DT = 0.1  # seconds per simulation step

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
