# Constants
import pygame

"""Graphical constants"""

DISPLAY_SCALE = 1.3  # Scale factor for rendering

WIDTH, HEIGHT = 1920, 1080
BACKGROUND_COLOR =  (174, 237, 242) #(16, 117, 38)   #(13, 12, 66)
ROAD_COLOR = (190, 190, 190)
LANE_WIDTH = 4.125 * DISPLAY_SCALE
STRIPE_WIDTH = 0.375 * DISPLAY_SCALE

BASE_CAR_LENGTH = 35 # Base size for car images at zoom level 1.0 in pixel
BASE_CAR_WIDTH = 20 
REAL_CAR_LENGTH = 4.36 * DISPLAY_SCALE # in meters
REAL_CAR_WIDTH = 2.5 * DISPLAY_SCALE  # in meters
CAR_WEIGHT = 1500  # in kg

RENDER = True  # Whether to render the simulation or not.


pygame.font.init()
font = pygame.font.Font(None, 24)
font_medium = pygame.font.Font(None, 32)

def to_pixel(value):
    return value * PIXEL_PER_METER

def to_world(value):
    return value / PIXEL_PER_METER

DEBUG = False

RING_ROAD_ENTER_MIN_DISTANCE = 20

"""Simulation Constants """
NUMBER_OF_CARS = 30

ROUNDABOUT_RADIUS = 100   # in meters

MIN_SIMULATION_TIME = 600  # in seconds

MAX_SPEED = 70 / 3.6  # m/s

TARGET_DISTANCE = 20        #Deltax_obj


ALPHA_INTERVAL = [0.02, 0.02]

BETA_INTERVAL = [0.01, 0.5]

GAMMA_INTERVAL = [0.01, 0.5] 


RESOLUTION = 10


##### Degraded Mode Parameters #####

DEGRADED_MODE = False  
"""
ALPHA_FLUID = 0.02
BETA_FLUID = 0.1188
GAMMA_FLUID = 0.1188
"""
ALPHA_FLUID = 0.02
BETA_FLUID = 0.174
GAMMA_FLUID = 0.174


ALPHA_CONGESTED = 0.02      
BETA_CONGESTED = 0.0644       # 0.12
GAMMA_CONGESTED = 0.0644    #0.2

# Number of points to represent the roundabout circle
ROUNDABOUT_RESOLUTION = 80  

# Whether to use a ring road setup or a circular road setup.
RING_ROAD = False   

CAR_SPAWN_INTERVAL = 20 # seconds between car spawns

# pixel per meter at zoom level 1.0
PIXEL_PER_METER = BASE_CAR_LENGTH / REAL_CAR_LENGTH

DT = 0.1  # seconds per simulation step

EPSILON = 1e-8


