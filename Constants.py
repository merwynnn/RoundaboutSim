# Constants
import pygame


WIDTH, HEIGHT = 1400, 1000
BACKGROUND_COLOR = (0, 150, 0)
ROAD_COLOR = (113, 112, 113)
LANE_WIDTH = 3.125
STRIPE_WIDTH = 0.375

BASE_CAR_LENGTH = 35 # Base size for car images at zoom level 1.0
BASE_CAR_WIDTH = 20  # Base width for car images at zoom level 1.0
REAL_CAR_LENGTH = 4.36 # in meters
REAL_CAR_WIDTH = 2.5  # in meters

# pixel per meter at zoom level 1.0
PIXEL_PER_METER = BASE_CAR_LENGTH / REAL_CAR_LENGTH

DT = 0.1  # seconds per simulation step

DEBUG = False

REACTION_TIME = 400

pygame.font.init()
font = pygame.font.Font(None, 24)
font_medium = pygame.font.Font(None, 32)

def to_pixel(value):
    return value * PIXEL_PER_METER

def to_world(value):
    return value / PIXEL_PER_METER