import random
import time

import pygame
from pygame import Vector2 as Vec2
from Constants import *
from Road import RoadExtremity
import math

class RingRoad:
    def __init__(self, pos, radius, exits_dir):
        self.pos = Vec2(pos)
        from Simulator import Simulator
        self.simulator = Simulator.get_instance()
        self.exits = []

        self.detection_angle_threshold = 50
        self.detection_rotation_angle = 0


        self.radius = radius
        self.center = Vec2(pos)
        self.nb_lanes = 1
        
        self.exits = []
        for exit_dir in exits_dir:
            self.exits.append(RoadExtremity((self.center.x + self.radius * exit_dir.x, self.center.y + self.radius * exit_dir.y), self))

        self.targets = self.get_evenly_spaced_points(ROUNDABOUT_RESOLUTION)[::-1]

    def draw(self, win):
        transformed_center = self.simulator.camera.apply(self.center)

        scaled_radius = self.simulator.camera.get_scaled_value(self.radius)
        # Ensure radius is at least 1 after scaling to be drawable
        scaled_radius = max(1, int(scaled_radius))

        scaled_lane_width = self.simulator.camera.get_scaled_value(LANE_WIDTH)
        scaled_stripe_width = max(1, int(self.simulator.camera.get_scaled_value(STRIPE_WIDTH)))

        pygame.draw.circle(win, ROAD_COLOR, transformed_center, scaled_radius)
        inner_radius = scaled_radius - self.nb_lanes * scaled_lane_width
        if inner_radius < 0 : inner_radius = 0 # Prevent negative radius
        pygame.draw.circle(win, BACKGROUND_COLOR, transformed_center, inner_radius)

        pygame.draw.circle(win, (255, 255, 255), transformed_center, scaled_radius, scaled_stripe_width)
        if inner_radius > 0: # Only draw inner stripe if visible
             pygame.draw.circle(win, (255, 255, 255), transformed_center, inner_radius, scaled_stripe_width)

        if self.simulator.debug:
            debug_radius = max(1, int(self.simulator.camera.get_scaled_value(0.5)))
            for i, target in enumerate(self.targets):
                transformed_target = self.simulator.camera.apply(target)
                pygame.draw.circle(win, (10, 0, 0), transformed_target, debug_radius)
            


    def get_evenly_spaced_points(self, n_points):
        """
        Returns a list of Vec2 points evenly spaced along the circumference of the roundabout.
        """
        points = []
        radius = self.radius - LANE_WIDTH/2
        
        for i in range(n_points):
            angle = (2 * math.pi / n_points) * i
            x = self.center.x + radius * math.cos(angle)
            y = self.center.y + radius * math.sin(angle)
            points.append(Vec2(x, y))
        return points
    
    def get_closest_target(self, pos, dir=None):
        """ Returns the index of the closest target to the given position, optionally considering only targets in the direction of 'dir' """
        min_dist = math.inf
        closest_target_id = None
        for id, target in enumerate(self.targets):
            dist = (target - pos).length()
            if dist < min_dist:
                if dir is not None:
                    if (target - pos).length() != 0:
                        target_dir = (target - pos).normalize()
                        if target_dir.dot(dir) > 0:
                            min_dist = dist
                            closest_target_id = id
                else:
                    min_dist = dist
                    closest_target_id = id

        return closest_target_id
        
    def get_index(self, i):
        return i%len(self.targets)

    def get_next_target_position(self, start_extremity, exit_extremity, current_target_index, car=None):
        """ returns the position of the next target for a car, based on the current_target_index (which is the index of the last target reached by the car) """
        start_target_index = self.get_index(selfget_closest_target(start_extremity.get_other_extremity().get_end_car_pos_dir()[0])+1)
        exit_target_index = self.get_index(selfget_closest_target(exit_extremity.get_start_car_pos_dir()[0])-1)
        target_index = self.get_index(start_target_index+current_target_index)
        return self.targets[target_index], exit_target_index == target_index

    
    def spawn_evenly_spaced_cars(self, n_cars):
        """ Spawns n_cars evenly spaced on the roundabout, with random perturbations """
        positions = self.get_evenly_spaced_points(n_cars)
        delta_angle = 2 * math.pi / n_cars
        for i, _ in enumerate(positions):
            angle = (2 * math.pi / n_cars) * i 
            if not RING_ROAD:
                angle += random.uniform(0, delta_angle*0.8)  # Add some randomness to the angle to avoid perfect spacing (creates a tiny perturbation)
            x = self.center.x + self.radius * math.cos(angle)
            y = self.center.y + self.radius * math.sin(angle)

            pos = Vec2(x, y)
            car_dir = Vec2(math.sin(angle), -math.cos(angle))  # Tangential direction
            index = self.get_index(selfget_closest_target(pos, car_dir))
            self.simulator.spawn_car_at_position(pos, car_dir, intersection=self, target_position=self.targets[index], target_index = index)