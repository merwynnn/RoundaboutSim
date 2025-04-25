import pygame
from pygame import Vector2 as Vec2
from Constants import *
from Road import RoadExtremity
import math
class Intersection:
    def __init__(self, pos):
        self.pos = Vec2(pos)
        from Simulator import Simulator
        self.simulator = Simulator.get_instance()
        

    def update(self):
        pass
    
    def can_car_enter(self, car):
        """
        Parent method to check if a car can enter the intersection. Should be overridden by subclasses.
        Returns True if the car can enter, False otherwise.
        """
        return True

    def draw(self, win):
        pass

    def get_next_target_position(self, start_extremity, exit_extremity, current_target_index, car=None):
        #return next_target_pos, is_last_pos
        pass
        
class ClassicRoundabout(Intersection):
    def __init__(self, pos, radius, exits_dir):
        super().__init__(pos)
        self.radius = radius
        self.center = Vec2(pos)
        self.nb_lanes = 1

        self.nb_target_to_check_before_enter = 2

        self.exits = []
        for exit_dir in exits_dir:
            self.exits.append(RoadExtremity((self.center.x + self.radius * exit_dir.x, self.center.y + self.radius * exit_dir.y), self))

        self.targets = self.get_evenly_spaced_points(14)[::-1]

        self.cars_between_targets = [[] for _ in self.targets]   # 0: between 0 and 1, 1: between 1 and 2, etc.

    def draw(self, win):
        pygame.draw.circle(win, ROAD_COLOR, self.center, self.radius)
        pygame.draw.circle(win, BACKGROUND_COLOR, self.center, self.radius-self.nb_lanes*LANE_WIDTH)
        pygame.draw.circle(win, (255, 255, 255), self.center, self.radius, STRIPE_WIDTH)
        pygame.draw.circle(win, (255, 255, 255), self.center, self.radius-self.nb_lanes*LANE_WIDTH, STRIPE_WIDTH)

        if self.simulator.debug:
            for i, target in enumerate(self.targets):
                pygame.draw.circle(win, (10, 0, 0), target, 4)
            


    def get_evenly_spaced_points(self, n_points):
        """
        Returns a list of Vec2 points evenly spaced along the circumference of the roundabout.
        """
        points = []
        radius = self.radius - LANE_WIDTH//2
        
        for i in range(n_points):
            angle = (2 * math.pi / n_points) * i
            # Use the center and radius to calculate the point position
            x = self.center.x + radius * math.cos(angle)
            y = self.center.y + radius * math.sin(angle)
            points.append(Vec2(x, y))
        return points
    
    def closest_target(self, pos):
        min_dist = math.inf
        closest_target_id = None
        for id, target in enumerate(self.targets):
            dist = (target - pos).length()
            if dist < min_dist:
                min_dist = dist
                closest_target_id = id
        return closest_target_id
        
    def get_index(self, i):
        return i%len(self.targets)

    def get_next_target_position(self, start_extremity, exit_extremity, current_target_index, car=None):
        """ returns the position of the current_target_index based on an extremity """
        start_target_index = self.get_index(self.closest_target(start_extremity.get_other_extremity().get_end_car_pos_dir()[0])+1)
        exit_target_index = self.get_index(self.closest_target(exit_extremity.get_start_car_pos_dir()[0])-1)
        target_index = self.get_index(start_target_index+current_target_index)
        if car:
            btw_index = self.get_index(target_index-1)
            if car in self.cars_between_targets[self.get_index(btw_index-1)]:
                self.cars_between_targets[self.get_index(btw_index-1)].remove(car)
            if target_index != exit_target_index:
                self.cars_between_targets[btw_index].append(car)
        return self.targets[target_index], exit_target_index == target_index
    
    def can_car_enter(self, extremity):
        """
        Returns True if there is enough space for the car to enter the roundabout (no car within min_distance meters).
        """
        start_target_index = self.get_index(self.closest_target(extremity.get_other_extremity().get_end_car_pos_dir()[0])+1)
        for i in range(self.nb_target_to_check_before_enter):
            btw_index = self.get_index(start_target_index - i - 1)
            if self.cars_between_targets[btw_index]:
                return False
        return True