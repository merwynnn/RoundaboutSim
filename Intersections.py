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
        self.exists = []

        self.min_detection_range = REAL_CAR_LENGTH
        self.detection_angle_threshold = 50
        

    def update(self, dt):
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

        self.min_detection_range = REAL_CAR_LENGTH*1.5
        self.detection_angle_threshold = 50

        # Increased from 3 to 4 to look further ahead for congestion before entering the roundabout.
        self.nb_target_to_check_before_enter = 7

        self.speed_enter_threshold = 1.0 # m/s, cars moving slower than this are considered "slow" when checking if a car can enter the roundabout

        self.exits = []
        for exit_dir in exits_dir:
            self.exits.append(RoadExtremity((self.center.x + self.radius * exit_dir.x, self.center.y + self.radius * exit_dir.y), self))

        # Increased number of points from 14 to 20 for a smoother path around the roundabout.
        self.targets = self.get_evenly_spaced_points(20)[::-1]

        self.cars_between_targets = [[] for _ in self.targets]   # 0: between 0 and 1, 1: between 1 and 2, etc.

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
        start_target_index = self.get_index(self.closest_target(extremity.get_other_extremity().get_end_car_pos_dir()[0])+2)
        for i in range(self.nb_target_to_check_before_enter):
            btw_index = self.get_index(start_target_index - i - 1)
            if self.cars_between_targets[btw_index]:
                # Check for slow-moving cars in critical segments
                for car_in_segment in self.cars_between_targets[btw_index]:
                    if car_in_segment.speed > self.speed_enter_threshold or i < 2: # Speed threshold
                        return False 
                    
        return True

class RedLightIntersection(Intersection):
    def __init__(self, pos, exits_dir, light_duration=10, yellow_light_duration=3, size=LANE_WIDTH*6):
        super().__init__(pos)
        self.size = size
        self.exits_dir = exits_dir
        self.lights = {}  # N, E, S, W
        self.exits = []
        for i, exit_dir in enumerate(exits_dir):
            self.lights[tuple(exit_dir)] = "red"
            self.exits.append(RoadExtremity(self.pos + exit_dir * (self.size / 2), self))


        self.light_duration = light_duration
        self.yellow_light_duration = yellow_light_duration
        self.timer = 0
        self.current_green_pair = (tuple(self.exits_dir[2]), tuple(self.exits_dir[3]))  # N-S green initially

        self.min_detection_range = REAL_CAR_LENGTH * 2
        self.detection_angle_threshold = 70

    def update(self, dt):
        self.timer += dt

        total_cycle_time = self.light_duration + self.yellow_light_duration

        # Set all to red before green phase
        for exit_dir in self.exits_dir:
            self.lights[tuple(exit_dir)] = "red"


        if self.timer < self.light_duration:
            # Green light phase
            self.lights[self.current_green_pair[0]] = "green"
            self.lights[self.current_green_pair[1]] = "green"
        elif self.timer < total_cycle_time:
            # Yellow light phase
            self.lights[self.current_green_pair[0]] = "yellow"
            self.lights[self.current_green_pair[1]] = "yellow"
        else:
            # Switch to the other pair of lights
            self.lights[self.current_green_pair[0]] = "red"
            self.lights[self.current_green_pair[1]] = "red"
            
            # Toggle between N-S and E-W pairs
            if self.current_green_pair == (tuple(self.exits_dir[2]), tuple(self.exits_dir[3])):
                self.current_green_pair = (tuple(self.exits_dir[0]), tuple(self.exits_dir[1]))
            else:
                self.current_green_pair = (tuple(self.exits_dir[2]), tuple(self.exits_dir[3]))
            
            self.timer = 0  # Reset timer for the new cycle

    def can_car_enter(self, start_extremity):
        incoming_dir = start_extremity.road.dir if start_extremity.road.end_extremity == start_extremity else -start_extremity.road.dir
        
        # Find the closest exit_dir to the incoming_dir
        closest_dir = None
        min_angle = float('inf')
        for exit_dir in self.exits_dir:
            angle = abs(incoming_dir.angle_to(exit_dir))
            angle = (angle + 180) % 360 - 180
            if angle < min_angle:
                min_angle = angle
                closest_dir = exit_dir

        return self.lights.get(tuple(closest_dir)) == "green"

    def draw(self, win):
        # Draw the intersection asphalt
        size = self.size
        pos = self.pos

        p1 = pos + Vec2(-size / 2, -size / 2)
        p2 = pos + Vec2(size / 2, -size / 2)
        p3 = pos + Vec2(size / 2, size / 2)
        p4 = pos + Vec2(-size / 2, size / 2)

        p1_transformed = self.simulator.camera.apply(p1)
        p2_transformed = self.simulator.camera.apply(p2)
        p3_transformed = self.simulator.camera.apply(p3)
        p4_transformed = self.simulator.camera.apply(p4)

        pygame.draw.polygon(win, ROAD_COLOR, [p1_transformed, p2_transformed, p3_transformed, p4_transformed])

        # Draw the traffic lights
        light_radius = self.simulator.camera.get_scaled_value(0.5)
        for exit_dir_tuple, state in self.lights.items():
            exit_dir = Vec2(exit_dir_tuple)
            # Position the light near the intersection edge
            light_pos = pos + exit_dir * (size / 2)
            transformed_light_pos = self.simulator.camera.apply(light_pos)

            color = (128, 128, 128) # Grey for off
            if state == "red":
                color = (255, 0, 0)
            elif state == "yellow":
                color = (255, 255, 0)
            elif state == "green":
                color = (0, 255, 0)
            
            pygame.draw.circle(win, color, transformed_light_pos, light_radius)

    def get_next_target_position(self, start_extremity, exit_extremity, current_target_index, car=None):
        """current_target_index is the index of the target to reach (starting from 0)"""

        start_pos, start_dir = start_extremity.get_other_extremity().get_end_car_pos_dir()
        end_pos, end_dir = exit_extremity.get_start_car_pos_dir()

        dot_product = start_dir.dot(end_dir)
        cross_product = start_dir.cross(end_dir)

        # Straight
        if cross_product == 0:
            return end_pos, True
        
        # Right turn
        elif cross_product > 0.9:
            control_point = start_pos + start_dir * self.size * 0.3
            path = [start_pos, control_point, end_pos]
        # Left turn
        else:
            control_point1 = start_pos + start_dir * self.size * 0.4
            path = [start_pos, control_point1, self.pos, end_pos]

        is_last_pos = current_target_index >= len(path) - 1
        
        if is_last_pos:
            return end_pos, True
        else:
            return path[current_target_index], False
