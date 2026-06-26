from typing import List, Optional, Tuple
import numpy as np
import pygame
from pygame import Vector2 as Vec2
from Constants import *
import math
from collections import deque
import matplotlib.pyplot as plt
import cmath

pygame.font.init()  # Initialize font module


class Car:

    def __init__(self,
                 simulator,
                 path,
                 creation_tick,
                 car_image=None,
                 target_position=None, id=0):
        self.simulator = simulator

        self.pos, self.dir = path[0].get_start_car_pos_dir() if path else (
            Vec2(0, 0), Vec2(1, 0))

        self.status = "APPROACHING"  

        self.speed = 0
        self.max_speed = MAX_SPEED  # Vitesse maximale autorisée
        self.acceleration = 0

        # model parameters
        self.target_speed = 0  # Automaticaly set

        ## Helly model parameters
        self.alpha= 0.4
        self.beta = 2
        self.gamma = 0.4

        self.desired_distance = TARGET_DISTANCE

        self.following_car = None
        self.leading_car = None

        self.creation_tick = creation_tick

        # Path
        self.path = path

        self.detection_angle = 220
        self.detection_angle_ring_road = 50
        self.detection_rotation_angle = 0
        self.detection_range = self.desired_distance * 4

        # Targets
        self.has_reached_last_intersection_target = False

        # Extremities
        self.last_extremity = path[0] if path else None
        self.current_target_extremity = self.path[0] if path else None
        self.current_target_position = self.get_next_target_position(
        ) if not target_position else target_position

        self.current_target_index = 0
        self.intersection_slowing_range = 20
        self.intersection_slowing_part_max_speed = 3
        self.intersection_checking_range = 6

        self.can_enter_intersection = False

        self.distance_to_intersection = math.inf
        self.distance_on_exit_road = math.inf

        self.critical_distance = math.sqrt((REAL_CAR_WIDTH / 2)**2 +
                                           (REAL_CAR_LENGTH / 2)**2) * 2.25 * 1.5

        self.distance_to_obstacle = 0

        self.selected = False  # Add selected attribute

    def check_front(self):

        # When exiting, only look at the direct front to avoid being too much influenced by cars on the side when trying to exit
        detection_angle = self.detection_angle if self.status == "APPROACHING" or self.status == "EXITING" else self.detection_angle_ring_road
        if self.current_target_extremity:
            if self.current_target_extremity.intersection is None:
                detection_angle = 1 


        cos_seuil = math.cos(math.radians(detection_angle/2))
        
        closest_car_distance = math.inf 
        
        car = None
        
        cars = self.simulator.cars
        
        for other_car in cars:
            if other_car is self:
                continue
            if other_car.status == "APPROACHING" and self.status == "IN_RING_ROAD":
                continue

            start_pos = self.pos
            vector_to_other = other_car.pos - start_pos
            distance = vector_to_other.length()
            if 0 < distance: # < self.detection_range:
                

                if vector_to_other.length_squared() > 1e-6:
                    v_normalized = vector_to_other.normalize()
                    if v_normalized.dot(self.dir) <= cos_seuil:
                        continue  # La voiture est derrière ou exactement sur le côté, ignorer
                        # La voiture est devant et dans la portée
                    if distance < closest_car_distance:
                        closest_car_distance = distance
                        car = other_car
        
        # Mémorise la voiture détectée la plus proche devant soi
        if self.simulator.total_ticks > 2:
            if self.leading_car:
                if self.leading_car.following_car is self:
                    self.leading_car.following_car = None
            self.leading_car = car  

            if self.status == "IN_RING_ROAD" and self.leading_car:
                self.leading_car.following_car = self

        return closest_car_distance, car


    def move(self, dt):
        if self.current_target_position:
            target_vector = self.current_target_position - self.pos
            distance = target_vector.length()
            if distance > self.critical_distance / 2:  # target position not reached
                
                target_dir = target_vector.normalize()
                self.dir = target_dir

            else:
                # current target position reached
                if len(self.path) == 1:
                    self.simulator.car_reached_destination(self)
                    return

                self.current_target_position = self.get_next_target_position()
                return


            distance_to_obstacle, obstacle = self.check_front()

            self.distance_to_obstacle = distance_to_obstacle

            # Handle collisions
            if self.distance_to_obstacle<self.critical_distance:
                self.acceleration = 0
                new_speed = obstacle.speed * 0.5
                if new_speed < self.speed:
                    self.speed = new_speed
                return 

            # If the car is approaching an intersection, slow down enough to enter the intersection
            self.target_speed = self.max_speed

            leading_car_speed = 0
            if obstacle:
                leading_car_speed = obstacle.speed


            # Helly model for acceleration
            if not obstacle:
                self.acceleration = -self.beta * (self.speed - self.target_speed)
            else:
                self.acceleration = self.alpha* (
                    self.distance_to_obstacle - self.desired_distance) - self.beta * (self.speed - self.target_speed) + self.gamma * (
                        leading_car_speed - self.speed) 

        else:  # No current target position -> stop the car

            print("No target position")
            self.acceleration = 0
            self.current_target_position = self.get_next_target_position()

        # --- Mise à jour de la position --- Euler semi implicite

        self.acceleration = self.alpha* (
                    self.distance_to_obstacle - self.desired_distance) - self.beta * 
                        (self.speed - self.target_speed) + self.gamma * (
                        leading_car_speed - self.speed) 
        
        dv = self.acceleration * dt
        self.speed += dv
        self.speed = max(0, self.speed)

        dpos = self.dir * self.speed * dt
        self.pos += dpos


    def get_next_target_extremity(self):
        self.path.pop(0)
        return self.path[0]


    def get_next_target_position(self):
        # Sortie imminente du périphérique
        if self.status == "IN_RING_ROAD" and self.has_reached_last_intersection_target:
            self.has_reached_last_intersection_target = False
            self.status = "EXITING"
            self.last_extremity = self.current_target_extremity
            return self.current_target_extremity.get_start_car_pos_dir(delta=-0.6)[0]

        # Approche ou circulation dans le périphérique
        if self.status in ("IN_RING_ROAD", "APPROACHING"):
            if self.status == "APPROACHING":
                # Attente si la voiture ne peut pas s'insérer
                lead = self.leading_car
                if lead and lead.following_car and (lead.following_car.pos - self.pos).length() < (lead.pos - self.pos).length() < self.critical_distance:
                    return self.current_target_position
                
                self.last_extremity, self.current_target_extremity = self.current_target_extremity, self.get_next_target_extremity()
                self.status = "IN_RING_ROAD"

            # Calcul de la prochaine position cible dans le périphérique
            self.can_enter_intersection = False
            inter = self.last_extremity.intersection
            
            if self.path:
                next_pos, is_last = inter.get_next_target_position(self.last_extremity, self.current_target_extremity, self.current_target_index, car=self)
            else:
                next_pos, is_last = inter.targets[inter.get_index(self.current_target_index + 1)], False

            self.current_target_index += 1
            if is_last:
                self.has_reached_last_intersection_target = True
                self.current_target_index = 0
            return next_pos

        # Fin de la phase de sortie
        if self.status == "EXITING":
            self.can_enter_intersection = False
            self.last_extremity, self.current_target_extremity = self.current_target_extremity, self.get_next_target_extremity()
            self.status = "APPROACHING"

            target_pos, self.dir = self.last_extremity.get_end_car_pos_dir()
            self.pos = self.last_extremity.get_start_car_pos_dir(delta=-0.6)[0]
            return target_pos

        self.last_extremity, self.current_target_extremity = self.current_target_extremity, self.get_next_target_extremity()
        return self.get_next_target_position()

    def draw(self, win):

        if not self.simulator.camera.is_point_on_screen(self.pos, margin=40):
            return
        # Car image scaling
        scaled_car_size = int(
            self.simulator.camera.get_scaled_value(
                REAL_CAR_LENGTH))  # Scale based on car height
        if scaled_car_size < 1: scaled_car_size = 1

        # Scale the car_image (which is the one from preloaded_car_images)
        temp_scaled_image = pygame.transform.scale(
            self.car_image, (scaled_car_size, scaled_car_size))

        # Rotate
        angle_degrees = self.dir.angle_to(Vec2(1, 0))
        rotated_image = pygame.transform.rotate(temp_scaled_image,
                                                angle_degrees - 90)

        # Apply camera transformation to the car's center position
        transformed_center = self.simulator.camera.apply(self.pos)
        new_rect = rotated_image.get_rect(center=transformed_center)

        # Dessiner l'image rotatée
        win.blit(rotated_image, new_rect.topleft)

        if self.selected:
            scaled_offset = self.simulator.camera.get_scaled_value(
                0.1)  # Scale offset for selection box
            scaled_line_thickness = max(
                1, int(self.simulator.camera.get_scaled_value(
                    0.3)))  # Scale line thickness
            selection_rect = pygame.Rect(new_rect.left - scaled_offset,
                                         new_rect.top - scaled_offset,
                                         new_rect.width + 2 * scaled_offset,
                                         new_rect.height + 2 * scaled_offset)
            pygame.draw.rect(win, (0, 255, 0), selection_rect,
                             scaled_line_thickness)
        if self.simulator.debug:
            # Get detection distance once
            closest_obstacle_distance, _ = self.check_front()

            # Display detection distance
            if closest_obstacle_distance != math.inf:
                
                text_surface = font.render(f"{self.speed*3.6:.1f} km/h",
                                           True, (255, 255, 255))  # White text
                text_rect = text_surface.get_rect(
                    center=(transformed_center.x, transformed_center.y -
                            scaled_car_size // 2 - 10))  # Above the car
                win.blit(text_surface, text_rect)

            debug_circle_radius = max(
                1, int(self.simulator.camera.get_scaled_value(0.5)))
            debug_line_thickness = max(
                1, int(self.simulator.camera.get_scaled_value(0.3)))

            if self.current_target_position:
                target_pos_to_draw = self.current_target_position
                transformed_debug_target = self.simulator.camera.apply(
                    target_pos_to_draw)
                pygame.draw.circle(win, (0, 0, 255), transformed_debug_target,
                                   debug_circle_radius)

            color_value = 0
            # self.detection_range is in world units, no direct scaling here for the logic.
            if closest_obstacle_distance == math.inf or self.detection_range == 0:
                color = (0, 255, 0)
            else:
                color_value = int(
                    255 -
                    (closest_obstacle_distance / self.detection_range) * 255)
                color_value = max(0, min(255, color_value))
                color = (0, color_value, 0)

            # world_start_point uses self.car_height (world unit)
            world_start_point = self.pos
            transformed_start_point = self.simulator.camera.apply(
                world_start_point)
            
            target_point = self.simulator.camera.apply(self.pos + self.dir * 10)
            #pygame.draw.line(win, (255, 0, 0), transformed_start_point, target_point, debug_line_thickness)
            """if self.leading_car:
                target_point = self.simulator.camera.apply(self.leading_car.pos)
                pygame.draw.line(win, (255, 255, 0), transformed_start_point, target_point, debug_line_thickness)
                pygame.draw.circle(win, (255, 255, 0), target_point, debug_circle_radius)"""
            detection_angle = self.detection_angle if self.status == "APPROACHING" else self.detection_angle_ring_road
            if self.dir.length_squared() > 0:
                angle_degrees += -self.detection_rotation_angle
                # world_p1/p2 use self.detection_range (world unit)
                world_p1 = world_start_point + Vec2(
                    self.detection_range,
                    0).rotate(-detection_angle/2 - angle_degrees)
                world_p2 = world_start_point + Vec2(
                    self.detection_range,
                    0).rotate(detection_angle/2 - angle_degrees)

                draw_p1 = self.simulator.camera.apply(world_p1)
                draw_p2 = self.simulator.camera.apply(world_p2)

                pygame.draw.line(win, color, transformed_start_point, draw_p1,debug_line_thickness)
                pygame.draw.line(win, color, transformed_start_point, draw_p2,debug_line_thickness)

                # Arc drawing: The radius used for pygame.Rect should be scaled.
                scaled_arc_display_radius = self.simulator.camera.get_scaled_value(
                    self.detection_range)
                debug_arc_rect_size = scaled_arc_display_radius * 2
                debug_arc_rect = pygame.Rect(
                    transformed_start_point.x - scaled_arc_display_radius,
                    transformed_start_point.y - scaled_arc_display_radius,
                    debug_arc_rect_size, debug_arc_rect_size)
                try:
                    pygame.draw.arc(
                        win, color, debug_arc_rect,
                        math.radians(angle_degrees -
                                     detection_angle/2),
                        math.radians(angle_degrees +
                                     detection_angle/2),
                        debug_line_thickness)
                except Exception:
                    pass
            
    def handle_click(
            self,
            world_pos):  # world_pos is from simulator.camera.screen_to_world
        distance_to_car_center = (world_pos - self.pos).length()
        # car_width and car_height are world units if not scaled with image.
        # For click detection, using a fixed world-unit radius is reasonable.
        click_radius = (BASE_CAR_LENGTH / 8)  # Fixed radius in world units
        return distance_to_car_center < click_radius
