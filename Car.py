import pygame
from pygame import Vector2 as Vec2
from Constants import *
import math
import numpy as np
from numba import jit

# Numba-jitted helper function for angle calculation
@jit(nopython=True, cache=True)
def numba_angle_to(dir_x, dir_y, vec_x, vec_y):
    # Calculate the dot product
    dot_product = dir_x * vec_x + dir_y * vec_y
    # Calculate the determinant (cross product in 2D)
    determinant = dir_x * vec_y - dir_y * vec_x
    # Calculate the angle in radians
    angle_rad = math.atan2(determinant, dot_product)
    # Convert to degrees
    angle_deg = math.degrees(angle_rad)
    return angle_deg

@jit(nopython=True, cache=True)
def _numba_check_front_calculations(
    car_pos_x, car_pos_y, car_dir_x, car_dir_y,
    car_status_val, # 0 for APPROACHING, 1 for INTERSECTION, 2 for EXITING
    car_current_target_extremity_intersection_id, # id of intersection, or -1 if None
    car_detection_range, car_detection_angle_threshold, car_height,
    other_cars_data # NumPy array: [pos_x, pos_y, status_val, current_target_extremity_intersection_id, id]
    ):
    closest_car_distance = np.inf
    closest_car_id = -1

    start_pos_x = car_pos_x + car_dir_x * car_height / 2
    start_pos_y = car_pos_y + car_dir_y * car_height / 2

    for i in range(other_cars_data.shape[0]):
        other_car_pos_x = other_cars_data[i, 0]
        other_car_pos_y = other_cars_data[i, 1]
        other_car_status_val = int(other_cars_data[i, 2])
        other_car_current_target_extremity_intersection_id = int(other_cars_data[i, 3])
        other_car_id = int(other_cars_data[i, 4]) # Store original index or unique ID

        # Filter based on status and intersection (ported from original logic)
        # if other_car is self (handled by not including self in other_cars_data)
        # or (self.status=="APPROACHING" and other_car.status == "APPROACHING" and other_car.current_target_extremity.intersection != self.current_target_extremity.intersection)
        # or ((self.status == "INTERSECTION" or self.status == "EXITING") and other_car.status == "APPROACHING" and other_car.current_target_extremity.intersection == self.current_target_extremity.intersection):
        if (car_status_val == 0 and other_car_status_val == 0 and \
            other_car_current_target_extremity_intersection_id != car_current_target_extremity_intersection_id and \
            car_current_target_extremity_intersection_id != -1 and other_car_current_target_extremity_intersection_id != -1) or \
           ((car_status_val == 1 or car_status_val == 2) and other_car_status_val == 0 and \
            other_car_current_target_extremity_intersection_id == car_current_target_extremity_intersection_id and \
            car_current_target_extremity_intersection_id != -1):
            continue

        vec_to_other_x = other_car_pos_x - start_pos_x
        vec_to_other_y = other_car_pos_y - start_pos_y

        distance_sq = vec_to_other_x**2 + vec_to_other_y**2
        if 0 < distance_sq < car_detection_range**2: # Compare squared distances
            distance = math.sqrt(distance_sq)
            if distance_sq > 1e-12: # Check for non-zero length before normalization/angle
                # Numba compatible angle calculation
                angle = numba_angle_to(car_dir_x, car_dir_y, vec_to_other_x, vec_to_other_y)
                # angle = (angle + 180) % 360 - 180 # Already handled by atan2 logic in numba_angle_to for -180 to 180 range

                if abs(angle) < car_detection_angle_threshold:
                    if distance < closest_car_distance:
                        closest_car_distance = distance
                        closest_car_id = other_car_id
    return closest_car_distance, closest_car_id


class Car:
    
    def __init__(self, path, creation_tick, car_image=None):
        from Simulator import Simulator
        self.simulator = Simulator.get_instance()

        self.creation_tick = creation_tick

        self.pos, self.dir = path[0].get_start_car_pos_dir()

        self.selected = False # Add selected attribute

        self.speed = 0
        self.max_speed = 3.0 # Vitesse maximale autorisée
        self.max_intersection_speed = 2.0
        self.acceleration = 0

        self.engine_force = 0.05  
        self.steering_speed = 5

        self.alpha = 5 # Increased alpha for stronger obstacle avoidance response
        self.beta = 3
        self.max_acceleration = 0.006
        self.max_deceleration = -0.03 # Increased max_deceleration for quicker stops

        # Extremities
        self.last_extremity = path[0]

        # Path
        self.path = path

        
        # Image
        if car_image is not None:
            self.car_image = car_image.copy()
        else:
            self.car_image = pygame.Surface((40, 40))
            self.car_image.fill((255, 0, 0))
        self.car_width = self.car_image.get_width() if self.car_image else 40
        self.car_height = self.car_image.get_height() if self.car_image else 40


        
        self.detection_range_normal = self.car_height * 2.5
        self.detection_angle_threshold_normal = 25.0 

        self.detection_range_angular = self.car_height * 1.7
        self.detection_angle_threshold_angular = 35.0
        
        self.detection_range = self.detection_range_normal
        self.detection_angle_threshold = self.detection_angle_threshold_normal
        
        # Targets
        self.status = "EXITING"
        self.reached_last_intersection_target = False
        self.current_target_extremity = None
        self.current_target_position = None
        self.current_target_extremity = self.path[0]
        self.current_target_position = self.get_next_target_position()
        self.current_target_index = 0

        self.intersection_delta = 0
        self.target_delta = -20

        self.can_enter_intersection = False

        self.distance_to_intersection = math.inf
        self.distance_on_exit_road = math.inf


        

    _status_to_int = {"APPROACHING": 0, "INTERSECTION": 1, "EXITING": 2}
    _int_to_status = {v: k for k, v in _status_to_int.items()}


    def check_front(self):
        # Get neighboring cars
        neighboring_cars_objects = self.simulator.spatial_grid.get_cars_in_neighborhood(self)

        # Prepare data for Numba function
        # Filter out self and prepare data for other cars
        other_cars_list_for_numba = []
        # Keep a mapping from the simple ID used in Numba back to car objects
        # Using list index as a simple ID for this call
        id_to_car_map = {}

        for i, other_car_obj in enumerate(neighboring_cars_objects):
            if other_car_obj is self:
                continue
            
            other_car_status_val = self._status_to_int.get(other_car_obj.status, -1) # Default to -1 if status unknown

            # Handle intersection ID for other_car
            other_car_intersection_id = -1
            if other_car_obj.current_target_extremity and other_car_obj.current_target_extremity.intersection:
                # Assuming intersection objects have a unique 'id' attribute or we can use hash() or some other unique int id
                # For now, let's assume intersection objects themselves are hashable or have an id.
                # This part might need adjustment based on how Intersection objects are identified.
                # Using object's Python id() if no specific id attribute exists.
                other_car_intersection_id = id(other_car_obj.current_target_extremity.intersection)


            other_cars_list_for_numba.append([
                other_car_obj.pos.x, other_car_obj.pos.y,
                other_car_status_val,
                other_car_intersection_id,
                i # Using list index as a temporary ID for this call
            ])
            id_to_car_map[i] = other_car_obj

        if not other_cars_list_for_numba:
            return math.inf, None

        other_cars_data_np = np.array(other_cars_list_for_numba, dtype=np.float64)

        # Current car data
        car_pos_x = self.pos.x
        car_pos_y = self.pos.y
        car_dir_x = self.dir.x
        car_dir_y = self.dir.y

        car_status_val = self._status_to_int.get(self.status, -1)

        car_current_target_extremity_intersection_id = -1
        if self.current_target_extremity and self.current_target_extremity.intersection:
            # Using object's Python id()
            car_current_target_extremity_intersection_id = id(self.current_target_extremity.intersection)


        closest_distance, closest_car_id_from_numba = _numba_check_front_calculations(
            car_pos_x, car_pos_y, car_dir_x, car_dir_y,
            car_status_val,
            car_current_target_extremity_intersection_id,
            self.detection_range, self.detection_angle_threshold, self.car_height,
            other_cars_data_np
        )

        if closest_car_id_from_numba != -1 and closest_car_id_from_numba in id_to_car_map:
            return closest_distance, id_to_car_map[closest_car_id_from_numba]
        else:
            return math.inf, None

    def move(self, dt):
        time_factor = dt / (1000.0 / 60.0) if dt > 0 else 1

        if self.current_target_position:
            target_vector = self.current_target_position - self.pos
            distance = target_vector.length()
            if distance > self.speed*5: # target position not reached

                target_dir = target_vector.normalize()
                # Tourner progressivement vers la cible
                angle = self.dir.angle_to(target_dir)
                angle = (angle + 180) % 360 - 180
                if abs(angle) > self.steering_speed:
                    self.dir = self.dir.rotate(self.steering_speed * math.copysign(1, angle) * time_factor)
                else:
                    self.dir = target_dir                    

            else:
                # current target position reached
                if len(self.path) == 1:
                    self.simulator.car_reached_destination(self)
                    return
                
                self.current_target_position = self.get_next_target_position()
                return

            # --- Accélération ---

            distance_to_obstacle, obstacle = self.check_front()

            if self.status == "INTERSECTION":
                if obstacle:
                    if obstacle.speed > 1.5:
                        distance_to_obstacle = math.inf

            self.distance_on_exit_road = math.inf
            if (self.status == "INTERSECTION" and self.reached_last_intersection_target) or self.status == "EXITING":
                exit_road = self.current_target_extremity if self.status == "INTERSECTION" else self.last_extremity
                exit_point, exit_dir = exit_road.get_start_car_pos_dir()

                for other_car in self.simulator.spatial_grid.get_cars_in_neighborhood(self):
                    if other_car is self:
                        continue

                    # Check if the other car is on the exit road, in front of the intersection exit
                    vec_from_exit_to_other = other_car.pos - exit_point
                    
                    # Project the vector onto the exit direction
                    proj = vec_from_exit_to_other.dot(exit_dir)

                    if proj > 0: # If the car is physically after the exit point of the intersection
                        # Check if the car is reasonably aligned with the road
                        # and close enough to be a concern.
                        dist_to_road_line = (exit_point + proj * exit_dir) - other_car.pos
                        if dist_to_road_line.length() < 15: # 15 pixels tolerance from road center line
                            
                            # It's on the road, calculate distance from our car
                            dist_to_other_car = (other_car.pos - self.pos).length()
                            
                            if dist_to_other_car < self.distance_on_exit_road:
                                self.distance_on_exit_road = dist_to_other_car

            self.distance_to_intersection = math.inf

            if self.status == "APPROACHING" and self.current_target_extremity.intersection:     # Approaching intersection
                self.distance_to_intersection = (self.last_extremity.get_end_car_pos_dir(delta=self.intersection_delta)[0] - self.pos).length() 
                if self.distance_to_intersection > self.detection_range:
                    self.distance_to_intersection = math.inf
                
                if (self.last_extremity.get_end_car_pos_dir(delta=self.target_delta)[0]-self.pos).length() < self.engine_force*500:    # reached target
                    if self.can_enter_intersection:
                        self.distance_to_intersection = math.inf
                    elif self.current_target_extremity.intersection.can_car_enter(self.current_target_extremity):
                        self.distance_to_intersection = math.inf
                        self.can_enter_intersection = True
            d = min(distance_to_obstacle, self.distance_to_intersection, self.distance_on_exit_road)
            if d<self.detection_range/5:
                self.acceleration = 0
                self.speed = 0
                return

            # Determine max_speed based on context (intersection or straight road)
            current_max_speed = self.max_speed if self.status == "APPROACHING" else self.max_intersection_speed
            
            new_acceleration = self.beta * (1- self.speed/current_max_speed) - self.alpha * (1-d/self.detection_range)
            ### make sure acceleration shift is not too important, if it is clamp
            if new_acceleration > self.acceleration:
                self.acceleration = min(new_acceleration, self.acceleration + self.max_acceleration)
            elif new_acceleration < self.acceleration:
                self.acceleration = max(new_acceleration, self.acceleration + self.max_deceleration)
            else:
                self.acceleration = new_acceleration

        else:
            self.acceleration = -self.engine_force*2
            self.current_target_position = self.get_next_target_position()
            

        # --- Mise à jour de la position ---
        self.speed += self.acceleration * time_factor
        # max_speed was already determined above for the acceleration calculation
        self.speed = max(0, min(self.speed, current_max_speed))

        if self.speed == current_max_speed or self.speed == 0:
            self.acceleration = 0

        speed = self.speed
        if self.speed < 0.2:
            speed = 0
            self.acceleration = 0

        self.acceleration = min(max(self.acceleration, -0.3), 0.3)
            
        self.pos += self.dir * speed * time_factor

    def get_next_target_extremity(self):
        self.path.pop(0)        
            
        return self.path[0]

    def get_next_target_position(self):
        if self.status=="INTERSECTION" and self.reached_last_intersection_target:
            self.reached_last_intersection_target = False
            self.status = "EXITING"
            self.last_extremity = self.current_target_extremity


            return self.current_target_extremity.get_start_car_pos_dir(delta=-5)[0]
        
        elif self.status=="INTERSECTION":  # is in intersection
            self.can_enter_intersection = False
            next_target_pos, is_last_pos = self.last_extremity.intersection.get_next_target_position(self.last_extremity, self.current_target_extremity, self.current_target_index, car=self)
            
            self.current_target_index += 1
            if is_last_pos:
                self.reached_last_intersection_target = True
                self.current_target_index = 0
                
            return next_target_pos
        elif self.status == "APPROACHING":
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            next_target_pos, is_last_pos = self.last_extremity.intersection.get_next_target_position(self.last_extremity, self.current_target_extremity, self.current_target_index, car=self)
            self.current_target_index += 1
            self.status = "INTERSECTION"

                        
            self.detection_range = self.detection_range_angular
            self.detection_angle_threshold = self.detection_angle_threshold_angular



            return next_target_pos
    
        elif self.status=="EXITING":
            self.can_enter_intersection = False
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            self.status="APPROACHING"

            self.detection_range = self.detection_range_normal
            self.detection_angle_threshold = self.detection_angle_threshold_normal

            return self.last_extremity.get_end_car_pos_dir()[0]
        
        else:   # Entering intersection
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            return self.get_next_target_position()

        

    def draw_rect(self, win):
        # Car rectangle scaling
        rect_width = self.simulator.camera.get_scaled_value(self.car_width*0.4)
        rect_height = self.simulator.camera.get_scaled_value(self.car_height*0.7)
        if rect_width < 1: rect_width = 1
        if rect_height < 1: rect_height = 1

        # Determine color based on speed
        if self.speed == 0:
            color = (255, 0, 0)  # Red for stationary
        else:
            # Interpolate from yellow (slow) to green (fast)
            speed_ratio = min(self.speed / self.max_speed, 1.0)
            red = int(255 * (1 - speed_ratio))
            blue = int(255 * speed_ratio)
            color = (red, 0, blue)

        # Create a surface for the rectangle
        rect_surface = pygame.Surface((rect_width, rect_height), pygame.SRCALPHA)
        rect_surface.fill(color)
        
        # Rotate the rectangle
        angle_degrees = self.dir.angle_to(Vec2(1, 0))
        rotated_surface = pygame.transform.rotate(rect_surface, angle_degrees - 90)
        
        # Apply camera transformation and draw
        transformed_center = self.simulator.camera.apply(self.pos)
        new_rect = rotated_surface.get_rect(center=transformed_center)
        win.blit(rotated_surface, new_rect.topleft)

    def draw(self, win):
        # Car image scaling
        base_car_size = 40 # Assuming the preloaded car_image is 40x40 at zoom_level 1.0
        scaled_car_size = int(self.simulator.camera.get_scaled_value(base_car_size))
        if scaled_car_size < 1: scaled_car_size = 1

        # Scale the car_image (which is the one from preloaded_car_images)
        temp_scaled_image = pygame.transform.scale(self.car_image, (scaled_car_size, scaled_car_size))

        # Rotate
        angle_degrees = self.dir.angle_to(Vec2(1, 0))
        rotated_image = pygame.transform.rotate(temp_scaled_image, angle_degrees-90)

        # Apply camera transformation to the car's center position
        transformed_center = self.simulator.camera.apply(self.pos)
        new_rect = rotated_image.get_rect(center = transformed_center)

        # Dessiner l'image rotatée
        

        if self.simulator.render_as_rect:
            self.draw_rect(win)
        else:
            win.blit(rotated_image, new_rect.topleft)

        if self.selected:
            scaled_offset = self.simulator.camera.get_scaled_value(5) # Scale offset for selection box
            scaled_line_thickness = max(1, int(self.simulator.camera.get_scaled_value(2))) # Scale line thickness
            selection_rect = pygame.Rect(
                new_rect.left - scaled_offset,
                new_rect.top - scaled_offset,
                new_rect.width + 2 * scaled_offset,
                new_rect.height + 2 * scaled_offset
            )
            pygame.draw.rect(win, (0, 255, 0), selection_rect, scaled_line_thickness)


        if self.simulator.debug and self.selected:
            debug_circle_radius = max(1, int(self.simulator.camera.get_scaled_value(5)))
            debug_line_thickness = max(1, int(self.simulator.camera.get_scaled_value(1)))

            if self.current_target_position:
                target_pos_to_draw = self.current_target_position
                transformed_debug_target = self.simulator.camera.apply(target_pos_to_draw)
                pygame.draw.circle(win, (0, 0, 255), transformed_debug_target, debug_circle_radius)

            d = self.check_front()[0]
            color_value = 0
            # self.detection_range is in world units, no direct scaling here for the logic.
            if d == math.inf or self.detection_range == 0:
                 color = (0,255,0)
            else:
                color_value = int(255 - (d / self.detection_range) * 255)
                color_value = max(0, min(255, color_value))
                color = (0, color_value, 0)

            # world_start_point uses self.car_height (world unit)
            world_start_point = self.pos + self.dir * self.car_height // 2
            transformed_start_point = self.simulator.camera.apply(world_start_point)

            if self.dir.length_squared() > 0:
                # world_p1/p2 use self.detection_range (world unit)
                world_p1 = world_start_point + Vec2(self.detection_range, 0).rotate(-self.detection_angle_threshold - angle_degrees)
                world_p2 = world_start_point + Vec2(self.detection_range, 0).rotate(self.detection_angle_threshold - angle_degrees)

                draw_p1 = self.simulator.camera.apply(world_p1)
                draw_p2 = self.simulator.camera.apply(world_p2)

                pygame.draw.line(win, color, transformed_start_point, draw_p1, debug_line_thickness)
                pygame.draw.line(win, color, transformed_start_point, draw_p2, debug_line_thickness)

                # Arc drawing: The radius used for pygame.Rect should be scaled.
                scaled_arc_display_radius = self.simulator.camera.get_scaled_value(self.detection_range)
                debug_arc_rect_size = scaled_arc_display_radius * 2
                debug_arc_rect = pygame.Rect(
                    transformed_start_point.x - scaled_arc_display_radius,
                    transformed_start_point.y - scaled_arc_display_radius,
                    debug_arc_rect_size,
                    debug_arc_rect_size
                )
                try:
                    pygame.draw.arc(win, color, debug_arc_rect,
                                    math.radians(angle_degrees - self.detection_angle_threshold),
                                    math.radians(angle_degrees + self.detection_angle_threshold), debug_line_thickness)
                except Exception: pass

    def handle_click(self, world_pos): # world_pos is from simulator.camera.screen_to_world
        distance_to_car_center = (world_pos - self.pos).length()
        # car_width and car_height are world units if not scaled with image.
        # For click detection, using a fixed world-unit radius is reasonable.
        click_radius = (self.car_width + self.car_height) / 4
        return distance_to_car_center < click_radius
