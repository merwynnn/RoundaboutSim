from typing import List, Optional, Tuple
import pygame
from pygame import Vector2 as Vec2
from Constants import *
import math
from collections import deque
pygame.font.init() # Initialize font module

class Car:
    
    def __init__(self, path, creation_tick, car_image=None):
        from Simulator import Simulator
        self.simulator = Simulator.get_instance()

        self.creation_tick = creation_tick

        self.pos, self.dir = path[0].get_start_car_pos_dir()

        self.selected = False # Add selected attribute

        self.speed = 0
        self.max_speed = 13.8889 # Vitesse maximale autorisée
        self.acceleration = 0
        self.max_intersection_speed = 8.33333

        # model parameters
        self.target_speed = 0       # Automaticaly set
        self.max_acceleration = 1.4
        self.desired_acceleration = 2
        self.jam_distance = 2
        self.safe_time_gap = 1.5
        
        self.max_deceleration = -2 # Increased max_deceleration for quicker stops
        
        self.reaction_time_ms = REACTION_TIME 
        self.acceleration_queue = deque([0] * int(self.reaction_time_ms), maxlen=int(self.reaction_time_ms))

        self.steering_speed = 40

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

        
        self.min_detection_range_normal = REAL_CAR_LENGTH
        self.detection_angle_threshold_normal = 70
        self.detection_rotation_angle_normal = 0
        
        self.min_detection_range = self.min_detection_range_normal
        self.detection_range = self.min_detection_range_normal
        self.detection_angle_threshold = self.detection_angle_threshold_normal
        self.detection_rotation_angle = self.detection_rotation_angle_normal
        
        # Targets
        self.status = "EXITING"
        self.reached_last_intersection_target = False
        self.current_target_extremity = None
        self.current_target_position = None
        self.current_target_extremity = self.path[0]
        self.current_target_position = self.get_next_target_position()
        self.current_target_index = 0

        self.target_delta = 0
        
        self.intersection_slowing_range = 20
        self.intersection_slowing_part_max_speed = 3
        self.intersection_checking_range = 6

        self.can_enter_intersection = False

        self.distance_to_intersection = math.inf
        self.distance_on_exit_road = math.inf

        self.critical_distance = math.sqrt((REAL_CAR_WIDTH/2)**2 + (REAL_CAR_LENGTH/2)**2)*2.25

        self.v_ema = 0

        # état courant et timestamp du dernier changement d'état
        self.state = FREE
        self.state_time = 0.0    # temps passé dans l'état courant (s)

    def update_ema(self, dt: float = DT, tau: float = TAU):
        """ Mise à jour discrète de l'EMA (équation différentielle). """
        # Euler explicite stable si dt << tau
        self.v_ema += (self.v - self.v_ema) * (dt / tau)

    def detect_state(self,
                     bottleneck_intervals: Optional[List[Tuple[float, float]]] = None,
                     v_free: float = V_FREE,
                     v_cong: float = V_CONG,
                     dv_up: float = DV_UP,
                     dv_down: float = DV_DOWN,
                     dt: float = DT,
                     t_min_state: float = T_MIN_STATE):
        """
        Détecte et met à jour l'état du véhicule en utilisant v et v_ema.
        bottleneck_intervals : liste de couples (x_begin, x_end) en mètres.
        """
        # 1) Calcul des flags locaux (bool)
        is_free = (self.v_ema > v_free)
        is_jam = (self.v_ema < v_cong)
        dv_now = self.v - self.v_ema
        is_up = (dv_now < -dv_up)
        is_down = (dv_now > dv_down)

        # 2) Bottleneck spatial (si fourni)
        in_bottleneck = False
        if bottleneck_intervals:
            for xb, xe in bottleneck_intervals:
                if xb < self.x < xe:
                    in_bottleneck = True
                    break

        # 3) Construire liste d'états candidats (vrais)
        candidates = []
        if is_down: candidates.append(DOWN)
        if in_bottleneck: candidates.append(BOTTLENECK)
        if is_jam: candidates.append(JAM)
        if is_up: candidates.append(UP)
        if is_free: candidates.append(FREE)

        # 4) Choix selon priorité
        new_state = None
        for s in PRIORITY:
            if s in candidates:
                new_state = s
                break

        # Si aucun critère n'est vrai, garder l'état courant (pas de changement)
        if new_state is None:
            new_state = self.state

        # 5) Hysteresis / dwell time : n'accepter changement d'état
        # que si on est resté T_MIN_STATE dans l'état courant, sauf si changement to DOWN
        # (on peut décider que DOWN est prioritaire et immédiat — ici on applique dwell sauf pour DOWN)
        if new_state != self.state:
            # autoriser le passage vers DOWN immédiatement (priorité de sécurité)
            if new_state == DOWN:
                accept = True
            else:
                accept = (self.state_time >= t_min_state)
            if accept:
                self.state = new_state
                self.state_time = 0.0  # reset timer
            else:
                # garder ancien état, incrémenter timer plus bas (voir update_state)
                pass

        # si pas de changement accepté, on garde l'état et on accumule time elsewhere


    def check_front(self):
        
        closest_car_distance = math.inf # Initialise avec l'infini pour trouver le minimum
        car = None

        cars = self.simulator.spatial_grid.get_cars_in_neighborhood(self)
        for other_car in cars:
            if other_car is self or (self.status=="APPROACHING" and other_car.status == "APPROACHING" and other_car.current_target_extremity.intersection != self.current_target_extremity.intersection)or ((self.status == "INTERSECTION" or self.status == "EXITING") and other_car.status == "APPROACHING" and other_car.current_target_extremity.intersection == self.current_target_extremity.intersection):
                continue
            
            start_pos = self.pos
            vector_to_other = other_car.pos - start_pos
            distance = vector_to_other.length()
            if 0 < distance < self.detection_range:

                if vector_to_other.length_squared() > 1e-6: 
                    try:
                        angle = self.dir.angle_to(vector_to_other) - self.detection_rotation_angle
                        angle = (angle + 180) % 360 - 180
                        if abs(angle) < self.detection_angle_threshold:
 
                            # La voiture est devant et dans la portée
                            if distance < closest_car_distance:
                                closest_car_distance = distance
                                car = other_car
                    except ValueError:
                         print(f"Warning: ValueError pendant le calcul d'angle pour la voiture à {self.pos}")
                         continue # Passer à la voiture suivante

        return closest_car_distance, car

    def update_reaction_time(self, dt):
        if dt > 0:
            new_maxlen = int(self.reaction_time_ms / dt)
            if new_maxlen != self.acceleration_queue.maxlen:
                new_queue = deque([0] * new_maxlen, maxlen=new_maxlen)
                # Keep the most recent values
                for i in range(min(len(self.acceleration_queue), new_maxlen)):
                    new_queue.append(self.acceleration_queue.pop())
                self.acceleration_queue = new_queue

    def move(self, dt):
        self.update_reaction_time(dt)

        # La formule est simple. Prenez le chiffre des dizaines (5 pour 50 km/h) et multipliez-le par 3 (5 x 3 = 15). Puis, multipliez ce résultat par 2 (15 x 2 = 30). Vous obtenez la distance approximative à maintenir entre vous et le véhicule de devant (pour l’exemple 30 mètres). Facile non !?

        """speed_km_h = self.speed * 3.6
        tens_digit = int(speed_km_h // 10)
        self.detection_range = max(self.min_detection_range, tens_digit * 3 * 2) + REAL_CAR_LENGTH"""
        
        self.detection_range = max(self.min_detection_range, self.speed*2) + REAL_CAR_LENGTH

        if self.current_target_position:
            target_vector = self.current_target_position - self.pos
            distance = target_vector.length()
            if distance > self.critical_distance/2: # target position not reached

                target_dir = target_vector.normalize()
                # Tourner progressivement vers la cible
                angle = self.dir.angle_to(target_dir)
                angle = (angle + 180) % 360 - 180
                if abs(angle) > self.steering_speed:
                    self.dir = self.dir.rotate(self.steering_speed * math.copysign(1, angle) * dt)
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
                """
                if obstacle:
                    if obstacle.speed > 1.5:
                        distance_to_obstacle = math.inf """

            self.distance_on_exit_road = math.inf
            if (self.status == "INTERSECTION" and self.reached_last_intersection_target) or self.status == "EXITING":
                """
                Find the closest car on the exit road, in front of the intersection exit.
                This is to detect if there's a car blocking the exit of the intersection or on the road after the intersection.
                """
                exit_road = self.current_target_extremity if self.status == "INTERSECTION" else self.last_extremity
                exit_point, exit_dir = exit_road.get_start_car_pos_dir()

                for other_car in self.simulator.spatial_grid.get_cars_in_neighborhood(self):
                    if other_car is self:
                        continue

                    # Project the vector from the exit point to the other car onto the exit direction
                    vec_from_exit_to_other = other_car.pos - exit_point
                    proj = vec_from_exit_to_other.dot(exit_dir)

                    if proj > 0: # If the car is physically after the exit point of the intersection
                        # Check if the car is reasonably aligned with the road
                        # and close enough to be a concern.
                        dist_to_road_line = (exit_point + proj * exit_dir) - other_car.pos
                        if dist_to_road_line.length() < 1.875:
                            
                            # It's on the road, calculate distance from our car
                            dist_to_other_car = (other_car.pos - self.pos).length()
                            
                            if dist_to_other_car < self.distance_on_exit_road:
                                self.distance_on_exit_road = dist_to_other_car

            self.distance_to_intersection = math.inf

            if self.status == "APPROACHING" and self.current_target_extremity.intersection:     # Approaching intersection
                # Calculate distance to intersection
                self.distance_to_intersection = (self.last_extremity.get_end_car_pos_dir(delta=3)[0] - self.pos).length()
                
                # Check if the car is within the checking range to start checking if it can enter the intersection
                if (self.last_extremity.get_end_car_pos_dir(delta=self.target_delta)[0]-self.pos).length() < self.intersection_checking_range: 
                    # If the car has reached the target, check if it can enter the intersection
                    if self.can_enter_intersection:
                        # If it can enter the intersection, set the distance to infinity
                        self.distance_to_intersection = math.inf
                    else:
                        # If it cannot enter the intersection, check if the current target's intersection can let the car enter
                        if self.current_target_extremity.intersection.can_car_enter(self.current_target_extremity):
                            # If it can enter, set the distance to infinity and mark that the car can enter the intersection
                            self.distance_to_intersection = math.inf
                            self.can_enter_intersection = True


            d = min(distance_to_obstacle, self.distance_to_intersection, self.distance_on_exit_road)
            if distance_to_obstacle<self.critical_distance:  # Collision imminent
                self.acceleration = 0
                self.speed = 0
                return  

            # Determine max_speed based on context (intersection or straight road)
            current_max_speed = self.max_speed if self.status == "APPROACHING" else self.max_intersection_speed


            # If the car is approaching an intersection, slow down enough to enter the intersection
            self.target_speed = current_max_speed
            if self.status == "APPROACHING":
                detection_range = self.intersection_slowing_range + REAL_CAR_LENGTH/2
                if self.distance_to_intersection < detection_range:
                    # The car should slow down to the intersection slowing part max speed
                    self.target_speed = self.intersection_slowing_part_max_speed
                    
            leading_car_speed = 0
            if obstacle:
                leading_car_speed = obstacle.speed
            desired_distance = self.jam_distance + self.speed*self.safe_time_gap + self.speed*(abs(leading_car_speed-self.speed))/(2*math.sqrt(self.max_acceleration*self.desired_acceleration))

            self.acceleration = self.max_acceleration*(1 - (self.speed/self.target_speed)**4 - (desired_distance/d)**2)

            
            self.acceleration_queue.append(self.acceleration)

        else:  # No current target position -> stop the car
            self.acceleration = -self.alpha
            self.current_target_position = self.get_next_target_position()
            

        # --- Mise à jour de la position ---
        dv = self.acceleration * dt
        self.speed += dv
        #self.speed += self.acceleration_queue[0] * dt

        # max_speed was already determined above for the acceleration calculation
        self.speed = max(0, min(self.speed, current_max_speed))
        
        dpos = self.dir * self.speed * dt
        self.pos += dpos

    def get_next_target_extremity(self):
        self.path.pop(0)        
            
        return self.path[0]

    def get_next_target_position(self):
        if self.status=="INTERSECTION" and self.reached_last_intersection_target:
            self.reached_last_intersection_target = False
            self.status = "EXITING"
            self.last_extremity = self.current_target_extremity


            return self.current_target_extremity.get_start_car_pos_dir(delta=-0.6)[0]
        
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

                        
            self.min_detection_range = self.last_extremity.intersection.min_detection_range
            self.detection_angle_threshold = self.last_extremity.intersection.detection_angle_threshold
            self.detection_rotation_angle = self.last_extremity.intersection.detection_rotation_angle
            



            return next_target_pos
    
        elif self.status=="EXITING":
            self.can_enter_intersection = False
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            self.status="APPROACHING"

            self.min_detection_range = self.min_detection_range_normal
            self.detection_angle_threshold = self.detection_angle_threshold_normal
            self.detection_rotation_angle = self.detection_rotation_angle_normal


            return self.last_extremity.get_end_car_pos_dir()[0]
        
        else:   # Entering intersection
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            return self.get_next_target_position()

        

    def draw_rect(self, win):
        # Car rectangle scaling
        rect_width = self.simulator.camera.get_scaled_value(REAL_CAR_WIDTH)
        rect_height = self.simulator.camera.get_scaled_value(REAL_CAR_LENGTH)
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
        
        if not self.simulator.camera.is_point_on_screen(self.pos, margin=40):
            return
        # Car image scaling
        scaled_car_size = int(self.simulator.camera.get_scaled_value(REAL_CAR_LENGTH))  # Scale based on car height
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
            scaled_offset = self.simulator.camera.get_scaled_value(0.1) # Scale offset for selection box
            scaled_line_thickness = max(1, int(self.simulator.camera.get_scaled_value(0.3))) # Scale line thickness
            selection_rect = pygame.Rect(
                new_rect.left - scaled_offset,
                new_rect.top - scaled_offset,
                new_rect.width + 2 * scaled_offset,
                new_rect.height + 2 * scaled_offset
            )
            pygame.draw.rect(win, (0, 255, 0), selection_rect, scaled_line_thickness)
        if self.simulator.debug:
            # Get detection distance once
            closest_obstacle_distance, _ = self.check_front()

            # Display detection distance
            if closest_obstacle_distance != math.inf:
                font_size = max(10, int(self.simulator.camera.get_scaled_value(1.5)))
                font = pygame.font.Font(None, font_size)
                text_surface = font.render(f"{int(closest_obstacle_distance)}", True, (255, 255, 255)) # White text
                text_rect = text_surface.get_rect(center=(transformed_center.x, transformed_center.y - scaled_car_size // 2 - 10)) # Above the car
                win.blit(text_surface, text_rect)


            debug_circle_radius = max(1, int(self.simulator.camera.get_scaled_value(0.5)))
            debug_line_thickness = max(1, int(self.simulator.camera.get_scaled_value(0.3)))

            if self.current_target_position:
                target_pos_to_draw = self.current_target_position
                transformed_debug_target = self.simulator.camera.apply(target_pos_to_draw)
                pygame.draw.circle(win, (0, 0, 255), transformed_debug_target, debug_circle_radius)

            color_value = 0
            # self.detection_range is in world units, no direct scaling here for the logic.
            if closest_obstacle_distance == math.inf or self.detection_range == 0:
                 color = (0,255,0)
            else:
                color_value = int(255 - (closest_obstacle_distance / self.detection_range) * 255)
                color_value = max(0, min(255, color_value))
                color = (0, color_value, 0)

            # world_start_point uses self.car_height (world unit)
            world_start_point = self.pos
            transformed_start_point = self.simulator.camera.apply(world_start_point)
    
            if self.dir.length_squared() > 0:
                angle_degrees += - self.detection_rotation_angle
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
        click_radius = (BASE_CAR_LENGTH/8)  # Fixed radius in world units
        return distance_to_car_center < click_radius
