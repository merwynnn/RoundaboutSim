import pygame
from pygame import Vector2 as Vec2
from Constants import *
import math
import time # Added for time tracking

class Car:
    
    def __init__(self, path, car_image=None):
        from Simulator import Simulator
        self.simulator = Simulator.get_instance()

        self.pos, self.dir = path[0].get_start_car_pos_dir()

        self.start_time = time.time() # Track start time
        self.time_spent_in_intersection = 0.0 # Track time in current intersection
        self.time_entered_intersection = 0.0 # Timestamp when car enters an intersection

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
        self.path = path # This is a list of RoadExtremity objects
        self.full_path_for_min_time_calc = list(path) # Keep a copy for min time calculation

        
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


        


    def check_front(self):
        
        closest_car_distance = math.inf # Initialise avec l'infini pour trouver le minimum
        car = None

        for other_car in self.simulator.cars:
            if other_car is self or (self.status=="APPROACHING" and other_car.status == "APPROACHING" and other_car.current_target_extremity.intersection != self.current_target_extremity.intersection)or ((self.status == "INTERSECTION" or self.status == "EXITING") and other_car.status == "APPROACHING" and other_car.current_target_extremity.intersection == self.current_target_extremity.intersection):
                continue
            
            start_pos = self.pos+self.dir*self.car_height/2
            vector_to_other = other_car.pos - start_pos
            distance = vector_to_other.length()
            if 0 < distance < self.detection_range:

                if vector_to_other.length_squared() > 1e-6: 
                    try:
                        angle = self.dir.angle_to(vector_to_other)
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

                for other_car in self.simulator.cars:
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
            self.time_entered_intersection = time.time() # Record time entering intersection
                        
            self.detection_range = self.detection_range_angular
            self.detection_angle_threshold = self.detection_angle_threshold_angular



            return next_target_pos
    
        elif self.status=="EXITING":
            self.can_enter_intersection = False
            current_intersection_object = self.last_extremity.intersection # Intersection being exited

            if self.time_entered_intersection > 0 and current_intersection_object: # Ensure it was set and we have an intersection
                self.time_spent_in_intersection = time.time() - self.time_entered_intersection
                self.time_entered_intersection = 0 # Reset for next intersection

                min_crossing_time = current_intersection_object.get_min_crossing_time()
                if min_crossing_time > 0: # Avoid division by zero
                    if self.time_spent_in_intersection > min_crossing_time * LOCAL_CONGESTION_THRESHOLD:
                        current_intersection_object.register_local_congestion()
                        # print(f"Car {id(self)} was LOCALLY congested in intersection {id(current_intersection_object)}. Actual: {self.time_spent_in_intersection:.2f}s, Min: {min_crossing_time:.2f}s")
                current_intersection_object.register_car_passed()


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
                if hasattr(self.last_extremity, 'get_end_car_pos_dir'):
                    try:
                        target_pos_to_draw = self.last_extremity.get_end_car_pos_dir(delta=-10)[0]
                        transformed_debug_target = self.simulator.camera.apply(target_pos_to_draw)
                        pygame.draw.circle(win, (0, 0, 255), transformed_debug_target, debug_circle_radius)
                    except (AttributeError, TypeError, IndexError): pass

            try:
                if hasattr(self.last_extremity, 'get_end_car_pos_dir'):
                    transformed_debug_intersection = self.simulator.camera.apply(self.last_extremity.get_end_car_pos_dir(delta=self.intersection_delta)[0])
                    pygame.draw.circle(win, (255, 0, 255), transformed_debug_intersection, debug_circle_radius)
                    transformed_debug_target_delta = self.simulator.camera.apply(self.last_extremity.get_end_car_pos_dir(delta=self.target_delta)[0])
                    pygame.draw.circle(win, (255, 255, 255), transformed_debug_target_delta, debug_circle_radius)
            except (AttributeError, TypeError, IndexError): pass

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

    def calculate_min_travel_time(self):
        """
        Calculates the minimum theoretical travel time for the car's full path.
        This is based on road lengths and intersection crossing estimates.
        """
        if not self.full_path_for_min_time_calc:
            return 0.0

        total_min_time = 0.0

        # Iterate through the path segments (extremity to extremity)
        for i in range(len(self.full_path_for_min_time_calc) - 1):
            current_ext = self.full_path_for_min_time_calc[i]
            next_ext = self.full_path_for_min_time_calc[i+1]

            if current_ext.road and next_ext.road and current_ext.road == next_ext.road:
                # Segment is on a road
                # The path gives extremities. If two consecutive extremities are on the same road,
                # it means the car is traversing that road.
                road_length = current_ext.road.length
                if self.max_speed > 0:
                    total_min_time += road_length / self.max_speed
                # If max_speed is 0, time would be infinite, handle as appropriate (e.g., skip or add a large penalty)

            elif current_ext.intersection and next_ext.intersection and current_ext.intersection == next_ext.intersection:
                # Segment is within an intersection (e.g. from an entry point to an exit point of the *same* intersection)
                # This case implies the path is detailed enough to list intermediate points within an intersection,
                # or that an "extremity" can also be an internal point of an intersection.
                # For now, we use the intersection's estimated crossing time.
                total_min_time += current_ext.intersection.get_min_crossing_time()

            elif current_ext.intersection and not next_ext.intersection:
                # Exiting an intersection onto a road - time already accounted for by intersection's min_crossing_time
                # or will be by the next road segment.
                pass # Or add specific logic if needed

            elif not current_ext.intersection and next_ext.intersection:
                # Entering an intersection from a road - time for road segment already added.
                # The intersection crossing time will be added when the path segment is *within* the intersection.
                 total_min_time += next_ext.intersection.get_min_crossing_time()


            # Consider transitions: Road -> Intersection -> Road
            # If current_ext is on a road and next_ext is an entry to an intersection,
            # the road travel time is covered by the road segment.
            # The time to cross the intersection is handled when path segments are *within* the intersection
            # or by a general estimate if the path doesn't detail internal intersection points.

        # A simple heuristic: sum of road travel times + sum of intersection crossing times
        # This requires careful path definition. If path is [RoadExt1_Entry, RoadExt1_Exit, IntExt1_Entry, IntExt1_Exit, RoadExt2_Entry, ...]

        # Let's refine: The path is a list of extremities.
        # A car moves from path[i] to path[i+1].
        # If path[i] and path[i+1] are on the same road, it's road travel.
        # If path[i] is an exit of an intersection and path[i+1] is an entry to another (or same for U-turn),
        # this implies traversing a road between them.
        # If path[i] is an entry to an intersection and path[i+1] is an exit of the *same* intersection,
        # this is crossing an intersection.

        # Reset and recalculate more clearly:
        total_min_time = 0.0

        # The path from simulator.generate_path is like:
        # [start_road_extremity, intersection_entry_extremity, intersection_exit_extremity, next_road_extremity, ...]
        # Or for a simple road: [road_start_extremity, road_end_extremity]

        for i in range(len(self.full_path_for_min_time_calc) -1 ):
            ext1 = self.full_path_for_min_time_calc[i]
            ext2 = self.full_path_for_min_time_calc[i+1]

            if ext1.road and ext1.get_other_extremity() == ext2: # Traversing a road
                road_length = ext1.road.length
                if self.max_speed > 0:
                    total_min_time += road_length / self.max_speed
                # print(f"Car path: Road segment {ext1.road} length {road_length}, time {road_length / self.max_speed if self.max_speed > 0 else float('inf')}")

            elif ext1.intersection and ext2.intersection and ext1.intersection == ext2.intersection: # Crossing an intersection
                # This condition implies ext1 is an entry point and ext2 is an exit point of the same intersection.
                # The path structure from generate_path might not directly give this for complex intersections.
                # generate_path typically gives [..., road_exit_extremity, intersection_entry_extremity, intersection_exit_extremity, next_road_entry_extremity, ...]
                # So, if ext1.intersection is set, it's an extremity *of* an intersection.
                # If ext1 is an entry and ext2 is an exit of the *same* intersection.
                # This means the car is currently "assigned" to ext1 (e.g. an entry point of roundabout)
                # and its next target ext2 is an exit point of the *same* roundabout.
                # This is the time spent *inside* the intersection.
                total_min_time += ext1.intersection.get_min_crossing_time()
                # print(f"Car path: Intersection segment {ext1.intersection}, time {ext1.intersection.get_min_crossing_time()}")


        # The path structure from `generate_path` alternates between road extremities and intersection extremities.
        # Example: [RoadExtremityA (on Road1), IntersectionExtremityB (entry to Intersection1), IntersectionExtremityC (exit from Intersection1), RoadExtremityD (on Road2)]
        # Segment 1: RoadExtremityA -> IntersectionExtremityB (This is travel ON Road1)
        # Segment 2: IntersectionExtremityB -> IntersectionExtremityC (This is travel INSIDE Intersection1)
        # Segment 3: IntersectionExtremityC -> RoadExtremityD (This is travel ON Road2)

        # Recalculating with the path structure in mind:
        total_min_time = 0.0
        # print(f"Car {id(self)} Path for min_time_calc: {[id(p) for p in self.full_path_for_min_time_calc]}")

        temp_path = list(self.full_path_for_min_time_calc)

        current_node = temp_path.pop(0)
        while temp_path:
            next_node = temp_path.pop(0)

            # Check if moving along a road
            # This happens if current_node is an extremity of a road, and next_node is the *other* extremity of the *same* road.
            if current_node.road and current_node.get_other_extremity() == next_node:
                road_length = current_node.road.length
                if self.max_speed > 0:
                    total_min_time += road_length / self.max_speed
                # print(f"  MinTime: Road {current_node.road.length / self.max_speed if self.max_speed > 0 else 0}")

            # Check if moving through an intersection
            # This happens if current_node is an entry extremity of an intersection,
            # and next_node is an exit extremity of the *same* intersection.
            elif current_node.intersection and next_node.intersection and current_node.intersection == next_node.intersection:
                # This implies generate_path includes both entry and exit extremities for an intersection crossing.
                total_min_time += current_node.intersection.get_min_crossing_time()
                # print(f"  MinTime: Intersection {current_node.intersection.get_min_crossing_time()}")

            current_node = next_node

        return total_min_time / (1000.0/60.0) # Convert from simulation ticks to seconds (assuming speed is in units/tick)
        # Assuming max_speed is in units per simulation step (frame).
        # If max_speed is units per second, then no division by time_factor needed here.
        # Let's assume speed units are consistent with distance units / dt.
        # The problem is that max_speed is units/tick, but time.time() is in seconds.
        # We need to be consistent. If travel times are calculated in ticks, then actual time also in ticks.
        # Car.speed is units/tick. Car.pos updated by speed * time_factor. time_factor is dt / (1000/60).
        # So speed is effectively units / (dt_normalized_to_60fps_step).
        # min_travel_time should be in seconds if actual_travel_time is in seconds.

        # If self.max_speed is pixels/tick (where a tick is 1/60th of a second conceptually)
        # And road_length is in pixels.
        # Then road_length / self.max_speed gives time in ticks.
        # To convert ticks to seconds: time_in_ticks * ( (1000/60) / 1000 ) = time_in_ticks / 60
        # The current time.time() is in seconds.
        # So, min_travel_time should also be in seconds.
        # total_min_time calculated above is in "ticks" if max_speed is units/tick.
        # Let's assume max_speed is in "units per nominal frame (1/60s)".
        # So, total_min_time is in "nominal frames". To convert to seconds: total_min_time * (1/60).
        # This assumes the simulation runs perfectly at 60 FPS for this calculation.
        # Or, if max_speed is intended as units / second already, then no conversion.
        # Given Car.move uses dt, and speed is updated, speed is units/frame.
        # Let's assume max_speed is units / (scaled dt).
        # The most robust way: define max_speed in world units per second.
        # Then convert it to units per tick for simulation movement.
        # For min_travel_time: time = distance / (max_speed_units_per_second)

        # Let's redefine max_speed in Car.py to be in world units per second.
        # And then in Car.move, scale it by dt for actual movement per frame.
        # For now, assuming self.max_speed is units / (nominal tick of 16.66ms)
        # So total_min_time is in ticks. Convert to seconds:
        return total_min_time * (16.666 / 1000.0) # ticks * seconds/tick
