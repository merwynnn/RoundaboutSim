import pygame
from pygame import Vector2 as Vec2
from Constants import *
import math

class Car:
    
    def __init__(self, path, car_image=None):
        from Simulator import Simulator
        self.simulator = Simulator.get_instance()

        self.pos, self.dir = path[0].get_start_car_pos_dir()

        self.speed = 0
        self.max_speed = 3.0 # Vitesse maximale autorisée
        self.acceleration = 0

        self.engine_force = 0.05  
        self.steering_speed = 5

        # Extremities
        self.last_extremity = path[0]

        # Path
        self.path = path
        
        # Targets
        self.status = "EXITING"
        self.reached_last_intersection_target = False
        self.current_target_extremity = None
        self.current_target_position = None
        self.current_target_extremity = self.path[0]
        self.current_target_position = self.get_next_target_position()
        self.current_target_index = 0

        # Image
        if car_image is not None:
            self.car_image = car_image.copy()
        else:
            self.car_image = pygame.Surface((40, 40))
            self.car_image.fill((255, 0, 0))
        self.car_width = self.car_image.get_width() if self.car_image else 40
        self.car_height = self.car_image.get_height() if self.car_image else 40

        self.detection_range = self.car_height * 2.5
        self.detection_angle_threshold = 4.0 # Angle (en degrés) pour considérer "devant" (+/- cet angle)

        
        


    def check_front(self):
        closest_car_distance = math.inf # Initialise avec l'infini pour trouver le minimum

        for other_car in self.simulator.cars:
            if other_car is self:
                continue

            vector_to_other = other_car.pos - self.pos
            distance = vector_to_other.length()
            if 0 < distance < self.detection_range:
                if vector_to_other.length_squared() > 1e-6: 
                    try:
                        angle = self.dir.angle_to(vector_to_other)

                        if abs(angle) < self.detection_angle_threshold:
                            # La voiture est devant et dans la portée
                            if distance < closest_car_distance:
                                closest_car_distance = distance
                    except ValueError:
                         print(f"Warning: ValueError pendant le calcul d'angle pour la voiture à {self.pos}")
                         continue # Passer à la voiture suivante


        # Si la distance la plus proche est toujours l'infini, aucune voiture n'a été trouvée devant
        if closest_car_distance == math.inf:
            return -1.0
        else:
            # Retourner la distance trouvée
            return closest_car_distance

    def move(self):
        if self.current_target_position:
            target_vector = self.current_target_position - self.pos
            distance = target_vector.length()
            if distance < self.speed*20 and distance >= self.speed*5 and self.status == "APPROACHING": # Getting close to intersection
                self.acceleration = -self.engine_force*2
                
            elif distance > self.speed*5: # target position not reached

                target_dir = target_vector.normalize()
                # Tourner progressivement vers la cible
                angle = self.dir.angle_to(target_dir)
                angle = (angle + 180) % 360 - 180
                if abs(angle) > self.steering_speed:
                    self.dir = self.dir.rotate(self.steering_speed * math.copysign(1, angle))
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

            distance_to_obstacle = self.check_front()

            if distance_to_obstacle != -1.0:
                self.acceleration = -pygame.math.lerp(0, self.engine_force, self.speed/self.max_speed)

                if distance_to_obstacle < self.car_height*2:  # if too close, brake
                    self.acceleration -= pygame.math.lerp(0, self.engine_force*10, self.speed/self.max_speed)
            else:
                self.acceleration = self.engine_force
        else:
            self.acceleration = -self.engine_force*2
            self.current_target_position = self.get_next_target_position()
            

        # --- Mise à jour de la position ---
        self.speed += self.acceleration
        self.speed = max(0, min(self.speed, self.max_speed))

        speed = self.speed
        if self.speed < 0.2:
            speed = 0
            
        self.pos += self.dir * speed

    def get_next_target_extremity(self):
        self.path.pop(0)        
            
        return self.path[0]

    def get_next_target_position(self):
        if self.status=="INTERSECTION" and self.reached_last_intersection_target:
            self.reached_last_intersection_target = False
            self.status = "EXITING"
            return self.current_target_extremity.get_start_car_pos_dir(delta=-5)[0]
        
        elif self.status=="INTERSECTION":  # is in intersection

            next_target_pos, is_last_pos = self.last_extremity.intersection.get_next_target_position(self.last_extremity, self.current_target_extremity, self.current_target_index, car=self)
            
            self.current_target_index += 1
            if is_last_pos:
                self.reached_last_intersection_target = True
                self.current_target_index = 0
                
            return next_target_pos
        elif self.status == "APPROACHING":
            if self.current_target_extremity.intersection.can_car_enter(self.last_extremity): 
                self.last_extremity = self.current_target_extremity
                self.current_target_extremity = self.get_next_target_extremity()
                next_target_pos, is_last_pos = self.last_extremity.intersection.get_next_target_position(self.last_extremity, self.current_target_extremity, self.current_target_index, car=self)
                self.current_target_index += 1
                self.status = "INTERSECTION"
                return next_target_pos
            else:
                return None

        elif self.status=="EXITING":
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            self.status="APPROACHING"
            return self.last_extremity.get_end_car_pos_dir()[0]
        
        else:   # Entering intersection
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            return self.get_next_target_position()

        

    def draw(self, win):
        # Rotate
        angle_degrees = self.dir.angle_to(Vec2(1, 0))
        rotated_car_image = pygame.transform.rotate(self.car_image, angle_degrees-90) # Négatif car pygame tourne anti-horaire

        # Obtenir le rectangle après rotation pour le centrer correctement
        new_rect = rotated_car_image.get_rect(center = (int(self.pos.x), int(self.pos.y)))

        # Dessiner l'image rotatée
        win.blit(rotated_car_image, new_rect.topleft)

        if self.simulator.debug:
            if self.current_target_position:
                pygame.draw.circle(win, (0, 0, 255), self.current_target_position, 5)
            if True: # Si vous avez un mode debug
                start_point = self.pos
                # Calcul des points limites du cône de détection
                p1 = self.pos + Vec2(self.detection_range, 0).rotate(-self.detection_angle_threshold - angle_degrees)
                p2 = self.pos + Vec2(self.detection_range, 0).rotate(self.detection_angle_threshold - angle_degrees)
                # Dessiner les lignes du cône
                pygame.draw.line(win, (0, 255, 255, 100), start_point, p1, 1)
                pygame.draw.line(win, (0, 255, 255, 100), start_point, p2, 1)
                pygame.draw.arc(win, (0, 255, 255, 100), pygame.Rect(self.pos.x - self.detection_range, self.pos.y - self.detection_range, 2*self.detection_range, 2*self.detection_range), math.radians(angle_degrees - self.detection_angle_threshold), math.radians(angle_degrees + self.detection_angle_threshold), 1)
