import os
import pygame
from pygame import Vector2 as Vec2
import sys
import heapq
import math
import random # For random spawning

# Constants
WIDTH, HEIGHT = 800, 800
BACKGROUND_COLOR = (0, 150, 0)
ROAD_COLOR = (113, 112, 113)
LANE_WIDTH = 25
STRIPE_WIDTH = 3

DEBUG = False

# Pygame setup
pygame.init()

class RoadExtremity:
    def __init__(self, simulator,pos=(0, 0),intersection = None, road = None, spawn_cars=False, spawn_cars_timer=5):
        self.pos = Vec2(pos)
        self.intersection = intersection
        self.road = road
        self.simulator = True
        self.spawn_cars = spawn_cars
        self.spawn_cars_timer = spawn_cars_timer
        self.simulator = simulator
        self.timer = 0

            
    def update(self):
        if self.spawn_cars:
            self.timer += 1
            if self.timer % self.spawn_cars_timer == 0:
                self.simulator.spawn_car(self)
                self.timer = 0
                

    def get_start_car_pos_dir(self):
        if self.road.start_extremity is self:
            return self.road.lanes_start_end_position[0][0].copy(), self.road.dir
        else:
            return self.road.lanes_start_end_position[1][0].copy(), -self.road.dir
        
    def get_end_car_pos_dir(self):
        if self.road.start_extremity is self:
            return self.road.lanes_start_end_position[0][1].copy(), self.road.dir
        else:
            return self.road.lanes_start_end_position[1][1].copy(), -self.road.dir
        
    def get_other_extremity(self):
        if self.road.start_extremity is self:
            return self.road.end_extremity
            
        else:
            return self.road.start_extremity
            
        
    def is_on_same_road(self, other_extremity):
        return self.road == other_extremity.road
    

                
            

class Road:
    
    def __init__(self, start_extremity, end_extremity):

        self.start_extremity = start_extremity
        self.end_extremity = end_extremity

        self.length = (self.end_extremity.pos - self.start_extremity.pos).length()
        
        self.nb_lanes = 2

        self.dir = self.end_extremity.pos - self.start_extremity.pos
        self.dir = self.dir.normalize()
        self.dir_orth = Vec2(-self.dir.y, self.dir.x)

        pos_delta = self.dir_orth * LANE_WIDTH // 2
        x_delta = Vec2(5, 0)

        self.lanes_start_end_position = [(self.start_extremity.pos+pos_delta+x_delta, self.end_extremity.pos+pos_delta-x_delta), (self.end_extremity.pos-pos_delta-x_delta, self.start_extremity.pos-pos_delta+x_delta )]

    def draw(self, win):
        
        delta = self.dir_orth * (self.nb_lanes//2 + (self.nb_lanes%2)/2) * LANE_WIDTH
        
        # main polygon
        p1 = self.start_extremity.pos - delta
        p2 = self.start_extremity.pos + delta
        p3 = self.end_extremity.pos + delta
        p4 = self.end_extremity.pos - delta

        pygame.draw.polygon(win, ROAD_COLOR, [p1, p2, p3, p4])

        # left right stripe
        p1 = self.start_extremity.pos - delta + self.dir_orth * STRIPE_WIDTH//2
        p2 = self.start_extremity.pos - delta + self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH)
        p3 = self.end_extremity.pos - delta + self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH)
        p4 = self.end_extremity.pos - delta + self.dir_orth * STRIPE_WIDTH//2

        pygame.draw.polygon(win, (255, 255, 255), [p1, p2, p3, p4])

        p1 = self.start_extremity.pos + delta - self.dir_orth * STRIPE_WIDTH//2
        p2 = self.start_extremity.pos + delta - self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH)
        p3 = self.end_extremity.pos + delta - self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH)
        p4 = self.end_extremity.pos + delta - self.dir_orth * STRIPE_WIDTH//2

        pygame.draw.polygon(win, (255, 255, 255), [p1, p2, p3, p4])

        # lane separation stripes
        for i in range(1, self.nb_lanes):
            stripe_delta = self.dir_orth * i * LANE_WIDTH
            p1 = self.start_extremity.pos + delta - stripe_delta
            p2 = self.end_extremity.pos + delta - stripe_delta

            pygame.draw.line(win, (255, 255, 255), p1, p2, STRIPE_WIDTH)

        if DEBUG:
            pygame.draw.circle(win, (255, 0, 0), self.lanes_start_end_position[0][0], 5)
            pygame.draw.circle(win, (255, 0, 0), self.lanes_start_end_position[0][1], 5)
            pygame.draw.circle(win, (0, 255, 0), self.lanes_start_end_position[1][0], 5)
            pygame.draw.circle(win, (0, 255, 0), self.lanes_start_end_position[1][1], 5)

    
    def update(self):
        pass

        
        

        

class Intersection:
    def __init__(self, simulator, pos):
        self.simulator = simulator
        self.pos = Vec2(pos)

    def get_yield_point(car):
        pass

    def update(self):
        pass


    def draw(self, win):
        pass

    def get_next_target_position(self, start_extremity, current_target_index):
        #return next_target_pos, is_last_pos
        pass
        
class ClassicRoundabout(Intersection):
    def __init__(self, simulator, pos, radius, exits_dir):
        super().__init__(simulator, pos)
        self.radius = radius
        self.center = Vec2(pos)
        self.nb_lanes = 1

        self.exits = []
        for exit_dir in exits_dir:
            self.exits.append(RoadExtremity(self.simulator, (self.center.x + self.radius * exit_dir.x, self.center.y + self.radius * exit_dir.y), self))

        self.targets = self.get_evenly_spaced_points(14)[::-1]
    

    def get_yield_point(self, car):
        # Calculate the yield point based on the car's direction
        car_dir = car.dir.normalize()
        yield_point = self.center - car_dir * (self.radius + 20)
        return yield_point

    def draw(self, win):
        pygame.draw.circle(win, ROAD_COLOR, self.center, self.radius)
        pygame.draw.circle(win, BACKGROUND_COLOR, self.center, self.radius-self.nb_lanes*LANE_WIDTH)
        pygame.draw.circle(win, (255, 255, 255), self.center, self.radius, STRIPE_WIDTH)
        pygame.draw.circle(win, (255, 255, 255), self.center, self.radius-self.nb_lanes*LANE_WIDTH, STRIPE_WIDTH)

        if DEBUG:
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
        

    def get_next_target_position(self, start_extremity, exit_extremity, current_target_index):
        start_target_index = (self.closest_target(start_extremity.get_other_extremity().get_end_car_pos_dir()[0])+1)%len(self.targets)
        exit_target_index = (self.closest_target(exit_extremity.get_start_car_pos_dir()[0])-1)%len(self.targets)
        target_index = (start_target_index+current_target_index)%len(self.targets)
        return self.targets[target_index], exit_target_index == target_index
        

class Car:
    
    def __init__(self, simulator, path):
        self.simulator = simulator

        self.pos,self.dir = path[0].get_start_car_pos_dir()

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
        self.car_image = pygame.image.load("Assets/"+random.choice(os.listdir("Assets")))
        self.car_image = pygame.transform.scale(self.car_image, (40, 40))
        self.car_width = self.car_image.get_width() if self.car_image else 40
        self.car_height = self.car_image.get_height() if self.car_image else 40

        self.detection_range = self.car_height * 2.5
        self.detection_angle_threshold = 4.0 # Angle (en degrés) pour considérer "devant" (+/- cet angle)

        
        


    def check_front(self):
        """
        Vérifie la présence de la voiture la plus proche directement devant,
        dans une portée et un angle définis (Méthode 1).

        Retourne:
            float: La distance à la voiture la plus proche devant, ou -1.0 si aucune voiture n'est détectée.
        """
        closest_car_distance = math.inf # Initialise avec l'infini pour trouver le minimum

        for other_car in self.simulator.cars:
            if other_car is self:
                continue

            vector_to_other = other_car.pos - self.pos
            distance = vector_to_other.length()

            # Étape 1: Vérifier si la voiture est dans la portée de détection
            # (et pas exactement à la même position pour éviter les divisions par zéro plus tard)
            if 0 < distance < self.detection_range:

                # Étape 2: Vérifier si la voiture est dans le cône de vision frontal
                # Assurer que le vecteur n'est pas nul avant de calculer l'angle
                if vector_to_other.length_squared() > 1e-6: # Utilise length_squared pour efficacité et éviter imprécision flottante
                    try:
                        angle = self.dir.angle_to(vector_to_other)

                        # Vérifier si l'angle absolu est dans le seuil défini
                        if abs(angle) < self.detection_angle_threshold:
                            # La voiture est devant et dans la portée
                            # Garder la distance de la voiture la plus proche trouvée jusqu'à présent
                            if distance < closest_car_distance:
                                closest_car_distance = distance
                    except ValueError:
                         # Peut arriver rarement si self.dir devient un vecteur nul malgré les précautions
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
            if distance > self.speed*5: # Éviter la division par zéro ou comportement erratique si trop proche
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
            return self.current_target_extremity.get_start_car_pos_dir()[0]
        
        elif self.status=="INTERSECTION":  # is in intersection

            next_target_pos, is_last_pos = self.last_extremity.intersection.get_next_target_position(self.last_extremity, self.current_target_extremity, self.current_target_index)
            
            self.current_target_index += 1
            if is_last_pos:
                self.reached_last_intersection_target = True
                self.current_target_index = 0
                
            return next_target_pos
        elif self.status == "APPROACHING":
            self.last_extremity = self.current_target_extremity
            self.current_target_extremity = self.get_next_target_extremity()
            next_target_pos, is_last_pos = self.last_extremity.intersection.get_next_target_position(self.last_extremity, self.current_target_extremity, self.current_target_index)
            self.status = "INTERSECTION"
            return next_target_pos

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

        if DEBUG:
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

            
class Simulator:
    INTERSECTION_COST = 10.0 # Define a cost for traversing an intersection (adjust as needed)

    def __init__(self):
        self.win = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Roundabout Simulator")
        self.clock = pygame.time.Clock()

        # --- Your existing setup code ---
        self.intersections = [ClassicRoundabout(self, (400, 400), 100, [Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)])]

        ext1 = RoadExtremity(self, (0, HEIGHT//2), spawn_cars=True, spawn_cars_timer=60)
        # Ensure the intersection exits are used correctly when creating roads
        ext2 = RoadExtremity(self, (WIDTH, HEIGHT//2), spawn_cars=True, spawn_cars_timer=120)

        ext_3 = RoadExtremity(self, (WIDTH//2, 0), spawn_cars=True, spawn_cars_timer=120)
        ext_4 = RoadExtremity(self, (WIDTH//2, HEIGHT), spawn_cars=True, spawn_cars_timer=120)
        

        # Assuming intersection.exits[0] connects to ext1 and intersection.exits[1] connects to ext2
        self.roads = [Road(ext1, self.intersections[0].exits[0]), Road(self.intersections[0].exits[1], ext2), Road(ext_3, self.intersections[0].exits[2]), Road(ext_4, self.intersections[0].exits[3])]

        self.road_extremity_spawners = [ext1, ext2, ext_3, ext_4]

        for road in self.roads:
            road.start_extremity.road = road
            road.end_extremity.road = road

        # Collect all extremities for pathfinding reference later if needed
        self.all_extremities = set()
        for road in self.roads:
             self.all_extremities.add(road.start_extremity)
             self.all_extremities.add(road.end_extremity)
        # --- End of existing setup ---

        self.cars = []


        self.run()

    def run(self):
        global DEBUG
        # --- Your existing run loop ---
        while True:
            dt = self.clock.tick(60) # Delta time in milliseconds
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_d:
                        DEBUG = not DEBUG

            self.win.fill(BACKGROUND_COLOR)

            for road in self.roads:
                road.update()
                road.draw(self.win)


            for intersection in self.intersections:
                intersection.update()
                intersection.draw(self.win)

            # Use a copy of the list for iteration if cars can be removed during the loop
            for car in list(self.cars):
                car.move()
                car.draw(self.win)

            for road_extremity in self.road_extremity_spawners:
                road_extremity.update()

            pygame.display.update()
        # --- End of existing run loop ---


    def generate_path(self, start_extremity, end_extremity):
        """
        Generates the shortest path between two RoadExtremity objects using Dijkstra's algorithm,
        adjusting the path for intersection U-turns.
        # ... (rest of docstring)
        """
        # 1. Gather nodes (no changes)
        all_nodes = set()
        for road in self.roads:
            all_nodes.add(road.start_extremity)
            all_nodes.add(road.end_extremity)
        for intersection in self.intersections:
             for exit_extremity in intersection.exits:
                 all_nodes.add(exit_extremity)


        # 2. Initialize Dijkstra's structures
        distances = {node: math.inf for node in all_nodes}
        previous_nodes = {node: None for node in all_nodes}
        distances[start_extremity] = 0

        # Initialize a counter for tie-breaking in the priority queue
        entry_count = 0
        priority_queue = [(0, entry_count, start_extremity)] # (distance, count, node)
        entry_count += 1

        # 3. Dijkstra's Algorithm
        while priority_queue:
            # Pop the element with the smallest distance, ignore the counter
            current_distance, _, current_node = heapq.heappop(priority_queue)

            # Optimization: If we've found a shorter path already, skip
            if current_distance > distances[current_node]:
                continue

            # If we reached the destination
            if current_node == end_extremity:
                break # Path found

            # --- Explore neighbors ---

            # Neighbor across the current node's road
            if current_node.road:
                other_road_extremity = current_node.get_other_extremity()
                if other_road_extremity:
                    cost = current_node.road.length
                    neighbor = other_road_extremity
                    # Check if neighbor exists in our collected nodes (should always if setup is correct)
                    if neighbor in distances:
                        new_distance = distances[current_node] + cost
                        if new_distance < distances[neighbor]:
                            distances[neighbor] = new_distance
                            previous_nodes[neighbor] = current_node
                            heapq.heappush(priority_queue, (new_distance, entry_count, neighbor))
                            entry_count += 1 # Increment counter

            # Neighbors across the intersection (if the node is part of an intersection)
            if current_node.intersection:
                cost = self.INTERSECTION_COST
                for neighbor in current_node.intersection.exits:
                    # Check if neighbor exists in our collected nodes
                    if neighbor in distances:
                        # Allow pathing to other exits or U-turn via the same exit
                        # Note: Dijkstra logic handles not going back immediately unless optimal
                        new_distance = distances[current_node] + cost
                        if new_distance < distances[neighbor]:
                             distances[neighbor] = new_distance
                             previous_nodes[neighbor] = current_node
                             heapq.heappush(priority_queue, (new_distance, entry_count, neighbor))
                             entry_count += 1 # Increment counter


        # --- 4. Reconstruct path ---
        path_nodes = []
        current = end_extremity
        if previous_nodes.get(current) is None and current != start_extremity: # Use .get for safety
            print(f"Warning: Path not found from start {id(start_extremity)} to end {id(end_extremity)}")
            return []

        while current is not None:
            path_nodes.append(current)
            # Basic cycle detection during reconstruction
            next_node_in_recon = previous_nodes.get(current) # Use .get
            if next_node_in_recon in path_nodes:
                 print(f"Error: Path reconstruction cycle detected near node id={id(current)}")
                 # Avoid infinite loop, return potentially incomplete path or empty
                 return []
            current = next_node_in_recon


        if not path_nodes or path_nodes[-1] != start_extremity:
             print(f"Warning: Path reconstruction failed for start {id(start_extremity)} to end {id(end_extremity)}")
             # Check if path_nodes is not empty before accessing [-1]
             if path_nodes and path_nodes[-1] != start_extremity:
                  print(f"  -> Path ends at {id(path_nodes[-1])} instead of {id(start_extremity)}")
             return []

        # Reverse to get the standard path order: [start, node2, ..., end]
        ordered_nodes = path_nodes[::-1]

        # --- 5. Correct Path for Intersection U-Turns ---
        # If the path is A -> B -> A, where B is an intersection node, change it to A -> B -> B -> A.
        final_path_corrected = []
        if not ordered_nodes:
            return []

        final_path_corrected.append(ordered_nodes[0]) # Add the first node

        i = 0
        while i < len(ordered_nodes) - 1:
            prev_node = ordered_nodes[i]
            curr_node = ordered_nodes[i+1]

            final_path_corrected.append(curr_node) # Always add the current node

            # Check if a U-turn pattern exists: prev -> curr -> next (where next == prev)
            # And ensure the turnaround node 'curr' is actually part of an intersection.
            if i + 2 < len(ordered_nodes):
                next_node = ordered_nodes[i+2]
                if next_node == prev_node and curr_node.intersection is not None:
                    # Found pattern A -> B -> A where B is in an intersection.
                    # Insert B again right after the first B.
                    final_path_corrected.append(curr_node) # Add the duplicate
                    # Skip the next node in the original list as we've processed the A->B->A triplet
                    i += 1

            i += 1 # Move to the next node


        # Debugging print (optional)
        # print(f"Original path nodes: {[id(n) for n in ordered_nodes]}")
        # print(f"Corrected path nodes: {[id(n) for n in final_path_corrected]}")

        return final_path_corrected


    def car_reached_destination(self, car):
        # Check if the car object exists in the list before attempting removal
        if car in self.cars:
            self.cars.remove(car)
        # Optional: Clean up the car object if necessary (Python's garbage collector usually handles this)
        # del car
        # print("Car reached destination and was removed.") # For debugging


    def spawn_car(self, start_extremity):
        # Choose a random end extremity that is NOT the start extremity or on the same road
        possible_ends = [
            ext for ext in self.road_extremity_spawners if ext != start_extremity
        ]

        if not possible_ends:
             # Fallback or specific logic if no suitable random end found
             # For the simple case, let's use the predefined ext2 if available
             end_extremity = self.roads[1].end_extremity if len(self.roads) > 1 else None
             # print("Warning: No random end extremity found, using default.")
             if not end_extremity or end_extremity == start_extremity:
                  print("Error: Cannot determine a valid end extremity for spawning car.")
                  return # Don't spawn if no valid end
        else:
             end_extremity = random.choice(possible_ends)


        # Generate the path for the new car
        path = self.generate_path(start_extremity, end_extremity)

        if path: # Only spawn if a path exists
            new_car = Car(self, path) # Pass the end_extremity
            # It seems the Car class already calls generate_path internally,
            # ensure it uses the passed end_extremity or remove the internal call.
            # Let's assume Car uses the provided final_target_extremity.
            self.cars.append(new_car)
            # print(f"Spawned car from {start_extremity.pos} to {end_extremity.pos}") # For debugging
        else:
            print(f"Warning: Could not generate path for new car from {start_extremity.pos} to {end_extremity.pos}. Car not spawned.")
        


simulator = Simulator()
