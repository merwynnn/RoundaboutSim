
import pygame
from pygame import Vector2 as Vec2
from Constants import *



class RoadExtremity:
    next_id = 0
    def __init__(self,pos=(0, 0),intersection = None, road = None, spawn_cars=False, spawn_cars_timer=5):

    
        self.id = None
        if spawn_cars:
            self.id = RoadExtremity.next_id
            RoadExtremity.next_id += 1

    
        self.pos = Vec2(pos)
        self.intersection = intersection
        self.road = road
        # self.simulator = None # Removed as using singleton
        self.spawn_cars = spawn_cars
        self.spawn_cars_timer = spawn_cars_timer
        self.timer = 0

     
        self.last_spawned_car = None

            
    def update(self, dt):
        if self.spawn_cars:
            self.timer += 0.05*dt
            if self.timer >= self.spawn_cars_timer:
                # if self.simulator: # Removed as using singleton
                from Simulator import Simulator
                if self.last_spawned_car:
                    if (self.last_spawned_car.pos - self.pos).length() > 50:
                        self.last_spawned_car = Simulator.get_instance().spawn_car(self)
                else:
                    self.last_spawned_car = Simulator.get_instance().spawn_car(self)
                self.timer = 0
                

    def get_start_car_pos_dir(self, delta=0):
        if self.road.start_extremity is self:
            return self.road.lanes_start_end_position[0][0].copy()+self.road.dir*delta, self.road.dir
        else:
            return self.road.lanes_start_end_position[1][0].copy()-self.road.dir*delta, -self.road.dir
        
    def get_end_car_pos_dir(self, delta=0):
        if self.road.start_extremity is self:
            return self.road.lanes_start_end_position[0][1].copy()+self.road.dir*delta, self.road.dir
        else:
            return self.road.lanes_start_end_position[1][1].copy()-self.road.dir*delta, -self.road.dir
        
    def get_other_extremity(self):
        if self.road.start_extremity is self:
            return self.road.end_extremity
            
        else:
            return self.road.start_extremity
            
        
    def is_on_same_road(self, other_extremity):
        return self.road == other_extremity.road
    

                
            

class Road:
    
    def __init__(self, start_extremity, end_extremity):

        from Simulator import Simulator
        self.simulator = Simulator.get_instance()

        self.start_extremity = start_extremity
        self.end_extremity = end_extremity

        self.length = (self.end_extremity.pos - self.start_extremity.pos).length()
        
        self.nb_lanes = 2

        self.dir = self.end_extremity.pos - self.start_extremity.pos
        self.dir = self.dir.normalize()
        self.dir_orth = Vec2(-self.dir.y, self.dir.x)

        pos_delta = self.dir_orth * LANE_WIDTH // 2
        x_delta = Vec2(5, 0)

        self.lanes_start_end_position = [(self.start_extremity.pos+pos_delta, self.end_extremity.pos+pos_delta), (self.end_extremity.pos-pos_delta, self.start_extremity.pos-pos_delta )]

    def draw(self, win):
        
        delta = self.dir_orth * (self.nb_lanes//2 + (self.nb_lanes%2)/2) * LANE_WIDTH
        
        # main polygon
        p1_transformed = self.simulator.camera.apply(self.start_extremity.pos - delta)
        p2_transformed = self.simulator.camera.apply(self.start_extremity.pos + delta)
        p3_transformed = self.simulator.camera.apply(self.end_extremity.pos + delta)
        p4_transformed = self.simulator.camera.apply(self.end_extremity.pos - delta)

        pygame.draw.polygon(win, ROAD_COLOR, [p1_transformed, p2_transformed, p3_transformed, p4_transformed])

        # left right stripe
        p1_ls_transformed = self.simulator.camera.apply(self.start_extremity.pos - delta + self.dir_orth * STRIPE_WIDTH//2)
        p2_ls_transformed = self.simulator.camera.apply(self.start_extremity.pos - delta + self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH))
        p3_ls_transformed = self.simulator.camera.apply(self.end_extremity.pos - delta + self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH))
        p4_ls_transformed = self.simulator.camera.apply(self.end_extremity.pos - delta + self.dir_orth * STRIPE_WIDTH//2)

        pygame.draw.polygon(win, (255, 255, 255), [p1_ls_transformed, p2_ls_transformed, p3_ls_transformed, p4_ls_transformed])

        p1_rs_transformed = self.simulator.camera.apply(self.start_extremity.pos + delta - self.dir_orth * STRIPE_WIDTH//2)
        p2_rs_transformed = self.simulator.camera.apply(self.start_extremity.pos + delta - self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH))
        p3_rs_transformed = self.simulator.camera.apply(self.end_extremity.pos + delta - self.dir_orth * (STRIPE_WIDTH//2 + STRIPE_WIDTH))
        p4_rs_transformed = self.simulator.camera.apply(self.end_extremity.pos + delta - self.dir_orth * STRIPE_WIDTH//2)

        pygame.draw.polygon(win, (255, 255, 255), [p1_rs_transformed, p2_rs_transformed, p3_rs_transformed, p4_rs_transformed])

        # lane separation stripes
        for i in range(1, self.nb_lanes):
            stripe_delta_world = self.dir_orth * i * LANE_WIDTH
            p1_stripe_transformed = self.simulator.camera.apply(self.start_extremity.pos + delta - stripe_delta_world)
            p2_stripe_transformed = self.simulator.camera.apply(self.end_extremity.pos + delta - stripe_delta_world)

            pygame.draw.line(win, (255, 255, 255), p1_stripe_transformed, p2_stripe_transformed, STRIPE_WIDTH) # STRIPE_WIDTH will be scaled later

        if self.simulator.debug:

            # Show road extremities index
            if self.start_extremity.spawn_cars:
                text = font_medium.render(str(self.start_extremity.id), True, (0, 0, 0))
                text_rect = text.get_rect()
                text_rect.center = self.simulator.camera.apply(self.start_extremity.pos)
                win.blit(text, text_rect)

            if self.end_extremity.spawn_cars:
                text = font_medium.render(str(self.end_extremity.id), True, (0, 0, 0))
                text_rect = text.get_rect()
                text_rect.center = self.simulator.camera.apply(self.end_extremity.pos)
                win.blit(text, text_rect)

    
    def update(self, dt):
        pass
