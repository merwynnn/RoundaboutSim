
import pygame
from pygame import Vector2 as Vec2
from Constants import *

class RoadExtremity:
    def __init__(self,pos=(0, 0),intersection = None, road = None, spawn_cars=False, spawn_cars_timer=5):

    
        self.pos = Vec2(pos)
        self.intersection = intersection
        self.road = road
        # self.simulator = None # Removed as using singleton
        self.spawn_cars = spawn_cars
        self.spawn_cars_timer = spawn_cars_timer
        self.timer = 0

            
    def update(self):
        if self.spawn_cars:
            self.timer += 1
            if self.timer % self.spawn_cars_timer == 0:
                # if self.simulator: # Removed as using singleton
                from Simulator import Simulator
                Simulator.get_instance().spawn_car(self)
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

        if self.simulator.debug:
            pygame.draw.circle(win, (255, 0, 0), self.lanes_start_end_position[0][0], 5)
            pygame.draw.circle(win, (255, 0, 0), self.lanes_start_end_position[0][1], 5)
            pygame.draw.circle(win, (0, 255, 0), self.lanes_start_end_position[1][0], 5)
            pygame.draw.circle(win, (0, 255, 0), self.lanes_start_end_position[1][1], 5)

    
    def update(self):
        pass