#   Author : Swadhin Agrawal
#   Couzin's model control dynamics

import numpy as np

class couzinsModel:
    def __init__(self,number_of_drones,velocity,swarm_initial,Rr,Ro,Ra,w,vision):
        self.zor = []
        self.zoo = []
        self.zoa = []
        self.swarm = swarm_initial
        self.velocity = velocity
        self.number_of_drones = number_of_drones
        self.Rr = Rr
        self.Ro = Ro
        self.Ra = Ra
        self.w = w
        self.swarm_velocities = np.array([[self.velocity,0.0] for i in range(self.number_of_drones)])
        self.vision_limit = vision

    def blind_spot_check(self,drone1,drone2):
        pointing_vec = np.subtract(drone2[:2],drone1[:2])
        vision_line = self.angle_wrap(np.arctan2(pointing_vec[1],pointing_vec[0]))
        neighbour_region =  self.angle_wrap_0_2pi(drone1[2] - vision_line)
        return abs(neighbour_region)

    def neighbours_in_zones(self,current_drone):
        for i in range(self.number_of_drones):
            dij = np.linalg.norm(current_drone[:2]-self.swarm[i,:2])
            blind = self.blind_spot_check(self.swarm[i],current_drone)
            if dij!=0.0 and blind<self.vision_limit:
                if dij< self.Rr:
                    self.zor.append([i,dij])
                elif dij>= self.Rr and dij<= self.Ro:
                    self.zoo.append([i,dij])
                elif dij> self.Ro and dij<= self.Ra:
                    self.zoa.append([i,dij])

    def dr_repulsion(self,current_drone):  #   1st Priority
        move_away = np.array([0.0,0.0])
        for i in self.zor:
            move_away += np.subtract(self.swarm[int(i[0]),:2],current_drone[:2])/np.linalg.norm(np.subtract(self.swarm[int(i[0]),:2],current_drone[:2]))
        return -1*move_away

    def do_orientation(self):  #   2nd Priority
        orient = np.array([0.0,0.0])
        for i in self.zoo:
            dronej = np.array([np.cos(self.swarm[i[0],2]),np.sin(self.swarm[i[0],2])])
            orient += dronej/np.linalg.norm(dronej)
        return orient

    def da_attract(self,current_drone):      #   3rd Priority
        attract = np.array([0.0,0.0])
        for i in self.zoa:
            attract += np.subtract(self.swarm[i[0],:2],current_drone[:2])/np.linalg.norm(np.subtract(current_drone[:2],self.swarm[i[0],:2]))
        return attract

    def net_di(self,current_drone):
        if len(self.zor)!=0:
            di = self.dr_repulsion(current_drone)
        elif len(self.zor)==0:
            if len(self.zoo)!=0 and len(self.zoa)==0:
                di = self.do_orientation()
            elif len(self.zoa)!=0 and len(self.zoo)==0:
                di = self.da_attract(current_drone)
            elif len(self.zoa)!=0 and len(self.zoo)!=0:
                di = 0.5*(self.do_orientation() + self.da_attract(current_drone))
            else:
                di = [np.cos(current_drone[2]),np.sin(current_drone[2])]
        return di

    def saturate(self,u,lim):
        if abs(u) > lim:
            u = u*lim/abs(u)
        return u
    
    def angle_wrap(self,angle):
        if angle<0:    
            angle += 2*np.pi
        return angle

    def angle_wrap_0_2pi(self,angle):
        while angle<0:
            angle += 2*np.pi
        while angle>2*np.pi:
            angle -= 2*np.pi
        return angle