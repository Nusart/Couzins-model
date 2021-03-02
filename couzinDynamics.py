#   Author : Swadhin Agrawal
#   Couzin's model control dynamics

import numpy as np

class couzinsModel:
    def __init__(self,number_of_drones,velocity,swarm_initial,Rr,Ro,Ra,vision):
        self.zor = []
        self.zoo = []
        self.zoa = []
        self.swarm = swarm_initial
        self.velocity = velocity
        self.number_of_drones = number_of_drones
        self.Rr = Rr
        self.Ro = Ro
        self.Ra = Ra
        self.vision_limit = vision

    def blind_spot_check(self,drone1,drone2):
        pointing_vec = np.subtract(drone1[:2],drone2[:2])
        pointing_vec = pointing_vec/np.linalg.norm(pointing_vec)
        angular_position = np.arccos(np.dot(pointing_vec,[np.cos(drone2[2]),np.sin(drone2[2])]))
        return angular_position

    def neighbours_in_zones(self,current_drone):
        for i in range(self.number_of_drones):
            dij = np.linalg.norm(current_drone[:2]-self.swarm[i,:2])
            if np.linalg.norm(current_drone - self.swarm[i])!=0 :
                blind = self.blind_spot_check(self.swarm[i],current_drone)
                if dij<= self.Rr:
                    self.zor.append([i,dij])
                elif dij> self.Rr and dij< self.Ro and blind<self.vision_limit:
                    self.zoo.append([i,dij])
                elif dij>= self.Ro and dij<= self.Ra and blind<self.vision_limit:
                    self.zoa.append([i,dij])
                else:
                    pass
                

    def dr_repulsion(self,current_drone):  #   1st Priority
        move_away = np.array([0.0,0.0])
        for i in self.zor:
            move_away += np.subtract(self.swarm[int(i[0]),:2],current_drone[:2])/np.linalg.norm(np.subtract(self.swarm[int(i[0]),:2],current_drone[:2]))
        return -1*move_away

    def do_orientation(self):  #   2nd Priority
        orient = np.array([0.0,0.0])
        for i in self.zoo:
            orient += np.array([np.cos(self.swarm[i[0],2]),np.sin(self.swarm[i[0],2])])
        return orient

    def da_attract(self,current_drone):      #   3rd Priority
        attract = np.array([0.0,0.0])
        for i in self.zoa:
            attract += np.subtract(self.swarm[i[0],:2],current_drone[:2])/np.linalg.norm(np.subtract(current_drone[:2],self.swarm[i[0],:2]))
        return attract

    def net_di(self,current_drone):
        if len(self.zor)!=0:
            di = self.dr_repulsion(current_drone)
        else:
            if len(self.zoo)!=0 :#and len(self.zoa)==0:
                di = self.do_orientation()
            elif len(self.zoa)!=0:# and len(self.zoo)==0:
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
        while angle>=2*np.pi:
            angle -= 2*np.pi
        return angle
    
    def angle_w_n_pi_p_pi(self,angle):
        while angle>np.pi:    
            angle -= 2*np.pi
        while angle<-np.pi:    
            angle += 2*np.pi
        return angle


if __name__=="__main__":
    drones_positions = np.array([[1,1,1.5],[1,2,1.8],[20,20,2]])    #	Theta (in Rad) of swarm members
    a = couzinsModel(3,1,drones_positions,1,1,10,150*np.pi/180)
    a.neighbours_in_zones(drones_positions[0])
    print(a.zor)
    print(a.zoo)
    print(a.zoa)
    b = a.net_di(drones_positions[0])