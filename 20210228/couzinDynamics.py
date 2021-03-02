#   Author : Swadhin Agrawal
#   Couzin's model control dynamics

import numpy as np

class couzinsModel:
    def __init__(self,Rr,Ro,Ra,vision):
        self.zor = []
        self.zoo = []
        self.zoa = []
        self.Rr = Rr
        self.Ro = Ro
        self.Ra = Ra
        self.visibility_angle = vision

    def blind_spot_check(self,other,this):
        pointing_vec = np.subtract(other.pose,this.pose)
        pointing_vec = pointing_vec/np.linalg.norm(pointing_vec)
        angular_position = np.arccos(np.dot(pointing_vec,this.velocity/np.linalg.norm(this.velocity)))
        return angular_position

    def neighbours_in_zones(self,this,flock):
        for i in range(len(flock)):
            dij = np.linalg.norm(this.pose-flock[i].pose)
            angular_position = abs(self.blind_spot_check(flock[i],this))
            if dij<= self.Rr and angular_position<self.visibility_angle:
                self.zor.append([i,dij])
            elif dij> self.Rr and dij< self.Ro and angular_position<self.visibility_angle:
                self.zoo.append([i,dij])
            elif dij>= self.Ro and dij<= self.Ra and angular_position<self.visibility_angle:
                self.zoa.append([i,dij])
            else:
                pass

    def repulsion(self,this,flock):  #   1st Priority
        repel = np.array([0.0,0.0])
        for i in self.zor:
            repel += np.subtract(flock[i[0]].pose,this.pose)/np.linalg.norm(np.subtract(flock[i[0]].pose,this.pose))
        return -1*repel/len(self.zor)
    
    def align(self,flock):  #   2nd Priority
        orient = np.array([0.0,0.0])
        for i in self.zoo:
            orient += flock[i[0]].velocity/np.linalg.norm(flock[i[0]].velocity)
        return orient/len(self.zoo)
    
    def cohesion(self,this,flock):      #   3rd Priority
        attract = np.array([0.0,0.0])
        for i in self.zoa:
            attract += np.subtract(flock[i[0]].pose,this.pose)/np.linalg.norm(np.subtract(flock[i[0]].pose,this.pose))
        return attract/len(self.zoa)

    def net_di(self,this,flock):
        if len(self.zor)!=0:
            di = self.repulsion(this,flock)
        else:
            if len(self.zoo)!=0 and len(self.zoa)==0:
                di = self.align(flock)
            elif len(self.zoa)!=0 and len(self.zoo)==0:
                di = self.cohesion(this,flock)
            elif len(self.zoa)!=0 and len(self.zoo)!=0:
                di = 0.5*(self.align(flock) + self.cohesion(this,flock))
            else:
                di = this.velocity
        return di
        
    def saturate_up(self,u,u_lim):
        if abs(u) > u_lim:
            u = u*u_lim/abs(u)
        return u

    def saturate(self,u,u_lim,l_lim):
        if abs(u) > u_lim:
            u = u*u_lim/abs(u)
        if abs(u) < l_lim:
            u = u*l_lim/abs(u)
        return u
    
    # def angle_wrap(self,angle):
    #     if angle<0:    
    #         angle += 2*np.pi
    #     return angle

    # def angle_wrap_0_2pi(self,angle):
    #     while angle<0:
    #         angle += 2*np.pi
    #     while angle>=2*np.pi:
    #         angle -= 2*np.pi
    #     return angle
    
    # def angle_w_n_pi_p_pi(self,angle):
    #     while angle>np.pi:    
    #         angle -= 2*np.pi
    #     while angle<-np.pi:    
    #         angle += 2*np.pi
    #     return angle
    
