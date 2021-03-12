#   Author : Swadhin Agrawal
#   Couzin's model 2D Animation

import numpy as np
import matplotlib.pyplot as plt

class Animation:
      def __init__(self,fig,ax):
            self.head_width = 1
            self.head_length = 1
            self.body_length = 2
            self.t = []
            self.fig = fig
            self.ax = ax
            self.pose = None
            self.velocity= None
            self.agent = None
            self.initial = True

      def update(self,pose,velocity):
            self.pose = pose
            self.velocity = velocity
            angle_x = np.arctan2((velocity[1]**2 + velocity[2]**2)**0.5,velocity[0])
            angle_y = np.arctan2((velocity[0]**2 + velocity[2]**2)**0.5,velocity[1])
            angle_z = np.arctan2((velocity[1]**2 + velocity[0]**2)**0.5,velocity[2])
            if self.initial ==False:     
                  self.agent.remove()
            else:
                  self.initial = False 
            self.agent = self.ax.quiver(pose[0], pose[1], pose[2],np.cos(angle_x)*self.body_length, np.cos(angle_y)*self.body_length,np.cos(angle_z)*self.body_length, fc='red', ec='red')
            