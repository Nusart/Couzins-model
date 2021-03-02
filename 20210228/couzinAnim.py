#   Author : Swadhin Agrawal
#   Couzin's model 2D Animation

import numpy as np
import matplotlib.pyplot as plt

class Animation:
      def __init__(self,fig):
            self.head_width = 1
            self.head_length = 1
            self.body_length = 2
            self.t = []
            self.fig = fig
            self.pose = None
            self.velocity= None

      def update(self,pose,velocity):
            self.pose = pose
            self.velocity = velocity
            angle = np.arctan2(velocity[1],velocity[0])
            self.swramy(pose,angle)

      def swramy(self,pose,angle):
            plt.arrow(pose[0], pose[1], np.cos(angle)*self.body_length, np.sin(angle)*self.body_length, head_width=self.head_width, head_length=self.head_length, fc='red', ec='red')
            