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
            self.pose =[]

      def update(self,pose):
            self.pose.append(pose)
            self.swramy(pose)

      def swramy(self,pose):
            plt.arrow(pose[0], pose[1], np.cos(pose[2])*self.body_length, np.sin(pose[2])*self.body_length, head_width=self.head_width, head_length=self.head_length, fc='red', ec='red')
            