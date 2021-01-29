#   Author : Swadhin Agrawal
#   Couzin's model Simulation

import numpy as np
import matplotlib.pyplot as plt
import couzinAnim as anim
import couzinDynamics as couzin

plt.ion()

number_of_drones = 50
origin = np.array([0,0])
dt = 0.1
Rr = 1
Ro = 3
Ra = 15
theta = 100*np.pi/180  # turning rate in degrees per second
s = 5

blind_spot = 150*np.pi/180  #   >40 degree is blind from each side

drones_positions = np.zeros((number_of_drones,3))
#	[x,y] coordinates of swarm members
drones_positions[:,:2] = np.random.uniform(low = 0,high =20 ,size=(number_of_drones,2))
#	Theta (in Rad) of swarm members
drones_positions[:,2] = np.array([np.random.uniform(low = 45,high =45)*np.pi/180 for i in range(number_of_drones)])

#	Figure for visualization
fig = plt.figure()
flock = []
#	Create the Flock
for i in range(number_of_drones):
    flock.append(anim.Animation(fig))
    flock[i].update(drones_positions[i])
#	Update Flock state

couzin_model = couzin.couzinsModel(number_of_drones,s,drones_positions,Rr,Ro,Ra,blind_spot)

count = 0

while count<=1000:
    for i in range(number_of_drones):
        couzin_model.neighbours_in_zones(couzin_model.swarm[i])
        di = couzin_model.net_di(couzin_model.swarm[i])

        di_ang = np.arctan2(di[1],di[0])

        desired_angle = ((di_ang - couzin_model.swarm[i,2])/abs(di_ang - couzin_model.swarm[i,2]))*np.arccos(np.dot(di,couzin_model.swarm[i,:2])/(np.linalg.norm(di)*np.linalg.norm(couzin_model.swarm[i,:2])))

        couzin_model.swarm[i,2] += couzin_model.saturate(desired_angle,theta * dt)

        # couzin_model.swarm[i,2] = couzin_model.angle_wrap_0_2pi(couzin_model.swarm[i,2])

        couzin_model.swarm[i,:2] += dt* np.array([s*np.cos(couzin_model.swarm[i,2]),s*np.sin(couzin_model.swarm[i,2])])

        flock[i].update(couzin_model.swarm[i])
        couzin_model.zor = []
        couzin_model.zoa = []
        couzin_model.zoo = []
        plt.xlim(-100,100)
        plt.ylim(-100,100)
    count += 1
    plt.pause(0.00001)
    fig.clear()
plt.waitforbuttonpress()
plt.close()

