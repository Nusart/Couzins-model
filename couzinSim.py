#   Author : Swadhin Agrawal
#   Couzin's model Simulation

import numpy as np
import matplotlib.pyplot as plt
import couzinAnim as anim
import couzinDynamics as couzin

plt.ion()

number_of_drones = 10
origin = np.array([0,0])
dt = 0.1
Rr = 5
Ro = 50
Ra = 65
theta = 2*np.pi/180  # turning rate in degrees per second
s = 3
max_s = 10
blind_spot = 2*np.pi  #   >40 degree is blind from each side

drones_positions = np.zeros((number_of_drones,3))
#	[x,y] coordinates of swarm members
drones_positions[:,:2] = np.random.uniform(low = 50,high =60 ,size=(number_of_drones,2))

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

couzin_model = couzin.couzinsModel(number_of_drones,s,drones_positions,Rr,Ro,Ra,theta,blind_spot)

count = 0

while count<=1000:
    for i in range(number_of_drones):
        couzin_model.neighbours_in_zones(couzin_model.swarm[i])
        di = couzin_model.net_di(couzin_model.swarm[i])

        desired_angle = couzin_model.angle_wrap(np.arctan2(di[1],di[0]))

        des_s =couzin_model.saturate(np.linalg.norm(di),max_s)

        couzin_model.swarm_velocities[i] += np.array([des_s,couzin_model.saturate((couzin_model.angle_wrap_0_2pi(desired_angle-drones_positions[i,2]))/dt,theta)])

        couzin_model.swarm_velocities[i,0] = couzin_model.saturate(couzin_model.swarm_velocities[i,0],max_s)
        couzin_model.swarm_velocities[i,1] = couzin_model.saturate(couzin_model.swarm_velocities[i,1],theta)

        couzin_model.swarm[i,2] += couzin_model.swarm_velocities[i,1] * dt

        couzin_model.swarm[i,2] = couzin_model.angle_wrap_0_2pi(couzin_model.swarm[i,2])

        couzin_model.swarm[i,:2] += np.array([couzin_model.swarm_velocities[i,0]*dt*np.cos(couzin_model.swarm[i,2]),couzin_model.swarm_velocities[i,0]*dt*np.sin(couzin_model.swarm[i,2])])

        flock[i].update(couzin_model.swarm[i])
        couzin_model.zor = []
        couzin_model.zoa = []
        couzin_model.zoo = []
        plt.xlim(0,1000)
        plt.ylim(0,1000)
    count += 1
    plt.pause(0.0001)
    fig.clear()
plt.waitforbuttonpress()
plt.close()

