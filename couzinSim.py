#   Author : Swadhin Agrawal
#   Couzin's model Simulation

import numpy as np
import matplotlib.pyplot as plt
import couzinAnim as anim
import couzinDynamics as couzin

plt.ion()

number_of_drones = 50
dt = 0.1
Rr = 1
Ro = 23
Ra = 150
s = 30
theta = 70*np.pi/(180) # turning rate in degrees per second

blind_spot = 360*np.pi/180  #   >40 degree is blind from each side

drones_positions = np.zeros((number_of_drones,3))
#	[x,y] coordinates of swarm members
drones_positions[:,:2] = np.random.uniform(low = 0,high =20 ,size=(number_of_drones,2))
#	Theta (in Rad) of swarm members
drones_positions[:,2] = np.array([np.random.uniform(low = 0,high =40)*np.pi/180 for i in range(number_of_drones)])

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
        couzin_model.neighbours_in_zones(couzin_model.swarm[i]) ####### OK
        di = couzin_model.net_di(couzin_model.swarm[i])   ####### confusion, whether current drone should be affected by others or it should affect others or both ways effect should be there?
        di = di/np.linalg.norm(di)

        # di_ang = couzin_model.angle_wrap_0_2pi(np.arctan2(di[1],di[0]))
        di_ang = np.arctan2(di[1],di[0])

        # print("current heading %f"%couzin_model.swarm[i,2])
        # print("direction to face %f"%di_ang)
        # print(di)
        # print([np.cos(couzin_model.swarm[i,2]),np.sin(couzin_model.swarm[i,2])])
        # print(np.arccos(np.dot(di,[np.cos(couzin_model.swarm[i,2]),np.sin(couzin_model.swarm[i,2])])/np.linalg.norm(di)))
        # print(-((di_ang - couzin_model.swarm[i,2])/abs(di_ang - couzin_model.swarm[i,2])))
        if di_ang - couzin_model.swarm[i,2] > 0.00 :
            # print(np.dot(di,[np.cos(couzin_model.swarm[i,2]),np.sin(couzin_model.swarm[i,2])]))
            desired_angle = ((di_ang - couzin_model.swarm[i,2])/abs(di_ang - couzin_model.swarm[i,2]))*np.arccos(np.dot(di,[np.cos(couzin_model.swarm[i,2]),np.sin(couzin_model.swarm[i,2])]))
        else:
            desired_angle = 0.0
        # print("desired change in angle %f"%desired_angle)

        couzin_model.swarm[i,2] += couzin_model.saturate(desired_angle,theta * dt)
        # print("angle after turning unwraped %f"%couzin_model.swarm[i,2])
        couzin_model.swarm[i,2] = couzin_model.angle_w_n_pi_p_pi(couzin_model.swarm[i,2])
        # print("Wraped heading %f"%couzin_model.swarm[i,2])
        couzin_model.swarm[i,:2] += dt* np.array([s*np.cos(couzin_model.swarm[i,2]),s*np.sin(couzin_model.swarm[i,2])])
        # print(couzin_model.swarm)
        flock[i].update(couzin_model.swarm[i])
        couzin_model.zor = []
        couzin_model.zoa = []
        couzin_model.zoo = []
        plt.xlim(-50,200)
        plt.ylim(-50,200)
    count += 1
    plt.pause(0.01)
    fig.clear()
plt.waitforbuttonpress()
plt.close()

