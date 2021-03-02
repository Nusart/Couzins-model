#   Author : Swadhin Agrawal
#   Couzin's model Simulation

import numpy as np
import matplotlib.pyplot as plt
import couzinAnim as anim
import couzinDynamics as couzin
import keyboard

plt.ion()

number_of_drones = 20
dt = 0.1
Rr = 3
Ro = 3
Ra = 18
s = 15
s_min = 10
max_omega = 150*np.pi/(180) 
visibility_angle = 150*np.pi/180 
boundary_min_x = 100
boundary_max_x = 200
boundary_min_y = 100
boundary_max_y = 200
#	[x,y] coordinates of swarm members
positions = np.random.uniform(low = 100,high = 120 ,size=(number_of_drones,2))

#	[vx,vy] velocities of swarm members
velocities = np.random.uniform(low = 0,high = 10 ,size=(number_of_drones,2))

#	Figure for visualization
fig = plt.figure()
flock = []
#	Create the Flock
for i in range(number_of_drones):
    flock.append(anim.Animation(fig))
    flock[i].update(positions[i],velocities[i])

couzin_model = couzin.couzinsModel(Rr,Ro,Ra,visibility_angle)


while True:
    for i in range(number_of_drones):
        couzin_model.neighbours_in_zones(flock[i],flock[:i]+flock[i+1:]) 
        di = couzin_model.net_di(flock[i],flock[:i]+flock[i+1:])   
        steer = np.subtract(di,flock[i].velocity)
        flock[i].velocity += steer*dt
        flock[i].pose += dt*couzin_model.saturate(np.linalg.norm(flock[i].velocity),s,s_min)*flock[i].velocity/np.linalg.norm(flock[i].velocity)
        if flock[i].pose[0]>boundary_max_x:
            flock[i].pose[0] = flock[i].pose[0] - boundary_max_x + boundary_min_x
        if flock[i].pose[0]<boundary_min_x:
            flock[i].pose[0] = flock[i].pose[0] + abs(boundary_max_x)
        if flock[i].pose[1]>boundary_max_y:
            flock[i].pose[1] = flock[i].pose[1] - boundary_max_y + boundary_min_y
        if flock[i].pose[1]<boundary_min_y:
            flock[i].pose[1] = flock[i].pose[1] + abs(boundary_max_y)

        flock[i].update(flock[i].pose,flock[i].velocity)
        couzin_model.zor = []
        couzin_model.zoa = []
        couzin_model.zoo = []

    plt.xlim(boundary_min_x,boundary_max_x)
    plt.ylim(boundary_min_y,boundary_max_y)
    plt.pause(0.001)
    fig.clear() 
    if keyboard.is_pressed('a'):
        print("ok")
        plt.close()
        break

