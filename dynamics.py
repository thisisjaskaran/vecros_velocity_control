#!/usr/bin/env python

import multiprocessing
import velocity_control
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

def initialise_world(world_size):

    world = [[ [0 for col in range(world_size)] for col in range(world_size)] for row in range(world_size)]

    return world

def add_obstacle(world,x,y,z,obstacles):

    world[x][y][z]=1

    obstacles.append([x,y,z])

    return world

def update_position(velocity_signal,current_position):

    timestep=0.1

    # print("Position: {}".format(current_position[:]))

    # print("Signal: {}".format(velocity_signal[:]))

    for i in range(3):
        current_position[i]=current_position[i]+velocity_signal[i]*timestep

    print("Updated Position: {}".format(current_position[:]))
    
if __name__=="__main__":

    destination_threshold=0.1
    source=[0.0,0.0,0.0]
    dest=[10.0,9.0,0.0]

    world_size=10

    velocity_signal=multiprocessing.Array('d',3)
    current_position=multiprocessing.Array('d',3)

    current_position[0]=source[0]
    current_position[1]=source[1]
    current_position[2]=source[2]

    bot_range=2.0

    world=initialise_world(world_size)

    obstacles=[]

    world=add_obstacle(world,5,6,5,obstacles)
    world=add_obstacle(world,1,2,3,obstacles)
    world=add_obstacle(world,5,4,3,obstacles)
    world=add_obstacle(world,7,6,9,obstacles)
    world=add_obstacle(world,1,6,2,obstacles)
    world=add_obstacle(world,9,9,2,obstacles)
    world=add_obstacle(world,9,9,9,obstacles)
    world=add_obstacle(world,1,1,2,obstacles)
    world=add_obstacle(world,2,2,2,obstacles)
    world=add_obstacle(world,6,6,6,obstacles)

    bot_radius=0.2

    # fig = plt.figure(figsize=(dest[0]-source[0],dest[1]-source[1]))
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')

    # for i in range(3):
    #     velocity_signal[i]=0.0

    while(abs(current_position[0]-dest[0])>destination_threshold or abs(current_position[1]-dest[1])>destination_threshold or abs(current_position[2]-dest[2])>destination_threshold):

        p1 = multiprocessing.Process(target=velocity_control.perform_planning, args=(velocity_signal, current_position,bot_range,world,bot_radius,dest,obstacles))

        p2 = multiprocessing.Process(target=update_position,args=(velocity_signal,current_position))

        # fig = plt.figure(figsize=(20,20))
        # ax = fig.add_subplot(111, projection='3d')
        ax.scatter(current_position[0],current_position[1],current_position[2])
        # plt.show()

        p1.start()
        p2.start()

        p1.join()
        p2.join()

        # time.sleep(1)
    
    for obstacle in obstacles:
        ax.scatter(obstacle[0],obstacle[1],obstacle[2])

    plt.show()

    print("Destination Reached!")