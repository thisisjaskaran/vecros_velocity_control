#!/usr/bin/env python

import multiprocessing
import velocity_control
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# initialise world
def initialise_world(world_size):

    world = [[ [0 for col in range(world_size)] for col in range(world_size)] for row in range(world_size)]

    return world

# function to add obstacles
def add_obstacle(world,x,y,z,obstacles):

    world[x][y][z]=1

    obstacles.append([x,y,z])

    return world

# update position based on control signal
def update_position(velocity_signal,current_position):

    timestep=0.1

    for i in range(3):
        current_position[i]=current_position[i]+velocity_signal[i]*timestep

    print("Updated Position: {}".format(current_position[:]))
    
if __name__=="__main__":

    # define bot and world parameters
    destination_threshold=0.1
    source=[0.0,0.0,0.0]
    dest=[10.0,10.0,10.0]
    bot_range=1.0
    bot_radius=0.5

    #define shared variables
    velocity_signal=multiprocessing.Array('d',3)
    current_position=multiprocessing.Array('d',3)

    current_position[0]=source[0]
    current_position[1]=source[1]
    current_position[2]=source[2]

    # initialise world of size (world_size * world_size * world_size) and add obstacles
    world_size=10

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

    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')

    while(abs(current_position[0]-dest[0])>destination_threshold or abs(current_position[1]-dest[1])>destination_threshold or abs(current_position[2]-dest[2])>destination_threshold):

        p1 = multiprocessing.Process(target=velocity_control.perform_planning, args=(velocity_signal, current_position,bot_range,world,bot_radius,dest,obstacles))

        p2 = multiprocessing.Process(target=update_position,args=(velocity_signal,current_position))

        #plot bot position
        ax.scatter(current_position[0],current_position[1],current_position[2])

        p1.start()
        p2.start()

        p1.join()
        p2.join()

    #plot obstacles    
    for obstacle in obstacles:
        ax.scatter(obstacle[0],obstacle[1],obstacle[2])

    plt.show()

    print("Destination Reached!")