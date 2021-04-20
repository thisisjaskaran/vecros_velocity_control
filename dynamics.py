#!/usr/bin/env python

import multiprocessing
import velocity_control

def initialise_world(world_size):

    world = [[ [0 for col in range(world_size)] for col in range(world_size)] for row in range(world_size)]

    return world

def add_obstacle(world,x,y,z):

    world[x][y][z]=1

    return world

def update_position(velocity_signal,current_position):

    timestep=0.1

    print("Position: {}".format(current_position[:]))

    print("Signal: {}".format(velocity_signal[:]))

    for i in range(3):
        current_position[i]=current_position[i]+velocity_signal[i]*timestep

    print("Updated Position: {}".format(current_position[:]))
    
if __name__=="__main__":

    destination_threshold=0.01
    source=[0.0,0.0,0.0]
    dest=[10.0,10.0,10.0]

    world_size=10

    velocity_signal=multiprocessing.Array('d',3)
    current_position=multiprocessing.Array('d',3)

    bot_range=5.0

    world=initialise_world(world_size)

    world=add_obstacle(world,1,1,1)

    bot_dimensions=[2.0,2.0,2.0]

    while(abs(current_position[0]-dest[0])>destination_threshold and abs(current_position[1]-dest[1])>destination_threshold and abs(current_position[2]-dest[2])>destination_threshold):

        p1 = multiprocessing.Process(target=velocity_control.perform_planning, args=(velocity_signal, current_position,bot_range,world))

        p2 = multiprocessing.Process(target=update_position,args=(velocity_signal,current_position))

        p1.start()
        p2.start()

        p1.join()
        p2.join()