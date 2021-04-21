#!/usr/bin/env python

import numpy as np
from math import sqrt

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def find_dist_from_dest_vec(vec,point,current_position):

    param = np.dot(point,vec)-np.dot(current_position,vec)

    dx=param*vec[0]+current_position[0]-point[0]
    dy=param*vec[1]+current_position[1]-point[1]
    dz=param*vec[2]+current_position[2]-point[2]

    return sqrt(dx*dx+dy*dy+dz*dz)

def distance(a,b):
    vec=[a[0]-b[0],a[1]-b[1],a[2]-b[2]]
    return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2])

def find_obstacles(obstacles,current_position,bot_range):

    visible=[]

    for obstacle in obstacles:

        if(distance(obstacle,current_position)<bot_range):
            visible.append(obstacle)
    
    return visible

def sort_by_distance(obstacles,current_position):
    n = len(obstacles)

    for i in range(n):

        for j in range(0, n-i-1):

            if distance(current_position,obstacles[j]) > distance(current_position,obstacles[j+1]) :
                obstacles[j], obstacles[j+1] = obstacles[j+1], obstacles[j]

    return obstacles

def obstacle_in_path(obstacles_in_range,dest_vector,current_position,bot_radius):

    in_path=[]

    inline=0

    for obstacle in obstacles_in_range:

        if(abs(find_dist_from_dest_vec(dest_vector,obstacle,current_position))<bot_radius*2):
            inline=1

        if(abs(find_dist_from_dest_vec(dest_vector,obstacle,current_position))<bot_radius):
            in_path.append(obstacle)

    return in_path,inline

def calculate_displacement_vec(obstacles_in_path,dest_vector,current_position):

    param = np.dot(obstacles_in_path[0],dest_vector)-np.dot(current_position,dest_vector)

    displacement_vec=[]

    displacement_vec_x=2*param*dest_vector[0]+current_position[0]-obstacles_in_path[0][0]
    displacement_vec_y=2*param*dest_vector[1]+current_position[1]-obstacles_in_path[0][1]
    displacement_vec_z=2*param*dest_vector[2]+current_position[2]-obstacles_in_path[0][2]

    displacement_vec=[displacement_vec_x,displacement_vec_y,displacement_vec_z]

    return displacement_vec

def perform_planning(velocity_signal,current_position,bot_range,world,bot_radius,dest,obstacles):

    dest_vector=[dest[0]-current_position[0],dest[1]-current_position[1],dest[2]-current_position[2]]
    dest_vector=normalize(dest_vector)

    obstacles_in_range=find_obstacles(obstacles,current_position,bot_range)

    obstacles_in_path,inline=obstacle_in_path(obstacles_in_range,dest_vector,current_position,bot_radius)

    obstacles_in_path=sort_by_distance(obstacles_in_path,current_position)

    if(inline==1):

        for i in range(3):
            velocity_signal[i]=0.0

        velocity_signal[0]=bot_radius/5

    else:

        if(len(obstacles_in_path)==0):

            for i in range(3):
                velocity_signal[i]=dest_vector[i]/3

        else:

            x=calculate_displacement_vec(obstacles_in_path,dest_vector,current_position)

            for i in range(3):
                velocity_signal[i]=x[i]/3
