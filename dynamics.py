#!/usr/bin/env python

from multiprocessing import Process, Manager, Value

def update_position(velocity_signal,current_position,timestep):

    timestep=0.1

    for i in current_position:
        current_position=current_position+velocity_signal*timestep
    
if __name__=="__main__":

    source=[0,0,0]
    dest=[10,10,10]

    velocity_signal=multiprocessing.Array('d',3)
    current_position=multiprocessing.Array('d',3)

    # p(t+1) = p(t) + u(t)*dt
    p1 = multiprocessing.Process(target=update_position, args=(velocity_signal, current_position))