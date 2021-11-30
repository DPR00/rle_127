# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time
import numpy as np
import pybullet as p  # PyBullet simulator

from .controller import c_walking_ID, c#, c_walking_IK_bezier # Controller functions
# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

import sys, select, termios, tty
from .sim_fb import systemStateEstimator
import csv
####################
####  FUNCTIONS  ###
####################

def getKey(key_timeout):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

##This part of code is just to save the raw telemetry data.
fieldnames = ["t","FR","FL","BR","BL","n_FR", "n_FL","n_BR","n_BL"]
with open('telemetria/data.csv','w') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames = fieldnames)
    csv_writer.writeheader()

def update_data():
    #take meassurement from simulation
    t , X = meassure.states()
    U , Ui ,torque = meassure.controls()
    norm_FR = np.linalg.norm(np.array([Ui[0,0], Ui[1,0], Ui[2,0]]))
    norm_FL = np.linalg.norm(np.array([Ui[3,0], Ui[4,0], Ui[5,0]]))
    norm_BR = np.linalg.norm(np.array([Ui[6,0], Ui[7,0], Ui[8,0]]))
    norm_BL = np.linalg.norm(np.array([Ui[9,0], Ui[10,0], Ui[11,0]]))
    with open('telemetria/data.csv','a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
        info = {"t" : t[-1],
                "FR" : Ui[2,0],
                "FL" : Ui[5,0],
                "BR" : Ui[8,0],
                "BL" : Ui[11,0],
                "n_FR": norm_FR,
                "n_FL": norm_FL,
                "n_BR": norm_BR,
                "n_BL": norm_BL}
        csv_writer.writerow(info)

####################
#  INITIALIZATION ##
####################

dt = 0.001  # time step of the simulation
# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation)
realTimeSimulation = True
enableGUI = True  # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)
obstacle = p.loadURDF("Models/obstacle.urdf", useFixedBase=True)
meassure = systemStateEstimator(robotId)

###############
#  MAIN LOOP ##
###############

i=0
while (1):  # run the simulation during dt * i_max seconds (simulation time)
    
    # Time at the start of the loop
    if realTimeSimulation:
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)
    
    key_timeout = 0.0005
    key = getKey(key_timeout)
    # Call controller to get torques for all joints
    jointTorques = c(q, qdot, dt, solo, i * dt,key)
    #print("Dentro del main ..")
    #print(jointTorques.shape)
    #print(q.shape)
    #print(qdot.shape)
    #print(jointTorques)
    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()
    update_data()

    i+=1
    # Sleep to get a real time simulation
    if realTimeSimulation:
        t_sleep = dt - (time.clock() - t0)
        #print("tiempo")
        #print(t_sleep)
        if t_sleep > 0:
            time.sleep(t_sleep)


# Shut down the PyBullet client
p.disconnect()
