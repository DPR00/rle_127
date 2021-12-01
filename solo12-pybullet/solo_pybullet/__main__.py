# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time
import numpy as np
import pybullet as p  # PyBullet simulator

from .b_controller import c, c_simple#, c_walking_IK_bezier # Controller functions
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
fieldnames = ["t","FR","FL","BR","BL","n_FR", "n_FL","n_BR","n_BL",
            "t_FL_HAA","t_FL_HFE", "t_FL_KFE","t_FR_HAA","t_FR_HFE","t_FR_KFE",
            "t_HL_HAA","t_HL_HFE","t_HL_KFE", "t_HR_HAA","t_HR_HFE","t_HR_KFE",
            "tref_FL_HAA", "tref_FL_HFE", "tref_FL_KFE",
            "tref_FR_HAA", "tref_FR_HFE", "tref_FR_KFE",
            "tref_HL_HAA", "tref_HL_HFE", "tref_HL_KFE",
            "tref_HR_HAA", "tref_HR_HFE", "tref_HR_KFE"]

with open('telemetria/data_prueba.csv','w') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames = fieldnames)
    csv_writer.writeheader()

def update_data(jointTorques, torques_ref):
    #take meassurement from simulation
    t , X = meassure.states()
    U , Ui ,torque = meassure.controls()

    norm_FL = np.linalg.norm(np.array([Ui[0,0], Ui[1,0], Ui[2,0]]))
    norm_FR = np.linalg.norm(np.array([Ui[3,0], Ui[4,0], Ui[5,0]]))
    norm_BL = np.linalg.norm(np.array([Ui[6,0], Ui[7,0], Ui[8,0]]))
    norm_BR = np.linalg.norm(np.array([Ui[9,0], Ui[10,0], Ui[11,0]]))
    torque_FL_HAA = jointTorques[0,0]; torque_FL_HFE = jointTorques[1,0]; torque_FL_KFE = jointTorques[2,0]
    torque_FR_HAA = jointTorques[3,0]; torque_FR_HFE = jointTorques[4,0]; torque_FR_KFE = jointTorques[5,0]
    torque_HL_HAA = jointTorques[6,0]; torque_HL_HFE = jointTorques[7,0]; torque_HL_KFE = jointTorques[8,0]
    torque_HR_HAA = jointTorques[9,0]; torque_HR_HFE = jointTorques[10,0]; torque_HR_KFE = jointTorques[11,0]
    tref_FL_HAA = torques_ref[0,0]; tref_FL_HFE = torques_ref[1,0]; tref_FL_KFE = torques_ref[2,0]
    tref_FR_HAA = torques_ref[3,0]; tref_FR_HFE = torques_ref[4,0]; tref_FR_KFE = torques_ref[5,0]
    tref_HL_HAA = torques_ref[6,0]; tref_HL_HFE = torques_ref[7,0]; tref_HL_KFE = torques_ref[8,0]
    tref_HR_HAA = torques_ref[9,0]; tref_HR_HFE = torques_ref[10,0]; tref_HR_KFE = torques_ref[11,0]

    with open('telemetria/data_prueba.csv','a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
        info = {"t" : t[-1],
                "FR" : Ui[5,0], "FL" : Ui[2,0], "BR" : Ui[11,0], "BL" : Ui[8,0],
                "n_FR": norm_FR, "n_FL": norm_FL, "n_BR": norm_BR, "n_BL": norm_BL,
                "t_FL_HAA": torque_FL_HAA, "t_FL_HFE": torque_FL_HFE, "t_FL_KFE": torque_FL_KFE,
                "t_FR_HAA": torque_FR_HAA, "t_FR_HFE": torque_FR_HFE, "t_FR_KFE": torque_FR_KFE,
                "t_HL_HAA": torque_HL_HAA, "t_HL_HFE": torque_HL_HFE, "t_HL_KFE": torque_HL_KFE,
                "t_HR_HAA": torque_HR_HAA, "t_HR_HFE": torque_HR_HFE, "t_HR_KFE": torque_HR_KFE,
                "tref_FL_HAA": tref_FL_HAA, "tref_FL_HFE": tref_FL_HFE, "tref_FL_KFE": tref_FL_KFE,
                "tref_FR_HAA": tref_FR_HAA, "tref_FR_HFE": tref_FR_HFE, "tref_FR_KFE": tref_FR_KFE,
                "tref_HL_HAA": tref_HL_HAA, "tref_HL_HFE": tref_HL_HFE, "tref_HL_KFE": tref_HL_KFE,
                "tref_HR_HAA": tref_HR_HAA, "tref_HR_HFE": tref_HR_HFE, "tref_HR_KFE": tref_HR_KFE
                }
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
#obstacle = p.loadURDF("Models/wedge.urdf", useFixedBase=True)
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
    jointTorques, torques_ref = c(q, qdot, dt, solo, i * dt,key)

    update_data(jointTorques, torques_ref)
    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()
    

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
