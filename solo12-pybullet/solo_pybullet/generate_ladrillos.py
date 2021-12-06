# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import pybullet as p  # PyBullet simulator

def generar_ladrillos():

    obstacle11 = p.loadURDF("Models/ladrillos.urdf",[0.5, 0.15, -0.01] ,useFixedBase=True)
    obstacle12 = p.loadURDF("Models/ladrillos.urdf",[0.5, 0.05, -0.01] ,useFixedBase=True)
    obstacle13 = p.loadURDF("Models/ladrillos.urdf",[0.5, -0.05, -0.01] ,useFixedBase=True)
    obstacle14 = p.loadURDF("Models/ladrillos.urdf",[0.5, -0.15, -0.015] ,useFixedBase=True)

    obstacle21= p.loadURDF("Models/ladrillos.urdf",[0.6, 0.15, -0.01] ,useFixedBase=True)
    obstacle22 = p.loadURDF("Models/ladrillos.urdf",[0.6, 0.05, 0] ,useFixedBase=True)
    obstacle23 = p.loadURDF("Models/ladrillos.urdf",[0.6, -0.05, 0.025] ,useFixedBase=True)
    obstacle24 = p.loadURDF("Models/ladrillos.urdf",[0.6, -0.15, -0.015] ,useFixedBase=True)

    obstacle31 = p.loadURDF("Models/ladrillos.urdf",[0.7, 0.15, 0.02] ,useFixedBase=True)
    obstacle32 = p.loadURDF("Models/ladrillos.urdf",[0.7, 0.05, -0.01] ,useFixedBase=True)
    obstacle33 = p.loadURDF("Models/ladrillos.urdf",[0.7, -0.05, 0.01] ,useFixedBase=True)
    obstacle34 = p.loadURDF("Models/ladrillos.urdf",[0.7, -0.15, 0.01] ,useFixedBase=True)

    obstacle41 = p.loadURDF("Models/ladrillos.urdf",[0.8, 0.15, 0.005] ,useFixedBase=True)
    obstacle42 = p.loadURDF("Models/ladrillos.urdf",[0.8, 0.05, 0] ,useFixedBase=True)
    obstacle43 = p.loadURDF("Models/ladrillos.urdf",[0.8, -0.05, 0.016] ,useFixedBase=True)
    obstacle44 = p.loadURDF("Models/ladrillos.urdf",[0.8, -0.15, -0.01] ,useFixedBase=True)

    obstacle51 = p.loadURDF("Models/ladrillos.urdf",[0.9, 0.15, 0.02] ,useFixedBase=True)
    obstacle52 = p.loadURDF("Models/ladrillos.urdf",[0.9, 0.05, -0.02] ,useFixedBase=True)
    obstacle53 = p.loadURDF("Models/ladrillos.urdf",[0.9, -0.05, 0.005] ,useFixedBase=True)
    obstacle54 = p.loadURDF("Models/ladrillos.urdf",[0.9, -0.15, 0.015] ,useFixedBase=True)

    obstacle61 = p.loadURDF("Models/ladrillos.urdf",[1, 0.15, 0] ,useFixedBase=True)
    obstacle62 = p.loadURDF("Models/ladrillos.urdf",[1, 0.05, 0.01] ,useFixedBase=True)
    obstacle63 = p.loadURDF("Models/ladrillos.urdf",[1, -0.05, -0.01] ,useFixedBase=True)
    obstacle64 = p.loadURDF("Models/ladrillos.urdf",[1, -0.15, -0.015] ,useFixedBase=True)

    obstacle71 = p.loadURDF("Models/ladrillos.urdf",[1.1, 0.15, 0.025] ,useFixedBase=True)
    obstacle72 = p.loadURDF("Models/ladrillos.urdf",[1.1, 0.05, -0.01] ,useFixedBase=True)
    obstacle73 = p.loadURDF("Models/ladrillos.urdf",[1.1, -0.05, 0.015] ,useFixedBase=True)
    obstacle74 = p.loadURDF("Models/ladrillos.urdf",[1.1, -0.15, 0] ,useFixedBase=True)

    obstacle81 = p.loadURDF("Models/ladrillos.urdf",[1.2, 0.15, 0] ,useFixedBase=True)
    obstacle82 = p.loadURDF("Models/ladrillos.urdf",[1.2, 0.05, 0.01] ,useFixedBase=True)
    obstacle83 = p.loadURDF("Models/ladrillos.urdf",[1.2, -0.05, -0.01] ,useFixedBase=True)
    obstacle84 = p.loadURDF("Models/ladrillos.urdf",[1.2, -0.15, -0.015] ,useFixedBase=True)




