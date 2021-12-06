# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import pybullet as p  # PyBullet simulator

def generar_ladrillos():

    obstacle11 = p.loadURDF("Models/ladrillos.urdf",[0.5, 0.3, 0] ,useFixedBase=True)
    obstacle12 = p.loadURDF("Models/ladrillos.urdf",[0.5, 0.1, 0.01] ,useFixedBase=True)
    obstacle13 = p.loadURDF("Models/ladrillos.urdf",[0.5, -0.1, -0.01] ,useFixedBase=True)
    obstacle14 = p.loadURDF("Models/ladrillos.urdf",[0.5, -0.3, 0.015] ,useFixedBase=True)
    obstacle15 = p.loadURDF("Models/ladrillos.urdf",[0.5, -0.5, 0.02] ,useFixedBase=True)
    obstacle16 = p.loadURDF("Models/ladrillos.urdf",[0.5, 0.5, -0.015] ,useFixedBase=True)
    obstacle17 = p.loadURDF("Models/ladrillos.urdf",[0.5, -0.7, 0] ,useFixedBase=True)
    obstacle18 = p.loadURDF("Models/ladrillos.urdf",[0.5, 0.7, 0.005] ,useFixedBase=True)

    obstacle21= p.loadURDF("Models/ladrillos.urdf",[0.6, 0.3, -0.01] ,useFixedBase=True)
    obstacle22 = p.loadURDF("Models/ladrillos.urdf",[0.6, 0.1, 0] ,useFixedBase=True)
    obstacle23 = p.loadURDF("Models/ladrillos.urdf",[0.6, -0.1, 0.025] ,useFixedBase=True)
    obstacle24 = p.loadURDF("Models/ladrillos.urdf",[0.6, -0.3, -0.015] ,useFixedBase=True)
    obstacle25 = p.loadURDF("Models/ladrillos.urdf",[0.6, -0.5, -0.015] ,useFixedBase=True)
    obstacle26 = p.loadURDF("Models/ladrillos.urdf",[0.6, 0.5, 0.025] ,useFixedBase=True)
    obstacle27 = p.loadURDF("Models/ladrillos.urdf",[0.6, -0.7, 0.01] ,useFixedBase=True)
    obstacle28 = p.loadURDF("Models/ladrillos.urdf",[0.6, 0.7, -0.017] ,useFixedBase=True)

    obstacle31 = p.loadURDF("Models/ladrillos.urdf",[0.7, 0.3, 0.02] ,useFixedBase=True)
    obstacle32 = p.loadURDF("Models/ladrillos.urdf",[0.7, 0.1, -0.01] ,useFixedBase=True)
    obstacle33 = p.loadURDF("Models/ladrillos.urdf",[0.7, -0.1, 0.01] ,useFixedBase=True)
    obstacle34 = p.loadURDF("Models/ladrillos.urdf",[0.7, -0.3, 0.01] ,useFixedBase=True)
    obstacle35 = p.loadURDF("Models/ladrillos.urdf",[0.7, -0.5, 0] ,useFixedBase=True)
    obstacle36 = p.loadURDF("Models/ladrillos.urdf",[0.7, 0.5, 0.005] ,useFixedBase=True)
    obstacle37 = p.loadURDF("Models/ladrillos.urdf",[0.7, -0.7, 0.002] ,useFixedBase=True)
    obstacle38 = p.loadURDF("Models/ladrillos.urdf",[0.7, 0.7, -0.01] ,useFixedBase=True)

    obstacle41 = p.loadURDF("Models/ladrillos.urdf",[0.8, 0.3, 0.005] ,useFixedBase=True)
    obstacle42 = p.loadURDF("Models/ladrillos.urdf",[0.8, 0.1, 0] ,useFixedBase=True)
    obstacle43 = p.loadURDF("Models/ladrillos.urdf",[0.8, -0.1, 0.016] ,useFixedBase=True)
    obstacle44 = p.loadURDF("Models/ladrillos.urdf",[0.8, -0.3, -0.01] ,useFixedBase=True)
    obstacle45 = p.loadURDF("Models/ladrillos.urdf",[0.8, -0.5, 0.01] ,useFixedBase=True)
    obstacle46 = p.loadURDF("Models/ladrillos.urdf",[0.8, 0.5, -0.015] ,useFixedBase=True)
    obstacle47 = p.loadURDF("Models/ladrillos.urdf",[0.8, -0.7, 0] ,useFixedBase=True)
    obstacle48 = p.loadURDF("Models/ladrillos.urdf",[0.8, 0.7, 0.01] ,useFixedBase=True)

    obstacle51 = p.loadURDF("Models/ladrillos.urdf",[0.9, 0.3, 0.02] ,useFixedBase=True)
    obstacle52 = p.loadURDF("Models/ladrillos.urdf",[0.9, 0.1, -0.02] ,useFixedBase=True)
    obstacle53 = p.loadURDF("Models/ladrillos.urdf",[0.9, -0.1, 0.005] ,useFixedBase=True)
    obstacle54 = p.loadURDF("Models/ladrillos.urdf",[0.9, -0.3, 0.015] ,useFixedBase=True)
    obstacle55 = p.loadURDF("Models/ladrillos.urdf",[0.9, -0.5, 0.02] ,useFixedBase=True)
    obstacle56 = p.loadURDF("Models/ladrillos.urdf",[0.9, 0.5, 0.006] ,useFixedBase=True)
    obstacle57 = p.loadURDF("Models/ladrillos.urdf",[0.9, -0.7, -0.012] ,useFixedBase=True)
    obstacle58 = p.loadURDF("Models/ladrillos.urdf",[0.9, 0.7, 0] ,useFixedBase=True)

    obstacle61 = p.loadURDF("Models/ladrillos.urdf",[1, 0.3, 0] ,useFixedBase=True)
    obstacle62 = p.loadURDF("Models/ladrillos.urdf",[1, 0.1, 0.01] ,useFixedBase=True)
    obstacle63 = p.loadURDF("Models/ladrillos.urdf",[1, -0.1, -0.01] ,useFixedBase=True)
    obstacle64 = p.loadURDF("Models/ladrillos.urdf",[1, -0.3, -0.015] ,useFixedBase=True)
    obstacle65 = p.loadURDF("Models/ladrillos.urdf",[1, -0.5, 0] ,useFixedBase=True)
    obstacle66 = p.loadURDF("Models/ladrillos.urdf",[1, 0.5, -0.005] ,useFixedBase=True)
    obstacle67 = p.loadURDF("Models/ladrillos.urdf",[1, -0.7, 0.01] ,useFixedBase=True)
    obstacle68 = p.loadURDF("Models/ladrillos.urdf",[1, 0.7, 0.015] ,useFixedBase=True)

    obstacle71 = p.loadURDF("Models/ladrillos.urdf",[1.1, 0.3, 0.025] ,useFixedBase=True)
    obstacle72 = p.loadURDF("Models/ladrillos.urdf",[1.1, 0.1, -0.01] ,useFixedBase=True)
    obstacle73 = p.loadURDF("Models/ladrillos.urdf",[1.1, -0.1, 0.015] ,useFixedBase=True)
    obstacle74 = p.loadURDF("Models/ladrillos.urdf",[1.1, -0.3, 0] ,useFixedBase=True)
    obstacle75 = p.loadURDF("Models/ladrillos.urdf",[1.1, -0.5, 0.015] ,useFixedBase=True)
    obstacle76 = p.loadURDF("Models/ladrillos.urdf",[1.1, 0.5, -0.015] ,useFixedBase=True)
    obstacle77 = p.loadURDF("Models/ladrillos.urdf",[1.1, -0.7, 0] ,useFixedBase=True)
    obstacle78 = p.loadURDF("Models/ladrillos.urdf",[1.1, 0.7, 0] ,useFixedBase=True)

    obstacle81 = p.loadURDF("Models/ladrillos.urdf",[1.2, 0.3, 0] ,useFixedBase=True)
    obstacle82 = p.loadURDF("Models/ladrillos.urdf",[1.2, 0.1, 0.01] ,useFixedBase=True)
    obstacle83 = p.loadURDF("Models/ladrillos.urdf",[1.2, -0.1, -0.01] ,useFixedBase=True)
    obstacle84 = p.loadURDF("Models/ladrillos.urdf",[1.2, -0.3, -0.015] ,useFixedBase=True)
    obstacle85 = p.loadURDF("Models/ladrillos.urdf",[1.2, -0.5, 0.02] ,useFixedBase=True)
    obstacle86 = p.loadURDF("Models/ladrillos.urdf",[1.2, 0.5, -0.022] ,useFixedBase=True)
    obstacle87 = p.loadURDF("Models/ladrillos.urdf",[1.2, -0.7, 0.015] ,useFixedBase=True)
    obstacle88 = p.loadURDF("Models/ladrillos.urdf",[1.2, 0.7, 0.015] ,useFixedBase=True)



