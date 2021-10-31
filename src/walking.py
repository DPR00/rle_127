#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from robot_solo import Robot
from kcontroller import WQPControllerBase
from utils import Tmat, quaternionMult

def get_errors(pfr_des, pfl_des, prr_des, prl_des, pbase_des):
    # Errors
    efr = robot.error_position_fright(pfr_des)
    efl = robot.error_position_fleft(pfl_des)
    err = robot.error_position_rright(prr_des)
    erl = robot.error_position_rleft(prl_des)
    ebase = robot.error_pose_base(pbase_des)

    return efr, efl, err, erl, ebase

def get_jacobians():
    # Task Jacobians
    Jfr = robot.taskj_position_fright()
    Jfl = robot.taskj_position_fleft()
    Jrr = robot.taskj_position_rright()
    Jrl = robot.taskj_position_rleft()
    Jbase = robot.taskj_pose_base()

    return Jfr, Jfl, Jrr, Jrl, Jbase

def verificar_estado(estado,val):

    if estado == 0:
        pfr_des[0] += 2*val
        #pfl_des[0] += val
        #prr_des[0] += val
        #prl_des[0] += 2*val
        pbase_des[0] +=val
        estado = 5

    elif estado == 1:
        pfl_des[0] += 2*val
        #pfr_des[0] += val
        #prr_des[0] += val
        #prl_des[0] += val
        pbase_des[0] +=val
        estado = 3

    elif estado ==2:
        prr_des[0] += 2*val
        #pfl_des[0] += val
        #pfr_des[0] += val
        #prl_des[0] += val
        #pbase_des[0] +=val
        estado = 1

    elif estado ==3:
        prl_des[0] += 2*val
        #pfl_des[0] += 2*val
        #prr_des[0] += val
        #pfr_des[0] += val
        #pbase_des[0] +=val
        estado = 0
    elif estado ==5 :
        estado = 2

    return estado

if __name__ == "__main__":
    # Joint names
    jnames = ["px", "py", "pz", "ew", "ex", "ey", "ez",
        "FR_HAA", "FR_HFE", "FR_KFE",
        "FL_HAA", "FL_HFE", "FL_KFE",
        "HR_HAA", "HR_HFE", "HR_KFE",
        "HL_HAA", "HL_HFE", "HL_KFE"]

    # Initial configuration (position, quat, qactuated)
    angle = 0.0
    #q0 = np.array([0.1, 0.3, 0.4, np.cos(angle/2.0), 0., np.sin(angle/2.0), 0.,
    #               0.2, -0.2, 0.5, 0.3, -0.5, 0.5,
    #               0., -0.5, 0.5, 0., -0.5, 0.5])
    q0 = np.array([0, 0, 0.3, 1, 0., 0., 0.,
                0., -0.5, 1, 0., -0.5, 1,
                0., 0.5, -1, 0., 0.5, -1])

    # Internal robot representation
    robot = Robot()
    robot.update_config(q0)

    # Initialize the node
    rospy.init_node('test')
    # Loop frequency
    dt = 0.010
    freq = 1.0/dt

    # Initialize kinematic controller
    #weights = [10.0, 10.0, 10.0, 10.0, 1.0]
    weights = [1.0, 1.0, 1.0, 1.0, 1.0]
    lda = 4.0
    lambdas = [lda, lda, lda, lda, lda]
    #lambdas = [10.0, 10.0, 10.0, 10.0, 10.0]
    solver = WQPControllerBase(weights, lambdas, dt)

    # Ball markers
    bmarker1 = BallMarker(color['RED'])
    bmarker2 = BallMarker(color['RED'])
    bmarker3 = BallMarker(color['RED'])
    bmarker4 = BallMarker(color['RED'])

    # Publisher for joint states 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # Creation of a message
    jstate = JointState()
    jstate.name = jnames
    jstate.position = q0

    robot.update_config(q0)
    pfr_des = robot.fkine_fright()[0:3,3]
    pfl_des = robot.fkine_fleft()[0:3,3]
    prr_des = robot.fkine_rright()[0:3,3]
    prl_des = robot.fkine_rleft()[0:3,3]
    #pbase_des = robot.fkine_base()[0:3]
    pbase_des = robot.fkine_base()

    # Change initial configuration
    val = 0.05
    pbase_des[0] += val
    #pbase_des
    pfl_des[0] += val
    #pfl_des[1] = 0.0
    #pfl_des[2] += 0.05

    pfr_des[0] += 2*val
    #pfl_des[1] = 0.0x  
    #pfl_des[2] += 0.05

    prl_des[0] += val
    #pfl_des[1] = 0.0
    #pfl_des[2] += 0.05

    #prr_des[0] += val
    #pfl_des[1] = 0.0
    #pfl_des[2] += 0.05
    valido = True
    valido2 = False
    valido3 = False
    valido4 = False
    valido5 = False
    estado = 2
    rate = rospy.Rate(freq)
    q = np.copy(q0)
    while not rospy.is_shutdown():

        efr, efl, err, erl, ebase = get_errors(pfr_des, pfl_des, prr_des, prl_des, pbase_des)
        Jfr, Jfl, Jrr, Jrl, Jbase = get_jacobians()

        # Get the joint velocity
        dq = solver.get_dq(q, efr, Jfr, efl, Jfl, err, Jrr, erl, Jrl, ebase, Jbase)

        # Integrate rotation
        w = np.dot(Tmat(q), dq[3:7])
        dth = np.linalg.norm(w)
        if abs(dth)>1e-9:
            u = w/dth
            dQ = np.array([np.cos(dth*dt/2.0), u[0]*np.sin(dth*dt/2.0),
                        u[1]*np.sin(dth*dt/2.0), u[2]*np.sin(dth*dt/2.0)])
            Q = quaternionMult(dQ, q[3:7])
            q[3:7] = Q

        # Integrate position and joint configuration
        q[0:3] = q[0:3] + dt*dq[0:3]
        q[7:]  = q[7:] + dt*dq[7:]
        # print 'dq:', dq
        # print 'q:', np.round(q,2), '\n'
        # Update the robot configuration
        robot.update_config(q)

        n_efr = np.linalg.norm(efr)
        n_err = np.linalg.norm(err)
        n_efl = np.linalg.norm(efl)
        n_erl = np.linalg.norm(erl)
        condicion = np.linalg.norm(efr)<1e-2 and np.linalg.norm(err)<1e-2 and np.linalg.norm(efl)<1e-2 and np.linalg.norm(erl)<1e-2

        if condicion and valido5:
            estado = verificar_estado(estado, val) 
            valido5 = False
            valido = True

        if condicion and valido4:
            estado = verificar_estado(estado, val) 
            valido4 = False
            valido5 = True
        
        if condicion and valido3:
            estado = verificar_estado(estado, val) 
            valido3 = False
            valido4 = True   
        
        if condicion and valido2:
            estado = verificar_estado(estado, val) 
            valido2 = False
            valido3 = True     

        if condicion and valido:
            estado = verificar_estado(estado, val) 
            valido = False
            valido2 = True

        
        # Set message
        jstate.header.stamp = rospy.Time.now()
        jstate.position = q
        pub.publish(jstate)
        
        # Show the markers
        bmarker1.xyz(pfr_des)
        bmarker2.xyz(pfl_des)
        bmarker3.xyz(prr_des)
        bmarker4.xyz(prl_des)
        
        rate.sleep()


