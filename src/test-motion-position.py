#!/usr/bin/python
#
# Test of the robot motion using the floating base
#
#    roslaunch freebase-example display.launch
#    rosrun freebase-example test-motion-position.py
#

import rospy
from sensor_msgs.msg import JointState
from markers import *
from robot_solo import Robot
from kcontroller import WQPController
from utils import Tmat, quaternionMult
import numpy as np

pi = np.pi; sin = np.sin; cos = np.cos

# Joint names
jnames = ["px", "py", "pz", "ew", "ex", "ey", "ez",
    "FR_HAA", "FR_HFE", "FR_KFE",
    "FL_HAA", "FL_HFE", "FL_KFE",
    "HR_HAA", "HR_HFE", "HR_KFE",
    "HL_HAA", "HL_HFE", "HL_KFE"]

# Initial configuration (position, quat, qactuated)
angle = 0.0
#q0 = np.array([0.3, 0.4, 0.38, sin(pi/6), 0., cos(pi/6), 0.,
#               0.2, 0., 0.2, 0., 0.2, 0.5,
#               0., 0.5, 0.3, 0., 0.2, 0.])

q0 = np.array([0.3, 0.4, 0.38, 1., 0., 0., 0.,
               0., 0., 0., 0., 0., 0.,
               0., 0., 0., 0., 0., 0.])

# Internal robot representation
robot = Robot()
robot.update_config(q0)

# Initialize the node
rospy.init_node('test')
# Loop frequency
dt = 0.010
freq = 1.0/dt

# Initialize kinematic controller
weights = [1.0, 1.0, 1.0, 1.0]
#lambdas = [0.01, 0.01, 0.01, 0.01]
#lambdas = [0.1, 0.1, 0.1, 0.1]
lambdas = [0.5, 0.5, 0.5, 0.5]
#lambdas = [1.0, 1.0, 1.0, 1.0]
#lambdas = [10.0, 10.0, 10.0, 10.0]
solver = WQPController(weights, lambdas, dt)

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
#pfr_des = robot.fkine_fright()[0:3,3]
#pfl_des = robot.fkine_fleft()[0:3,3]
#prr_des = robot.fkine_rright()[0:3,3]
#prl_des = robot.fkine_rleft()[0:3,3]
pfr_des = [0.4, 0.4, 0.7]
pfl_des = [0.4, 0.6, 0.7]
prr_des = [-0.4, 0.4, 0.7]
prl_des = [-0.4, 0.6, 0.7]
# Change initial configuration
#pfr_des[2] = 0.02
pfr_des[1] += 0.1

# Create logs
fpfr_des = open("/tmp/pfr_des.txt", "w")
fq = open("/tmp/q.txt", "w")
fpfr = open("/tmp/pfr.txt", "w")

rate = rospy.Rate(freq)
q = np.copy(q0)
# Initial time
t0 = rospy.get_time()
while not rospy.is_shutdown():
    # Current time
    t = np.round(rospy.get_time() - t0, 3)
    print t
    # Logs
    fpfr_des.write(str(t)+" "+str(pfr_des[0])+" "+str(pfr_des[1])+" "+str(pfr_des[2])+"\n")
    fq.write(str(t)+" ")
    for j in range(19):
        fq.write(str(q[j])+" ")
    fq.write("\n")
    pfr = robot.fkine_fright()[0:3,3]
    fpfr.write(str(t)+" "+str(pfr[0])+" "+str(pfr[1])+" "+str(pfr[2])+"\n")
    
    # Errors
    efr = robot.error_position_fright(pfr_des)
    efl = robot.error_position_fleft(pfl_des)
    err = robot.error_position_rright(prr_des)
    erl = robot.error_position_rleft(prl_des)
    # Task Jacobians
    Jfr = robot.taskj_position_fright()
    Jfl = robot.taskj_position_fleft()
    Jrr = robot.taskj_position_rright()
    Jrl = robot.taskj_position_rleft()
    
    # Get the joint velocity
    dq = solver.get_dq(q, efr, Jfr, efl, Jfl, err, Jrr, erl, Jrl)

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
    if (False):
        print 'dq:', dq
        print 'q:', np.round(q,2), '\n'
    # Update the robot configuration
    robot.update_config(q)

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

# Close logs
fpfr_des.close()
fq.close()
fpfr.close()


