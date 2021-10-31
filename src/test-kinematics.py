#!/usr/bin/python
#
# Test of the robot kinematics using the floating base
#
#    roslaunch freebase-example display.launch
#    rosrun freebase-example test-kinematics.py
#

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from markers import *
from robot_solo import Robot

pi = np.pi; sin = np.sin; cos = np.cos

# Joint names
jnames = ["px", "py", "pz", "ew", "ex", "ey", "ez",
    "FR_HAA", "FR_HFE", "FR_KFE",
    "FL_HAA", "FL_HFE", "FL_KFE",
    "HR_HAA", "HR_HFE", "HR_KFE",
    "HL_HAA", "HL_HFE", "HL_KFE"]

# Initial configuration (position, quat, qactuated)
q0 = np.array([0, 0, 0.3, 1, 0., 0., 0.,
              0., 0.5, -1, 0., 0.5, -1,
              0., 0.5, -1, 0., 0.5, -1])

#q0 = np.array([0, 0, 0.3, 1, 0., 0., 0.,
#               0., 0., 0., 0., 0., 0.,
#               0., 0., 0., 0., 0., 0.])

#q0 = np.array([0.5, 0.55, 0.3, 1, 0., 0., 0.,
#               0., 0.5, -1, 0., -1, 1.5,
#               0., 0.5, -1, 0., 0.5, -1])

# Internal robot representation
robot = Robot()
robot.update_config(q0)

# Initialize the node
rospy.init_node('test')

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

rate = rospy.Rate(10)
q = np.copy(q0)
while not rospy.is_shutdown():

    jstate.header.stamp = rospy.Time.now()
    pub.publish(jstate)

    # Forward kinematics
    robot.update_config(q)
    # To test for different legs use:
    #    fkine_fleft, fkine_fright, fkine_rleft, fkine_rright
    # Front right foot
    xfr = robot.fkine_fright()
    # Front left foot
    xfl = robot.fkine_fleft()
    # Rear right foot
    xrr = robot.fkine_rright()
    # Rear left foot
    xrl = robot.fkine_rleft()
    # Show the markers
    bmarker1.position(xfr)
    bmarker2.position(xfl)
    bmarker3.position(xrr)
    bmarker4.position(xrl)
    
    rate.sleep()



