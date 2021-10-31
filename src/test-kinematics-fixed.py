#!/usr/bin/python
#
# Test of the robot kinematics using a fixed base
#
#    roslaunch freebase-example display-no-ff.launch
#    rosrun freebase-example test-kinematics-fixed.py
#

import rospy
from sensor_msgs.msg import JointState
from markers import *
from robot import Robot

# Joint names
jnames = ["front_right_j1", "front_right_j2", "front_right_j3",
          "front_left_j1", "front_left_j2", "front_left_j3",
          "rear_right_j1", "rear_right_j2", "rear_right_j3",
          "rear_left_j1", "rear_left_j2", "rear_left_j3"]

# Initial configuration (position, quat, qactuated)
q0 = np.array([0., 0.2, 0., 0., 0., 0.,
               0., 0., 0.2, 0., 0., 0.])

# Internal robot representation
r = Robot()
r.update_config(q0)

# Initialize the node
rospy.init_node('test')

# Ball markers
bmarker1 = BallMarker(color['RED'])

# Publisher for joint states 
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
# Creation of a message
jstate = JointState()
jstate.name = jnames
jstate.position = q0[7:]

rate = rospy.Rate(10)
q = np.copy(q0)
while not rospy.is_shutdown():

    jstate.header.stamp = rospy.Time.now()
    pub.publish(jstate)

    # Forward kinematics
    r.update_config(q)
    xfr = r.fkine_fright_base()
    Jfr = r.jacobian_fright()
    bmarker1.position(xfr)
    rate.sleep()



