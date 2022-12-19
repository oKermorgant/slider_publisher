#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateResponse

def set_model_state_callback(req):
    print('Incoming request\n {}'.format(req))
    return SetModelStateResponse()

rospy.init_node('gazebo')
s = rospy.Service('/gazebo/set_model_state', SetModelState, set_model_state_callback)
rospy.spin()


