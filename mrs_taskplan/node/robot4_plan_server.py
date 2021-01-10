#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Vector3
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import numpy as np
import copy

import actionlib
from mrs_taskplan.msg import TaskAction, TaskGoal, TaskResult, TaskFeedback

from Khepera_server_class import KepRobot


# robot 1
try:
    # geometry_name, agent_radius, agent_id, v_max, initial_position
    currentDir = '/home/i2rlab/catkin_ws/src/mrs_taskplan/node/results/'
    textfile4_d = open(currentDir + 'traj4d.txt', 'w')
    textfile4_a = open(currentDir + 'traj4a.txt', 'w')

    kepherarobot4 = KepRobot('ColumbusCirclePoly', 0.177, 3, 0.30, [3.2, 2.5],\
       '/robot4/mobile_base/commands/velocity', '/robot4/odom', [textfile4_d, textfile4_a], 0.05, [2.38,1.38])

    kepherarobot4.task_server.start()

    rospy.spin()
    textfile4_d.close()
    textfile4_a.close()
except rospy.ROSInterruptException:
    pass
