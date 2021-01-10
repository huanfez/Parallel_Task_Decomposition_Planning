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
    textfile2_d = open(currentDir + 'traj2d.txt', 'w')
    textfile2_a = open(currentDir + 'traj2a.txt', 'w')

    kepherarobot2 = KepRobot('ColumbusCirclePoly', 0.177, 1, 0.30, [0.5, 2.5],\
       '/robot2/mobile_base/commands/velocity', '/robot2/odom', [textfile2_d, textfile2_a], 0.05, [])

    kepherarobot2.task_server.start()

    rospy.spin()
    textfile2_d.close()
    textfile2_a.close()
except rospy.ROSInterruptException:
    pass
