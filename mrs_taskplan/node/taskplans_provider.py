#! /usr/bin/env python

import rospy
import actionlib
from mrs_taskplan.msg import PlanindexAction, PlanindexGoal, PlanindexResult, PlanindexFeedback

global stop_flag

stop_flag = False


taskplan_status_0 = PlanindexGoal()
taskplan_status_0.step = 0
taskplan_status_0.group = 0

taskplan_status_1 = PlanindexGoal()
taskplan_status_1.step = 1
taskplan_status_1.group = 0

taskplan_status_2 = PlanindexGoal()
taskplan_status_2.step = 1
taskplan_status_2.group = 1

taskplan_status_mat = [[taskplan_status_0],\
   [taskplan_status_1, taskplan_status_2]]

try:
    rospy.init_node('taskplan_provider_node', anonymous=True)

    taskplan_client0 = actionlib.SimpleActionClient('/taskplan0', PlanindexAction)
    taskplan_client1 = actionlib.SimpleActionClient('/taskplan1', PlanindexAction)
    taskplan_clients = [taskplan_client0, taskplan_client1]

    for taskplan_status_array in taskplan_status_mat:
        #send task plan simutaneously############
        length_array = len(taskplan_status_array)
        for group_index in range(0,length_array):
            taskplan_clients[group_index].wait_for_server()
            taskplan_clients[group_index].send_goal(taskplan_status_array[group_index])

        for group_index in range(0,length_array):
            taskplan_clients[group_index].wait_for_result()

    stop_flag = True

except rospy.ROSInterruptException:
    pass
