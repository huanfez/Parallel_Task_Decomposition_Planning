#! /usr/bin/env python

import rospy
import actionlib
from mrs_smp.msg import TaskAction, TaskGoal, TaskResult
from mrs_taskplan.msg import PlanindexAction, PlanindexGoal, PlanindexResult, PlanindexFeedback
from mrs_plan_client_class import Transition, Taskplan
global keph0_task_client, keph1_task_client, keph2_task_client, keph3_task_client
global kephera_clients


def get_taskplan2(goal):
    # 0.1 feedback info
    feedback = PlanindexFeedback()
    feedback.group = goal.group

    # 0.2 Receiving task and goal info
    result = PlanindexResult()
    result.step = goal.step
    result.group = goal.group

    for taskplan in taskplans:
        if taskplan.planid == goal.step:
            taskplan.send_goal()

    taskplan_server2.set_succeeded(result, "Task has been settled")


# taskplan2: store a sequence of (event, rid): dictionary
tasklist2_1 = ['d', 'e', 'g']
transList2_1 = [Transition('d',[3]), Transition('e',[3]), Transition('g',[3])]
robotlist2_1 = 3
subeventList2_1 = [['d', 'e', 'g']]
taskplan2 = Taskplan(tasklist2_1, transList2_1, robotlist2_1, 1, subeventList2_1, 1)
taskplans = [taskplan2]

try:
    rospy.init_node('taskplanning2', anonymous=True)

    taskplan_server2 = actionlib.SimpleActionServer('/taskplan1', PlanindexAction, get_taskplan2, False)
    taskplan_server2.start()

    rospy.spin()
except rospy.ROSInterruptException:
        pass
