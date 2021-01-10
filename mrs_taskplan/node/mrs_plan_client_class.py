#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from mrs_smp.msg import TaskAction, TaskGoal, TaskResult
from mrs_taskplan.msg import PlanindexAction, PlanindexGoal, PlanindexResult, PlanindexFeedback


"""First, construct four clients for all the (four) robots"""
# #####################################################
global keph0_task_client, keph1_task_client, keph2_task_client, keph3_task_client
global kephera_clients


keph0_task_client = actionlib.SimpleActionClient('kephera0task', TaskAction)
keph1_task_client = actionlib.SimpleActionClient('kephera1task', TaskAction)
keph2_task_client = actionlib.SimpleActionClient('kephera2task', TaskAction)
keph3_task_client = actionlib.SimpleActionClient('kephera3task', TaskAction)
# ######################################################
kephera_clients = [keph0_task_client, keph1_task_client, keph2_task_client, keph3_task_client]


class Transition:
    def __init__(self, task, robotidlist):
        self.task = task # task described action
        self.robotidlist = robotidlist # action associated robot list


class Taskplan:
    """Task plan definition, e.g.: [state1, state2, state3] --[transition]-->
    [state1', state2', state3']"""
    def __init__(self, tasklist, transList, robotlist, teamid, eventsubsetList, planid):
        self.tasklist = tasklist # state (task-tuple) List
        self.transList = transList # transition list
        self.robotlist = robotlist # robot index permutation of state (task)
        self.teamid = teamid # current served subtask automaton index
        self.planid = planid
        self.eventsubsetList = eventsubsetList # (List of) event set of all task automata)
        self.stepcount = 0 # index of task plan proceeding


    """task planner client -- send individual task to each robot's
    action server:
        0. Obtain the event and 1st robot id in robot id list (assocaiated
        with event)
        1. If it is a single event --
           1.1 if is parallel executable, find the next robot id, then
               send to 1st and next robot id clients
           1.2 if not, send to 1st robot id client

        2. If it is a cooperative event --
        find cooperative robot id client, then send to 1st and cooperative robot
        id client"""
    def send_goal(self):
        goal = TaskGoal()
        nextgoal = TaskGoal()
        while self.stepcount < len(self.transList): # when tasks exist
            # generate current goal
            current_transition = self.transList[self.stepcount]
            goal.teamid = self.teamid
            goal.task = current_transition.task[0]
            goal.rid = current_transition.robotidlist[0]
            print "task is: " + goal.task

            # deal with the single event
            if len(current_transition.robotidlist) == 1:
                # generate next goal if it exists
                if self.stepcount + 1 < len(self.transList):
                    next_transition = self.transList[self.stepcount + 1]
                    nextgoal.task = next_transition.task[0]
                    nextgoal.teamid = self.teamid
                    if len(next_transition.robotidlist) == 1:
                        nextgoal.rid = next_transition.robotidlist[0]

                # send goals based on the parallel property
                if not self.are_tasks_parallel(goal, nextgoal): # not in parallel
                    kephera_clients[goal.rid].wait_for_server()
                    kephera_clients[goal.rid].send_goal(goal)
                    print "goal id:" + str(goal.rid) + "goal task:" + goal.task
                    kephera_clients[goal.rid].wait_for_result()

                    self.stepcount = self.stepcount + 1
                else: # In parallel
                    kephera_clients[goal.rid].wait_for_server()
                    kephera_clients[goal.rid].send_goal(goal)
                    kephera_clients[nextgoal.rid].wait_for_server()
                    kephera_clients[nextgoal.rid].send_goal(nextgoal)

                    kephera_clients[goal.rid].wait_for_result()
                    kephera_clients[nextgoal.rid].wait_for_result()

                    self.stepcount = self.stepcount + 2
            # deal with the cooperative event
            elif len(current_transition.robotidlist) > 1:
                # generate cooperative goal if it exists
                cooperative_goal = TaskGoal()
                cooperative_goal.task = current_transition.task[0]
                cooperative_goal.rid = current_transition.robotidlist[1]
                cooperative_goal.teamid = self.teamid

                # send goals based on the cooperative property
                kephera_clients[goal.rid].wait_for_server()
                kephera_clients[goal.rid].send_goal(goal)
                kephera_clients[cooperative_goal.rid].wait_for_server()
                kephera_clients[cooperative_goal.rid].send_goal(cooperative_goal)

                kephera_clients[goal.rid].wait_for_result()
                kephera_clients[cooperative_goal.rid].wait_for_result()

                self.stepcount = self.stepcount + 1


    def are_tasks_parallel(self, goal1, goal2):
        """two goals are parallel executable if goal1 and goal2 are
        in different event subsets"""
        taskset = {goal1.task, goal2.task}

        for eventsubset in self.eventsubsetList:
            if taskset.issubset(eventsubset):
                return False

        if goal1.rid != goal2.rid:
            return True
        else:
            return False
