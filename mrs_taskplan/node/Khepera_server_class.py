#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Vector3
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import numpy as np
import copy

import actionlib
from mrs_smp.msg import TaskAction, TaskGoal, TaskResult, TaskFeedback

from mrs_taskplan.msg import Trajectory

import DiscretePlanner
import ContinousPlanner

from DiscretePlanner import HighLevelPlanner
from ContinousPlanner import Robot

PI = 3.1415927

class KepRobot():
    def __init__(self, geometry_name, agent_radius, agent_id, \
       v_max, initial_position, vel_puber_name, pos_suber_name, textfiles, \
       time_step=0.05, temp_obstacle = []):

        node_name = 'robot'+ str(agent_id) + '_server'
        rospy.init_node(node_name, anonymous=True)

        server_name = 'kephera' + str(agent_id) + 'task'
        self.task_server = actionlib.SimpleActionServer(server_name, TaskAction, self.get_taskstate, False)

        self.hplanner = HighLevelPlanner(geometry_name, initial_position, agent_id, temp_obstacle)

        self.geometry_name = geometry_name
        self.agent_radius = agent_radius
        self.agent_id = agent_id
        self.v_max = v_max
        self.time_step = time_step
        self.initial_position = initial_position

        #'/robot2/mobile_base/commands/velocity', '/robot2/odom'
        self.vel_publisher = rospy.Publisher(vel_puber_name, Twist, queue_size=10)
        self.pos_subcriber = rospy.Subscriber(pos_suber_name, Odometry, self.get_odom_pos)

        # trajectory publisher
        self.dTrajectory_publisher = rospy.Publisher('dtraj_puber'+str(agent_id), Trajectory, queue_size=10)

        self.twod_pose = initial_position
        self.actualpos6 = Pose() # actual position in 6 dimensions
        self.taskpos6 = Pose() # task coordinates
        self.trajectory = []
        self.vel_list = []

        self.rate = rospy.Rate(20)
        self.task_pos_dict = {'a': [0.12,1.38], 'b': [1.28,2.76], 'c': [1.28,0.2], 'd': [2.58,2.76],\
           'e': [2.58,1.38], 'f': [2.58,0.2], 'g': [3.48,1.38]} #'c': [1.28,0.12]
        self.textfile_d = textfiles[0]
        self.textfile_a = textfiles[1]


    """ Call back function: based on if the task has been reached,
    get newly assigned task"""
    def get_taskstate(self, goal):

        desiredTraj = Trajectory()

        # 0.1 feedback info
        feedback = TaskFeedback()
        feedback.task = goal.task
        feedback.rid = goal.rid
        feedback.state = False

        # 0.2 Receiving task and goal info
        result = TaskResult()
        result.task = goal.task
        result.rid = goal.rid
        result.teamid = goal.teamid
        result.state = False

        replan = False
        #self.cplanner.is_finished = False

        # 1.1 discrete planner
        self.hplanner.goals = np.array(self.task2goal_pose(goal.task))
        self.hplanner.initial_position = self.twod_pose
        self.hplanner.discrete_planner()
        print 'receive goal task: ' + goal.task + \
           'Translated goal num: ' + str(self.task2goal_state(goal.task)) + \
           'goal triangle num: ' + str(self.hplanner.goal_triangle_num)

        # 1.2 continous planner
        self.cplanner = Robot(self.geometry_name, self.agent_radius, \
           self.agent_id, self.v_max, self.twod_pose, self.time_step)

        # 1.3 generate trajectory and velocity
        #self.update_pose()
        self.trajectory = [[self.twod_pose[0], self.twod_pose[1], self.actualpos6.theta]]
        self.vel_list = []
        #previous_twod_vel = np.array([0.0, 0.0])
        while not self.cplanner.is_finished:
            # generate the two dimensional velocity
            current_twod_vel = self.cplanner.move(self.twod_pose)
            abs_current_twod_vel = sqrt(current_twod_vel[0]**2 + current_twod_vel[1]**2)
            if abs_current_twod_vel < 0.001:
                current_twod_vel = current_twod_vel / abs_current_twod_vel * 0.02
                abs_current_twod_vel = sqrt(current_twod_vel[0]**2 + current_twod_vel[1]**2)

            # calaulate and record the reference position
            pose_xy = np.array(self.twod_pose) + 0.05 * current_twod_vel
            self.twod_pose = [pose_xy[0] , pose_xy[1]]
            roll = atan2(current_twod_vel[1], current_twod_vel[0])
            self.trajectory.append([self.twod_pose[0], self.twod_pose[1], roll])
            self.textfile_d.write(str(self.twod_pose[0]) + ' ' + str(self.twod_pose[1]) + '\n')

            # calaulate and record the reference velocity
            roll_vel = (self.trajectory[-1][2] - self.trajectory[-2][2]) / 0.05
            self.vel_list.append([current_twod_vel[0], current_twod_vel[1], abs_current_twod_vel, roll_vel])
            print current_twod_vel

        desiredTraj.xlist = [item[0] for item in self.trajectory]
        desiredTraj.ylist = [item[1] for item in self.trajectory]

        self.dTrajectory_publisher.publish(desiredTraj)

        """ use robot position subscriber to get autual position info self.actualpos6
        if not reaching, feedback some information about task
        else tell the taskplan node: the task has been completed"""
        print('current position is: ', self.actualpos6.x, self.actualpos6.y, self.actualpos6.theta)
        print len(self.vel_list)

        # 2.1 Rotating a angle
        initial_direction = atan2(self.vel_list[0][1], self.vel_list[0][0])
        self.rotate(initial_direction)

        # 2.2 following a trajectory
        step = 0
        while not result.state:
            self.task_server.publish_feedback(feedback)

            """when potential collision happens, replan both discrete and co
            ntinous path"""
            if self.cplanner.replan_request == True:
                print 'replanned'

            # Generate and send velocity info.
            desired_pose = self.trajectory[step+1]
            desired_vel = self.vel_list[step]

            #self.update_pose()
            err_px = desired_pose[0] - self.actualpos6.x
            err_py = desired_pose[1] - self.actualpos6.y
            err_theta = desired_pose[2] - self.actualpos6.theta
            errors = np.array([[err_px], [err_py], [err_theta]])

            #while (errors[0][0]**2 + errors[1][0]**2) > 0.001 or errors[3][0] > 0.1:
            ctrl_velocity = self.ctrl2_vel(desired_pose, desired_vel)
            #self.textfile_a.write(str(self.actualpos6.x) + ' ' + str(self.actualpos6.y) + '\n')

            self.vel_publisher.publish(ctrl_velocity)
            self.rate.sleep()
            #self.update_pose()
            step += 1

            # result info
            if step > len(self.vel_list) - 1:
                print "succeed"
                result.state = True

        # 3. Last step for stoping the motion
        ctrl_velocity = Twist()
        self.vel_publisher.publish(ctrl_velocity)
        self.task_server.set_succeeded(result, "Task has been settled")


    """use dictionary to find the position value of event:label"""
    def task2goal_pose(self, task):
        return self.task_pos_dict[task]


    """use dictionary to find the state value of event:label"""
    def task2goal_state(self, task):
        goal_pose = self.task_pos_dict[task]
        print "goal pose: " + str(goal_pose)
        return self.hplanner.pose2discrete_state(np.array(goal_pose))


    # Get the simulation position
    def get_odom_pos(self, data):
        """Call back function of subscriber: get robot current position"""
        self.actualpos6.x = data.pose.pose.position.x + self.initial_position[0] # get actual position $actualpos6$
        self.actualpos6.y = data.pose.pose.position.y + self.initial_position[1]
        # yaw (z-axis rotation)
        siny_cosp = +2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z +\
           data.pose.pose.orientation.x * data.pose.pose.orientation.y);
        cosy_cosp = +1.0 - 2.0 * (data.pose.pose.orientation.y**2 + data.pose.pose.orientation.z**2);
        self.actualpos6.theta = atan2(siny_cosp, cosy_cosp)


    # Use controller to generate the velocity
    def ctrl1_vel(self, desired_position, desired_velocity):
        cmd_vel = Twist()

        #self.update_pose()
        err_px = desired_position[0] - self.actualpos6.x
        err_py = desired_position[1] - self.actualpos6.y
        err_theta = desired_position[2] - self.actualpos6.theta
        errors = np.array([[err_px], [err_py], [err_theta]])

        matrix = np.array([[np.cos(self.actualpos6.theta), np.sin(self.actualpos6.theta), 0],\
                           [-np.sin(self.actualpos6.theta), np.cos(self.actualpos6.theta), 0],\
                           [0, 0, 1]])
        error_array_ = matrix.dot(errors)
        #print error_array_

        # coefficients
        b = 0.01
        epsilon = 0.03
        a = sqrt(desired_velocity[3]**2 + b * (desired_velocity[2]**2))

        K_x = 2 * epsilon * a
        K_y = b * desired_velocity[2]
        K_the = K_x

        cmd_vel.linear.x = desired_velocity[2] * np.cos(error_array_[2][0]) + K_x * error_array_[0][0]
        cmd_vel.angular.z = desired_velocity[3]+ (K_y * error_array_[1][0] * np.sin(error_array_[2][0]) / error_array_[2][0]) + K_the * error_array_[2][0]
        #print 'velocity:' + str(cmd_vel.linear.x) + ',' + str(cmd_vel.angular.z) + ',' + str(theta1 - self.actualpos6.theta)
        return cmd_vel


    # Use controller to generate the velocity
    def ctrl2_vel(self, desired_position, desired_velocity):
        cmd_vel = Twist()
#        kx = 2.2 # 2.0 0.50 for robot 192.168.1.11
#        ky = 2.2 # 2.0 0.50
#        b = 0.30 # 0.1 0.30

#        kx = 2.0 # 2.0 for robot 192.168.1.12
#        ky = 2.0 # 2.0
#        b = 0.35 # 0.1

        kx = 2.0 # 2.0 for robot 192.168.1.13
        ky = 2.0 # 2.0
        b = 0.1 # 0.1

#        kx = 2.2 # 2.0 0.50 for robot 192.168.1.10
#        ky = 2.2 # 2.0 0.50
#        b = 0.28 # 0.1 0.30

        #self.update_pose()
        err_px = desired_position[0] - self.actualpos6.x
        err_py = desired_position[1] - self.actualpos6.y
        err_theta = desired_position[2] - self.actualpos6.theta
        errors = np.array([[err_px], [err_py], [err_theta]])

        desired_velocity[0] += kx * err_px
        desired_velocity[1] += ky * err_py
        cmd_vel.linear.x = desired_velocity[0] * np.cos(self.actualpos6.theta) + desired_velocity[1] * np.sin(self.actualpos6.theta)
        cmd_vel.angular.z = 1.0 / b * (desired_velocity[1] * np.cos(self.actualpos6.theta) - desired_velocity[0] * np.sin(self.actualpos6.theta))
        #print 'velocity:' + str(cmd_vel.linear.x) + ',' + str(cmd_vel.angular.z) + ',' + str(theta1 - self.actualpos6.theta)
        return cmd_vel


    ''' Rotate first at the beginning of a task to adjust the angle'''
    def rotate(self, desired_angle):
        rot_vel = Twist()
        ka_z = 0.5

        #self.update_pose()
        err_anglez = desired_angle - self.actualpos6.theta

        while abs(err_anglez) > 0.05:
            #print 'angle error: ', err_anglez, desired_angle, self.actualpos6.x, self.actualpos6.y, self.actualpos6.theta
            rot_vel.angular.z = ka_z * err_anglez

            if rot_vel.angular.z > 0.6:
                rot_vel.angular.z = 0.6
            elif rot_vel.angular.z < -0.6:
                rot_vel.angular.z = -0.6

            if 0 < rot_vel.angular.z < 0.08:
                rot_vel.angular.z = 0.08
            elif -0.08 < rot_vel.angular.z < 0:
                 rot_vel.angular.z = -0.08

            self.vel_publisher.publish(rot_vel)
            #print 'rotation velocity: ', rot_vel.angular.z

            self.rate.sleep()
            #self.update_pose()
            #print 'position: ', self.actualpos6.x, self.actualpos6.y, self.actualpos6.theta
            err_anglez = desired_angle - self.actualpos6.theta


