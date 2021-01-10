#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from mrs_taskplan.msg import Trajectory

PI = 3.1415926

currentDir = '/home/i2rlab/catkin_ws/src/mrs_smp/node/'

class Position_clc():
    def __init__(self, leds):
        self.posf_subscriber = rospy.Subscriber(leds[0], Vector3, self.get_posf)
        self.posb_subscriber = rospy.Subscriber(leds[1], Vector3, self.get_posb)
        self.posl_subscriber = rospy.Subscriber(leds[2], Vector3, self.get_posl)
        self.posr_subscriber = rospy.Subscriber(leds[3], Vector3, self.get_posr)

        self.ledf = Vector3() # Front LED
        self.ledb = Vector3() # Back LED
        self.ledl = Vector3() # Left LED
        self.ledr = Vector3() # Right LED

        self.actualpos6 = Pose()

    ''' Front LED position info '''
    def get_posf(self, pose1):
        self.ledf.x = pose1.x
        self.ledf.y = pose1.y
        self.ledf.z = pose1.z


    ''' Back LED position info '''
    def get_posb(self, pose2):
        self.ledb.x = pose2.x
        self.ledb.y = pose2.y
        self.ledb.z = pose2.z


    ''' Left LED position info '''
    def get_posl(self, pose3):
        self.ledl.x = pose3.x
        self.ledl.y = pose3.y
        self.ledl.z = pose3.z


    ''' Right LED position info '''
    def get_posr(self, pose4):
        self.ledr.x = pose4.x
        self.ledr.y = pose4.y
        self.ledr.z = pose4.z


    ''' Estimate robot position based on 4 LEDs position info '''
    def update_pose(self):
        ledf_flag = True
        ledb_flag = True
        ledl_flag = True
        ledr_flag = True

        if self.ledf.x == 0 and self.ledf.y == 0 and self.ledf.z == 0:
            ledf_flag = False

        if self.ledb.x == 0 and self.ledb.y == 0 and self.ledb.z == 0:
            ledb_flag = False

        if self.ledl.x == 0 and self.ledl.y == 0 and self.ledl.z == 0:
            ledl_flag = False

        if self.ledr.x == 0 and self.ledr.y == 0 and self.ledr.z == 0:
            ledr_flag = False

        if ledf_flag and ledb_flag and ledl_flag and ledr_flag:
            self.actualpos6.y = (self.ledf.x + self.ledb.x + self.ledl.x + self.ledr.x) / 4.0 /1000.0 + 1.45
            self.actualpos6.x = -(self.ledf.y + self.ledb.y + self.ledl.y + self.ledr.y) / 4.0 /1000.0 + 1.83
            diff_fb_y = self.ledf.x - self.ledb.x
            diff_fb_x = -(self.ledf.y - self.ledb.y)
            diff_lr_y = self.ledl.x - self.ledr.x
            diff_lr_x = -(self.ledl.y - self.ledr.y)
            self.actualpos6.theta = (atan2(diff_fb_y, diff_fb_x) + atan2(diff_lr_y, diff_lr_x) - PI/2.0) / 2.0
        elif ledf_flag and ledb_flag and not ledl_flag and not ledr_flag:
            self.actualpos6.y = (self.ledf.x + self.ledb.x) / 2.0 / 1000.0 + 1.45
            self.actualpos6.x = -(self.ledf.y + self.ledb.y) / 2.0 / 1000.0 + 1.83
            diff_fb_y = self.ledf.x - self.ledb.x
            diff_fb_x = -(self.ledf.y - self.ledb.y)
            self.actualpos6.theta = atan2(diff_fb_y, diff_fb_x)
        elif not ledf_flag and not ledb_flag and ledl_flag and ledr_flag:
            self.actualpos6.y = (self.ledl.x + self.ledr.x) / 2.0 / 1000.0 + 1.45
            self.actualpos6.x = -(self.ledl.y + self.ledr.y) / 2.0 / 1000.0 + 1.83
            diff_lr_y = self.ledl.x - self.ledr.x
            diff_lr_x = -(self.ledl.y - self.ledr.y)
            self.actualpos6.theta = atan2(diff_lr_y, diff_lr_x) - PI/2.0


    def deviation_dist(self, actPos_x, actPos_y, desiredPos_x, desiredPos_y):
        return sqrt((actPos_x - desiredPos_x)**2 + (actPos_y - desiredPos_y)**2)


class GetTrajectory():
    def __init__ (self, agent_id):
        self.traj_subcriber = rospy.Subscriber('dtraj_puber'+str(agent_id), Trajectory, self.get_traj)
        self.x_poses = []
        self.y_poses = []

    def get_traj(self, data):
        self.x_poses = data.xlist
        self.y_poses = data.ylist


try:

    rospy.init_node('position_plot', anonymous = True)
    rate = rospy.Rate(20)

    # actual trajectory call back class
    #khepera1_pos_clc = Position_clc(['/LED14', '/LED15', '/LED18', '/LED19'])
    #khepera2_pos_clc = Position_clc(['/LED10', '/LED11', '/LED8', '/LED9'])
    #turtlebot34_pos_clc = Position_clc(['/LED12', '/LED13', '/LED24', '/LED25'])
    #turtlebot37_pos_clc = Position_clc(['/LED17', '/LED16', '/LED2', '/LED3'])

    # desired trajectory call back class
    robot0_traj = GetTrajectory(0)
    robot1_traj = GetTrajectory(1)
    robot2_traj = GetTrajectory(2)
    robot3_traj = GetTrajectory(3)

    # plot background image
    env_img = plt.imread(currentDir+'My_Image2.png')
    plotTraj, ax = plt.subplots()
    plt.subplots_adjust(left=0.0015, bottom=-0.07, right=1.085, top=1.10)
    plotTraj.canvas.toolbar.pack_forget() # Remove status bar (bottom)
    plotTraj.canvas.manager.full_screen_toggle() # toggle fullscreen mode
    env_plot = ax.imshow(env_img, extent=[0, 3.68, 0, 2.9])
#    plt.axis('off')

    ax.set_aspect(1.0, adjustable=None, anchor=None)
    plt.pause(1)

    #ax.plot([0,3.68],[0,2.9],'r')

#      #solution 1
#    while not rospy.is_shutdown():
#        khepera1_pos_clc.update_pose()
#        khepera2_pos_clc.update_pose()
#        turtlebot34_pos_clc.update_pose()
#        turtlebot37_pos_clc.update_pose()

#        ax.plot(khepera1_pos_clc.actualpos6.x, khepera1_pos_clc.actualpos6.y, 'g:',\
#                khepera2_pos_clc.actualpos6.x, khepera2_pos_clc.actualpos6.y, 'ro',\
#                turtlebot34_pos_clc.actualpos6.x, turtlebot34_pos_clc.actualpos6.y, 'bo:',\
#                turtlebot37_pos_clc.actualpos6.x, turtlebot37_pos_clc.actualpos6.y, 'cv')

#        #ax.plot([0,3.68],[2.9,0],'g')
#        plotTraj.canvas.draw()
#        plt.pause(0.50)
#        #fig.canvas.flush_events()
#        #rate.sleep()

    # solution 2
    while not rospy.is_shutdown():

        ax.plot(robot0_traj.x_poses, robot0_traj.y_poses, 'g:',\
                robot1_traj.x_poses, robot1_traj.y_poses, 'ro',\
                robot2_traj.x_poses, robot2_traj.y_poses, 'b+',\
                robot3_traj.x_poses, robot3_traj.y_poses, 'ys-')

        #ax.plot([0,3.68],[2.9,0],'g')
        plotTraj.canvas.draw()
        plt.pause(0.1)
        #plt.clf()


    plotTraj.show()
    rospy.spin()
except rospy.ROSInterruptException:
    pass
