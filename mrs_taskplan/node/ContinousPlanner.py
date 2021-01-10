#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
from interval import interval
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from operator import add
import JarvisMarch
import os.path
import DiscretePlanner

#currentDir = os.getcwd()
currentDir = '/home/i2rlab/catkin_ws/src/mrs_taskplan/node/'
geometryDir = os.path.join(currentDir, 'Geometry')


class Vertice(object):
    def __init__(self, geometry_name):
        with open(geometryDir +'/{}.1.node'.format(geometry_name)) as vpos:
            vertices_pos = vpos.readlines()
        vpos.close()
        del (vertices_pos[-1])
        self.vertices_coords_temp = []
        for i in range(1, len(vertices_pos)):
            vertice = vertices_pos[i].strip().split(' ')
            vertice = [float(i) for i in vertice if bool(i)]
            self.vertices_coords_temp.append(vertice[1:3])

    def position(self, vertice):
        return self.vertices_coords_temp[int(vertice - 1)]


class Triangle(Vertice):
    def __init__(self, geometry_name):
        super(Triangle, self).__init__(geometry_name)
        self.geometry_name = geometry_name
        self.number_of_triangles = 0
        self.triangle_nodes = []  # triangles = [[#ver.1, #ver.2, #ver.3],...,[#ver.1, #ver.2, #ver.3]]
        with open(geometryDir + '/{}.1.ele'.format(geometry_name)) as elements:
            triangle_vertice_list = elements.readlines()
        elements.close()
        del (triangle_vertice_list[-1])
        # Triangles numbers and Vertices numbers
        for i in range(1, len(triangle_vertice_list)):
            triangle_nodes = triangle_vertice_list[i].strip().split(' ')
            triangle_nodes = [float(i) for i in triangle_nodes if bool(i)]
            self.triangle_nodes.append(triangle_nodes[1:4])
        self.number_of_triangles = len(self.triangle_nodes)

        with open(geometryDir + '/{}.1.neigh'.format(geometry_name)) as neighbors_text:
            neighbors = neighbors_text.readlines()
        neighbors_text.close()
        del (neighbors[-1])
        self.neighbors_list_temp = []
        for i in range(1, len(neighbors)):
            neighbor_num = neighbors[i].strip().split(' ')
            neighbor_num = filter(None, neighbor_num)
            self.neighbors_list_temp.append(neighbor_num)

    def vertices(self, tri):
        return self.triangle_nodes[tri - 1]

    def vertices_poses(self, tri):
        return [self.position(value) for value in self.vertices(tri)]

    def neighbors_list(self, tri):
        return [int(value) for value in self.neighbors_list_temp[tri - 1] if value != '-1'][1:]

    def common_vertices(self, tri1, tri2):
        return [value for value in self.triangle_nodes[tri1 - 1] if value in self.triangle_nodes[tri2 - 1]]


class State(Triangle):
    def __init__(self, geometry_name, agent_num):
        super(State, self).__init__(geometry_name)
        self.state_list = ''
        with open(currentDir + 'state_order_{}_agent{}.txt'.format(geometry_name, agent_num)) as state:
            self.state_list = state.readlines()
        state.close()
        self.state_total_number = len(self.state_list)
        self.last_state = self.state_list[-1]
        self.gs = []
        self.global_vertex_index = []
        self.vector_field = dict()
        self.last_valid_patch = []
        self.patch_length = []

    def triangle_number(self, state_number):
        return self.state_list[state_number - 1]

    def vertices(self, state_num):
        return super(State, self).vertices(int(self.triangle_number(state_num)))

    def common_vertices(self, state1, state2):
        return super(State, self).common_vertices(int(self.triangle_number(state1)), int(self.triangle_number(state2)))

    def v1v2v3(self, state_number):
        if state_number == self.state_total_number:
            v1 = [value for value in self.vertices(state_number) if value not in
                  self.common_vertices(state_number, state_number - 1)][0]
            v2 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) + 1, 3)]
            v3 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) - 1, 3)]
        else:
            v1 = [value for value in self.vertices(state_number) if value not in
                  self.common_vertices(state_number, state_number + 1)][0]
            v2 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) + 1, 3)]
            v3 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) - 1, 3)]
        return [v1, v2, v3]

    def initial_gs(self):
        v_pos = [[0., 0.], [0., 0.], [0., 0.]]
        for state_iter in range(1, self.state_total_number + 1):
            for i in range(0, 3):
                v_pos[i] = self.position(self.v1v2v3(state_iter)[i])
            v12_vector = np.array(v_pos[1]) - np.array(v_pos[0])
            v12_angle = math.degrees(math.atan2(v12_vector[1], v12_vector[0]))
            v13_vector = np.array(v_pos[2]) - np.array(v_pos[0])
            v13_angle = math.degrees(math.atan2(v13_vector[1], v13_vector[0]))
            v23_vector = np.array(v_pos[2]) - np.array(v_pos[1])
            v23_angle = math.degrees(math.atan2(v23_vector[1], v23_vector[0]))
            v32_vector = np.array(v_pos[1]) - np.array(v_pos[2])
            v32_angle = math.degrees(math.atan2(v32_vector[1], v32_vector[0]))
            if v12_angle > v13_angle:
                int1 = interval([v12_angle, 180.], [-180., v13_angle])
            else:
                int1 = interval([v12_angle, v13_angle])

            if v12_angle > v23_angle:
                int2 = interval([v12_angle, 180.], [-180., v23_angle])
            else:
                int2 = interval([v12_angle, v23_angle])

            if v32_angle > v13_angle:
                int3 = interval([v32_angle, 180.], [-180., v13_angle])
            else:
                int3 = interval([v32_angle, v13_angle])
            self.gs.append([int1, int2, int3])

    def last_triangle_gs(self, last_tri_vs):
        self.last_tri_gs = []
        verts = [[0., 0.], [0., 0.], [0., 0.]]
        for i in range(0, 3):
            verts[i] = self.position(last_tri_vs[i])

        for i in range(0, 3):
            vector_right = np.array(verts[np.mod(i + 1, 3)] - np.array(verts[np.mod(i, 3)]))
            vector_left = np.array(verts[np.mod(i - 1, 3)] - np.array(verts[np.mod(i, 3)]))
            angle_right = math.degrees(math.atan2(vector_right[1], vector_right[0]))
            angle_left = math.degrees(math.atan2(vector_left[1], vector_left[0]))
            if angle_right > angle_left:
                self.last_tri_gs.append(interval([angle_right, 180.], [-180., angle_left]))
            else:
                self.last_tri_gs.append(interval([angle_right, angle_left]))
        return self.last_tri_gs

    def vertex_ijk(self):
        self.global_vertex_index.append(self.v1v2v3(1)[0])
        self.global_vertex_index.append(self.v1v2v3(1)[1])
        self.global_vertex_index.append(self.v1v2v3(1)[2])
        for j in range(1, self.state_total_number):
            self.global_vertex_index.append(list(set(self.v1v2v3(j + 1)) - set(self.common_vertices(j, j + 1)))[0])

    def patches(self):
        self.patch_list = []
        self.patch_state_list = []
        state_iter = 1
        pre_length = 0
        while state_iter <= self.state_total_number:
            self.patch_state_list.append(state_iter)
            new_g1 = new_g2 = new_g3 = interval([0,1])

            try:
                new_g1 = self.vector_field[self.v1v2v3(state_iter)[0]] & self.gs[state_iter - 1][0]
            except:
                pass
            try:
                new_g2 = self.vector_field[self.v1v2v3(state_iter)[1]] & self.gs[state_iter - 1][1]
            except:
                pass
            try:
                new_g3 = self.vector_field[self.v1v2v3(state_iter)[2]] & self.gs[state_iter - 1][2]
            except:
                pass

            if (new_g1 == interval() or new_g2 == interval() or new_g3 == interval()):
                for i in range(0, 3):
                    # if self.vector_field.has_key(self.v1v2v3(state_iter)[i]):  # if the vertice has been seen before
                    if self.v1v2v3(state_iter)[i] in self.vector_field:
                        self.vector_field[self.v1v2v3(state_iter)[i]] = self.vector_field[self.v1v2v3(state_iter)[i]] & self.last_triangle_gs(self.v1v2v3(state_iter))[i]
                    else:  # else if the vertice has not been seen yet
                        self.vector_field[self.v1v2v3(state_iter)[i]] = self.last_triangle_gs(self.v1v2v3(state_iter))[i]
                self.patch_list.append(self.vector_field)
                self.vector_field = dict()
                # print('patches', self.patch_list)
                self.patch_length.append(len(self.patch_list[-1]) - 2)
                pre_length = self.patch_length[-1]
                state_iter = state_iter - 1
            else:
                for i in range(0, 3):
                    if self.v1v2v3(state_iter)[i] in self.vector_field: # if the vertice has been seen before
                        self.vector_field[self.v1v2v3(state_iter)[i]] = self.vector_field[self.v1v2v3(state_iter)[i]] & self.gs[state_iter - 1][i]
                    else:  # else if the vertice has not been seen yet
                        self.vector_field[self.v1v2v3(state_iter)[i]] = self.gs[state_iter - 1][i]

            if state_iter == self.state_total_number:
                self.patch_list.append(self.vector_field)
                self.patch_length.append(state_iter - pre_length + 1)
                break

            state_iter = state_iter + 1

    def vector_field_point(self, point, patch_num, state_num, v_max):
        last = False
        last_patches_total_length = 0
        for i in range(0, patch_num):
            last_patches_total_length += len(self.patch_list[i]) - 2
        local_state = state_num + patch_num - 1 - last_patches_total_length
        g1_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[0]].midpoint
        g2_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[1]].midpoint
        g3_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[2]].midpoint
        g1 = np.array([np.cos(np.radians(g1_angle)[0][0]), np.sin(np.radians(g1_angle)[0][0])])
        g2 = np.array([np.cos(np.radians(g2_angle)[0][0]), np.sin(np.radians(g2_angle)[0][0])])
        g3 = np.array([np.cos(np.radians(g3_angle)[0][0]), np.sin(np.radians(g3_angle)[0][0])])
        v1 = self.position(self.v1v2v3(state_num)[0])
        v2 = self.position(self.v1v2v3(state_num)[1])
        v3 = self.position(self.v1v2v3(state_num)[2])
        Glocal = np.matrix([g1, g2, g3]).transpose()
        W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
        inv_W = np.matrix(W) ** -1
        GW = Glocal * inv_W
        [x, y] = [value for value in point]
        velocity_at_point = np.array(GW * [[x], [y], [1]]).transpose()[0] * v_max
        vertice1_pos = self.position(self.v1v2v3(state_num)[0])
        if len(self.patch_list[patch_num - 1]) == local_state:
            last = True
        return [velocity_at_point, last, vertice1_pos]

    def gConstraints(self, point_xy, patch_num, state_num, v_max):
        g_6points = []
        for i in range(0, 3):
            default_gs = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[i]]

            if len(default_gs) == 2:
                g_6points.append([[np.cos(math.radians(default_gs[0][1])), np.sin(math.radians(default_gs[0][1]))], [np.cos(math.radians(default_gs[1][0])), np.sin(math.radians(
                    default_gs[1][0]))]])
            else:
                g_6points.append([[np.cos(math.radians(default_gs[0][0])), np.sin(math.radians(default_gs[0][0]))], [np.cos(math.radians(default_gs[0][1])), np.sin(math.radians(
                    default_gs[0][1]))]])

        v = [self.position(self.v1v2v3(self.current_state)[0]), self.position(self.v1v2v3(self.current_state)[1]), self.position(self.v1v2v3(self.current_state)[2])]
        W = [[v[0][0], v[1][0], v[2][0]], [v[0][1], v[1][1], v[2][1]], [1.0, 1.0, 1.0]]
        inv_W = np.matrix(W) ** -1
        x, y = point_xy[0], point_xy[1]
        lambdas = np.dot(v_max,np.array(inv_W * [[x],[y], [1]]))  # .transpose()
        g_total_333 = [[[0., 0.], g_6points[0][0], g_6points[0][1]], [[0., 0.], g_6points[1][0], g_6points[1][1]], [[0., 0.], g_6points[2][0], g_6points[2][1]]]
        t1 = lambdas[0] * g_total_333[0]
        t2 = lambdas[1] * g_total_333[1]
        t3 = lambdas[2] * g_total_333[2]
        points = list()
        it = 0
        for i in t1:
            for j in t2:
                for k in t3:
                    it += 1
                    points.append(map(add, map(add, i, j), k))
        try:
            JarvisMarch.main(np.array(points))
        except ValueError:
            L = JarvisMarch.main(np.array(points))
        L = JarvisMarch.main(np.array(points))
        return L

    def plot_triangles(self):
        fig = plt.figure(1)
        ax = plt.axes()

        for i in range(0, self.number_of_triangles):
            for k in range(0, 3):
                ax = plt.plot([self.position(self.triangle_nodes[i][np.mod(k + 1, 3)])[1], self.position(self.triangle_nodes[i][k])[1]],
                         [-self.position(self.triangle_nodes[i][np.mod(k + 1, 3)])[0], -self.position(self.triangle_nodes[i][k])[0]], 'k')
        thismanager = plt.get_current_fig_manager()
        thatmanager = plt.gcf()
        plt.savefig('triangles.png')
        plt.show()

    def plot_vf_one_triangle(self, state_num = 1, patch_num = 1):
        g1_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[0]].midpoint
        g2_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[1]].midpoint
        g3_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[2]].midpoint
        g1 = np.array([np.cos(np.radians(g1_angle)[0][0]), np.sin(np.radians(g1_angle)[0][0])])
        g2 = np.array([np.cos(np.radians(g2_angle)[0][0]), np.sin(np.radians(g2_angle)[0][0])])
        g3 = np.array([np.cos(np.radians(g3_angle)[0][0]), np.sin(np.radians(g3_angle)[0][0])])
        v1 = self.position(self.v1v2v3(state_num)[0])
        v2 = self.position(self.v1v2v3(state_num)[1])
        v3 = self.position(self.v1v2v3(state_num)[2])
        Glocal = np.matrix([g1, g2, g3]).transpose()
        W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
        inv_W = np.matrix(W) ** -1
        GW = Glocal * inv_W
        for l1 in np.linspace(0, 1, 10):
            for l2 in np.linspace(0, 1. - l1, int(10 * (1 - l1))):
                l3 = 1 - l1 - l2
                [x, y] = l1 * np.array(v1) + l2 * np.array(v2) + l3 * np.array(v3)
                v = np.array(GW * [[x], [y], [1]]).transpose()[0]
                plt.quiver(x, y, v[0], v[1], scale = .02, scale_units = 'xy', width = 0.001)

    def plot_patch_vf(self, single_patch = True, patch_num = 1):
        if single_patch:
            if patch_num == 1:
                # patch_state_length = len(self.patch_list[patch_num - 1]) - 2

                if len(self.patch_length) == 0:
                    self.patch_length.append(self.state_total_number)

                for i in range(1, self.patch_length[0] + 1):
                    print(i, '*')
                    self.plot_vf_one_triangle(i, patch_num)

            else:
                patch_length = len(self.patch_list[patch_num - 1]) - 2
                patch_length_pre = 0

                for i in range(1, patch_num):
                    patch_length_pre += len(self.patch_list[i - 1]) - 2

                for i in range(patch_length_pre, min([patch_length_pre + patch_length, self.state_total_number + 1])):
                    self.plot_vf_one_triangle(i, patch_num)
        else:
            for patch_num in range(1, len(self.patch_list) + 1):
                f = plt.figure(patch_num, figsize = (7.5, 9), dpi = 100)
                self.plot_triangles()
                if patch_num == 1:
                    patch_state_length = len(self.patch_list[patch_num - 1]) - 2
                    for i in range(1, patch_state_length + 1):
                        self.plot_vf_one_triangle(i, patch_num)
                else:
                    patch_length = len(self.patch_list[patch_num - 1]) - 2
                    patch_length_pre = 0
                    for i in range(1, patch_num):
                        patch_length_pre += len(self.patch_list[i - 1]) - 2
                    for i in range(patch_length_pre, min([patch_length_pre + patch_length, self.state_total_number + 1])):
                        self.plot_vf_one_triangle(i, patch_num)
        plt.savefig('fig.png')
        plt.show()


class Robot(State):
    #def __init__(self, geometry_name, agent_radius, agent_id, v_max, goals_file, init_pos=[0.,0.], time_step=0.05):
    def __init__(self, geometry_name, agent_radius, agent_id, v_max, init_pos=[0.,0.], time_step=0.05):
        super(Robot, self).__init__(geometry_name, agent_id)
        super(Robot, self).initial_gs()
        super(Robot, self).vertex_ijk()
        super(Robot, self).patches()
        self.temp_obs = 0
        self.current_state = 1
        self.poly = 0
        self.tri_vo = 0
        self.velocity_ref_image = 0
        self.velocity_arrow_image = 0
        self.local_state = 1
        self.islast_state = False
        self.time_step = time_step
        self.trajectory_desired = list()
        v1 = self.position(self.v1v2v3(self.current_state)[0])
        v2 = self.position(self.v1v2v3(self.current_state)[1])
        v3 = self.position(self.v1v2v3(self.current_state)[2])

        if init_pos[0] == 0. and init_pos[1] == 0.:
            x_init = v1[0] + v2[0] + v3[0]
            y_init = v1[1] + v2[1] + v3[1]
            x_init /= 3.
            y_init /= 3.
            self.initial_position = np.array([x_init, y_init])
        else:
            self.initial_position = np.array(init_pos)

        self.waypoint = [0., 0.]
        self.auto = True
        self.reinitial = False
        self.v_max = v_max
        self.agent_id = agent_id
        self.agent_radius = agent_radius
        self.current_triangle = int(self.triangle_number(self.current_state))
        self.current_patch = 1
        self.is_finished = False
        self.current_position = self.initial_position
        self.current_velocity = np.array(self.vector_field_point(self.current_position, self.current_patch, self.current_state, self.v_max)[0])
        self.goal_list = []
        #self.goals_file = goals_file
        self.poly_pix = []

        with open(geometryDir + '/agent{}_goal_num_file.txt'.format(self.agent_id)) as elements:
            self.goal_list.append(int(elements.readlines()[0]))
        elements.close()

        self.visited_goals = []
        self.not_created = True

        self.replan_request = False

    def ispoint_in_tri(self, point, state_num):
        v1v2v3_poses = []
        self.is_new_state = False

        for i in range(0, 3):
            v1v2v3_poses.append(np.array(self.position(self.v1v2v3(state_number=state_num)[i])))

        p = np.array(point)
        [v1, v2, v3] = v1v2v3_poses
        vector0 = v3 - v1
        vector1 = v2 - v1
        vector2 = p - v1
        dot00 = np.dot(vector0, vector0)
        dot01 = np.dot(vector0, vector1)
        dot02 = np.dot(vector0, vector2)
        dot11 = np.dot(vector1, vector1)
        dot12 = np.dot(vector1, vector2)
        invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * invDenom
        v = (dot00 * dot12 - dot01 * dot02) * invDenom
        if (u >= 0) and (v >= 0) and (u + v <= 1):
            self.is_new_state = False
        else:
            self.is_new_state = True

        return self.is_new_state

    def reinitiate(self, current_position):
        self.current_velocity = [0., 0.]
        self.current_patch = 1
        self.current_state = 1
        self.__init__(self.geometry_name, self.agent_radius, self.agent_id, self.v_max, '{}_newgoals{}'.format(self.geometry_name, self.agent_id), init_pos=current_position)

    def move(self, current_pos):
        self.replan_request = False
        self.trajectory_desired.append(current_pos)

        if self.is_finished:
            self.current_velocity = np.array([0., 0.])
            print(finished)
            plt.plot(self.trajectory_desired)
            plt.savefig('traj.png')
            plt.show()

        else:
            velocity_value = np.sqrt(self.current_velocity[0] ** 2 + self.current_velocity[1] ** 2)

            if int(self.triangle_number(self.current_state)) == int(self.state_list[self.state_total_number - 1]):
                if velocity_value < 0.5:
                    self.is_finished = True

            if int(self.triangle_number(self.current_state)) in self.goal_list:  # if the agent is in the goal region
                self.visited_goals.append(self.current_state)
                print('agent {} reached its goal....'.format(self.agent_id))
                self.is_finished = True

            self.state_has_changed = self.ispoint_in_tri(current_pos, self.current_state)  # check and see if the agent has moved to a new state_order

            next_state_not_reached = False
            if self.current_state + 1 < self.state_total_number:
                next_state_not_reached = self.ispoint_in_tri(current_pos, self.current_state+1)

            if self.local_state == len(self.patch_list[self.current_patch - 1]) - 2:  # if the agent current state_order is the last in the patch
                self.current_velocity = np.array(self.vector_field_point(current_pos, self.current_patch, self.current_state, self.v_max)[0])  # update velocity

                """ if the current patch is the last patch finish otherwise
                update the state_order and the patch number"""
                if self.current_patch == len(self.patch_length):
                    self.is_finished = True  # signal to finish
                    print "Finsihed by patch----"
                else:
                    self.current_patch += 1
                    self.local_state = 1
                    self.state_has_changed = True
                    print "patch increased by +1----"
            else:  # not in the last state_order but the state_order has changed
                if self.state_has_changed and not next_state_not_reached:
                    self.state_has_changed = False
                    self.local_state += 1
                    self.current_state += 1
                    self.current_velocity = np.array(self.vector_field_point(current_pos, self.current_patch, self.current_state, self.v_max)[0])
                elif self.state_has_changed and next_state_not_reached:
                    self.current_velocity = np.array(self.vector_field_point(current_pos, self.current_patch, self.current_state, self.v_max)[0])
                    self.replan_request = True
                else:
                    """ not in the last state_order of the patch and
                    still in the same state_order """
                    self.current_velocity = np.array(self.vector_field_point(current_pos, self.current_patch, self.current_state, self.v_max)[0])

        return self.current_velocity

    def estimated_time_to_reach(self):
        current_position = self.initial_position.copy()
        initial_position = self.initial_position.copy()
        predicted_trajectory = []
        counter = 0
        while not self.is_finished:
            counter += 1
            next_velocity = self.move(current_position)
            current_position += next_velocity * self.time_step
            predicted_trajectory.append(list(current_position)[0])
            predicted_trajectory.append(list(current_position)[1])

        total_time = len(predicted_trajectory) / 2. * self.time_step

        self.reinitiate(initial_position)

        return total_time, predicted_trajectory


if __name__ == "__main__":
    Robot = Robot('ColumbusCirclePoly', 1, 1, 1, 'goals')
    Robot.plot_triangles()
