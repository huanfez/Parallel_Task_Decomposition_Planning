#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import os
import sys
import subprocess
# import os.path
import pandas as pd

#currentDir = os.getcwd()
currentDir = '/home/i2rlab/catkin_ws/src/mrs_taskplan/node/'
geometryDir = os.path.join(currentDir, 'Geometry')


def excel_to_polygon(excelFile):
    for d, r, f in os.walk(geometryDir):
        for files in f:
            if files.endswith('.xlsx'):
                path2excel = os.path.join(geometryDir, files)

    df = pd.read_excel(path2excel)

    X = df['X'] * 0.3048
    Y = df['y'] * 0.3048

    ID = df['Building_ID']
    ID = ID.tolist()

    del df  # delet dataFrame to save memory

    filename = excelFile + '.poly'
    file = open(os.path.join(geometryDir, filename), "w")
    polyFileName = os.path.join(geometryDir, filename)
    file.writelines(str(len(X)) + ' 2 1\n')

    for i in range(len(X)):
        file.writelines(str(i + 1) + ' ' + str(X[i]) + ' ' + str(Y[i]) + '\n')

    file.write(str(len(X)) + ' 0\n')

    start_point = 1
    for i in range(len(X)):
        if i < len(X) - 1:
            if ID[i] == ID[i + 1]:
                file.write(str(i + 1) + ' ' + str(i + 1) + ' ' + str(i + 2) + '\n')

            else:
                file.write(str(i + 1) + ' ' + str(i + 1) + ' ' + str(start_point) + '\n')
                start_point = i + 2

        else:
            file.write(str(i + 1) + ' ' + str(i + 1) + ' ' + str(start_point) + '\n')

    file.write(str(max(ID)) + '\n')

    j = ID.count(0)
    for i in range(1, max(ID) + 1):
        num_points = ID.count(i)
        x_mid = sum(X[j:j + num_points]) / num_points
        y_mid = sum(Y[j:j + num_points]) / num_points
        j += num_points
        file.write(str(i) + ' ' + str(x_mid) + ' ' + str(y_mid) + '\n')

    file.close()

    return X, Y, ID


class HighLevelPlanner:
    def __init__(self, geometry_name, initial_position, agent_num, temp_obstacle = []):
        self.vertices_coordinates = []
        self.tri_nodes_list = []
        self.neighbors_list_temp = []
        self.init_triangle = -1
        self.state_order = []
        self.poly = []
        self.geometry_name = geometry_name
        self.initial_position = initial_position
        #self.goal_file = goal_file
        self.goals = []
        self.not_added_goals = []
        self.temp_obstacle = 0
        self.temp_obstacle_pos = temp_obstacle
        #self.goal_list = goal_list
        self.agent_num = agent_num
        self.planning_validation = True
        self.goal_triangle_num = -1

    def discrete_planner(self):

        os.system('triangle -BQgneq25a.3 {}'.format(os.path.join(geometryDir, self.geometry_name) + '.poly'))
        print 'started triangle--'
        with open(os.path.join(geometryDir, self.geometry_name) + '.1.node') as vpos:
            self.vertices_pos = vpos.readlines()

        vpos.close()

        del (self.vertices_pos[-1])

        for i in range(1, len(self.vertices_pos)):
            vertice = self.vertices_pos[i].strip().split(' ')
            vertice = [float(i) for i in vertice if bool(i) == True]
            self.vertices_coordinates.append(vertice[1:3])

        with open(geometryDir + '/{}.1.ele'.format(self.geometry_name)) as elements:
            triangle_vertice_list = elements.readlines()

        elements.close()

        del (triangle_vertice_list[-1])

        # Triangles numbers and Vertices numbers
        for i in range(1, len(triangle_vertice_list)):
            triangle_nodes = triangle_vertice_list[i].strip().split(' ')
            triangle_nodes = [int(i) for i in triangle_nodes if bool(i) == True]
            self.tri_nodes_list.append(triangle_nodes[1:4])

        with open(geometryDir + '/{}.1.neigh'.format(self.geometry_name)) as neighbors_text:
            neighbors = neighbors_text.readlines()
        neighbors_text.close()

        del (neighbors[-1])

        for i in range(1, len(neighbors)):
            neighbor_num = neighbors[i].strip().split(' ')
            neighbor_num = [int(i) for i in neighbor_num if bool(i) == True]
            self.neighbors_list_temp.append(neighbor_num[1:4])

        for tri_num, tri_vrtx_num in enumerate(self.tri_nodes_list):
            p = self.initial_position
            v = []
            for i in range(0,3):
                v.append(np.array(self.vertices_coordinates[tri_vrtx_num[i] - 1]))
            triangle_number = tri_num + 1
            vector0 = v[2] - v[0]
            vector1 = v[1] - v[0]
            vector2 = p - v[0]
            dot00 = np.dot(vector0, vector0)
            dot01 = np.dot(vector0, vector1)
            dot02 = np.dot(vector0, vector2)
            dot11 = np.dot(vector1, vector1)
            dot12 = np.dot(vector1, vector2)
            invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
            u = (dot11 * dot02 - dot01 * dot12) * invDenom
            v = (dot00 * dot12 - dot01 * dot02) * invDenom
            if (u >= 0.) and (v >= 0.) and (u + v <= 1.):
                self.init_triangle = triangle_number
                print "initial state exists: " + str(triangle_number)
                break

        if self.init_triangle == -1:
            self.planning_validation = False
        else:
            self.planning_validation = True

#            if self.goal_list==[]:
#                with open(os.path.join(geometryDir, self.goal_file) + '.txt', 'r') as g:
#                    self.goals = g.readlines()

#                g.close()
#                self.goals = [int(value.strip()) for value in self.goals[0].split(",")]

#            else:
#                self.goals = self.goal_list


            for tri_num, tri_vrtx_num in enumerate(self.tri_nodes_list):
                v = []

                for j in range(0, 3):
                    v.append(np.array(self.vertices_coordinates[tri_vrtx_num[j] - 1]))
                triangle_number = tri_num + 1
                vector0 = v[2] - v[0]
                vector1 = v[1] - v[0]
                vector2 = self.goals - v[0]
                dot00 = np.dot(vector0, vector0)
                dot01 = np.dot(vector0, vector1)
                dot02 = np.dot(vector0, vector2)
                dot11 = np.dot(vector1, vector1)
                dot12 = np.dot(vector1, vector2)
                invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
                u = (dot11 * dot02 - dot01 * dot12) * invDenom
                v = (dot00 * dot12 - dot01 * dot02) * invDenom
                if (u >= 0.) and (v >= 0.) and (u + v <= 1.):
                    self.goal_triangle_num = triangle_number
                    print "goal state: " + str(triangle_number)
                    goal_file = open(os.path.join(geometryDir, 'agent{}_goal_num_file'.format(self.agent_num)) + '.txt', 'w')
                    goal_file.write(str(self.goal_triangle_num))
                    goal_file.close()
                    break

            # temporary obstacles
            if self.temp_obstacle_pos != []:
                for tri_num, tri_vrtx_num in enumerate(self.tri_nodes_list):
                    v = []

                    for j in range(0, 3):
                        v.append(np.array(self.vertices_coordinates[tri_vrtx_num[j] - 1]))
                    triangle_number = tri_num + 1
                    vector0 = v[2] - v[0]
                    vector1 = v[1] - v[0]
                    vector2 = self.temp_obstacle_pos - v[0]
                    dot00 = np.dot(vector0, vector0)
                    dot01 = np.dot(vector0, vector1)
                    dot02 = np.dot(vector0, vector2)
                    dot11 = np.dot(vector1, vector1)
                    dot12 = np.dot(vector1, vector2)
                    invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
                    u = (dot11 * dot02 - dot01 * dot12) * invDenom
                    v = (dot00 * dot12 - dot01 * dot02) * invDenom
                    if (u >= 0.) and (v >= 0.) and (u + v <= 1.):
                        self.temp_obstacle = triangle_number
                        print "temp_obstacle: " + str(triangle_number)
                        break

            if os.path.isfile('inNuSMV_agent{}.smv'.format(self.agent_num)):  # remove previous output file
                os.remove('inNuSMV_agent{}.smv'.format(self.agent_num))

            with open(geometryDir + '/{}.1.neigh'.format(self.geometry_name)) as f:  # opens input file (C.1.neigh)
                lines = f.readlines()
            f.close()

            del lines[-1]
            state_number = len(lines)
            text_file = open(currentDir + 'inNuSMV_agent{}.smv'.format(self.agent_num), 'w')
            text_file.write('MODULE main\nVAR\nx : grid;\n')
            text_file.write('LTLSPEC !( (')
            if self.temp_obstacle == 0:
                text_file.write(' &'.join(' F (x.state_order = ' + str(gg) + ')' for gg in [self.goal_triangle_num]) + ' ) )')
            else:
                text_file.write(' &'.join(' F (x.state_order = ' + str(gg) + ')' for gg in [self.goal_triangle_num]) + ' & G !(x.state_order = {} )'.format(self.temp_obstacle) + ' ) )')
            text_file.write('\nMODULE grid\nVAR\nstate_order : 1..')
            text_file.write(state_number.__str__())
            text_file.write(';\nASSIGN\ninit(state_order) := {};\nnext(state_order) := \ncase\n'.format(self.init_triangle))
            for i in range(1, state_number):
                pp = lines[i].strip().split(' ')
                pp = [ii for ii in pp if bool(ii) == True]
                pp = pp[1:]
                #print str(i) + ' ' + str(pp)

                try:
                    pp.remove('-1')
                except ValueError:
                    None

                try:
                    pp.remove('-1')
                except ValueError:
                    None

                text_file.write('state_order = %d : {' % i + ', '.join(p for p in pp) + '};\n')

            text_file.write('TRUE : state_order ;\nesac;')
            text_file.close()

            # Runs NuSMV and outputs the file state_order ######
            output = subprocess.check_output('NuSMV ' + currentDir + 'inNuSMV_agent{}.smv'.format(self.agent_num), shell=True, universal_newlines=True)
            text_file = open(currentDir + 'smv_output_agent{}.txt'.format(self.agent_num), 'w')
            text_file.write(output)
            text_file.close()
            text_file = open(currentDir + 'smv_output_agent{}.txt'.format(self.agent_num), 'r')
            #print text_file.read()

            with open(currentDir + 'smv_output_agent{}.txt'.format(self.agent_num), 'r') as path:
                paths = path.readlines()

                del paths[0:28]

            self.not_added_goals.append(self.goal_triangle_num)

            goal_written = False

            self.state_order = []
            for lines in paths:
                if 'x.state_order' in lines and not goal_written:
                    self.state_order.append(lines[18:].strip())

                    if str(self.goal_triangle_num) in lines:
                        goal_written = True

                    if int(self.state_order[-1]) in [self.not_added_goals]:
                        self.not_added_goals.remove(int(self.state_order[0][-1]))

            if os.path.isfile('state_order_{}_agent{}.txt'.format(self.geometry_name, self.agent_num)):  # remove previous output file
                os.remove('state_order_{}_agent{}.txt'.format(self.geometry_name, self.agent_num))
            text_file2 = open(currentDir + 'state_order_{}_agent{}.txt'.format(self.geometry_name, self.agent_num), 'w')
            text_file2.write('\n'.join(self.state_order))
            text_file2.close()

        return self.planning_validation

    def plotDiscretePath(self):
        with open(os.path.join(geometryDir, self.geometry_name) + '.poly', 'r') as geo:
            polygons = geo.readlines()

        geo.close()

        frame1 = plt.gca()
        frame1.axes.get_xaxis().set_ticks([])
        frame1.axes.get_yaxis().set_ticks([])

        for line in range(len(polygons)):
            polygonLines = polygons[line].strip().split(' ')
            polygonLines = [float(i) for i in polygonLines if bool(i) == True]
            self.poly.append(polygonLines)

        numLines = int(self.poly[0][0])

        for edge in range(numLines):
            edgeIdx = int(self.poly[edge + numLines + 2][1])
            edgeIdy = int(self.poly[edge + numLines + 2][2])
            plt.plot([self.poly[edgeIdx][1], self.poly[edgeIdy][1]],[self.poly[edgeIdx][2], self.poly[edgeIdy][2]],'b')

        # uncomment the following block to plot the discrete path

        # for trngls in range(len(self.state_order)):
        #     centerPoint = np.array([0.,0.])
        #
        #     for i, node in enumerate(self.tri_nodes_list[int(self.state_order[trngls]) - 1]):
        #         vxCoords = self.vertices_coordinates[node - 1]
        #         plt.scatter(vxCoords[0], vxCoords[1], marker= ',')
        #         centerPoint += np.array(vxCoords)
        #         print(vxCoords)
        #     centerPoint /= 3.
        #     plt.scatter(centerPoint[0], centerPoint[1], marker='.', linewidths = .1)
        #     print(centerPoint)

        # plt.savefig(self.geometry_name + '.png', frameon=False, bbox_inches="tight", pad_inches=0)
        #
        plt.show()

        return numLines, edgeIdx, edgeIdy, self.poly

    def stateCenterPointFinder(self, state):
        print(len(self.state_order))

        if len(self.state_order) == 0:
            self.discrete_planner()

        self.center_point = np.array([0., 0.])

        for i, node in enumerate(self.tri_nodes_list[int(self.state_order[state -1]) - 1]):
            vxCoords = self.vertices_coordinates[node - 1]
            plt.scatter(vxCoords[0], vxCoords[1], marker=',')
            self.center_point += np.array(vxCoords)

        self.center_point /= 3.

        print('State = {} --> Triangle Number = {} --> Center Point = {}'.format(state, self.state_order[state -1], self.center_point))

    def PathTriangles(self):
        points = []
        if len(self.state_order) == 0:
            self.discrete_planner()

        for j in range(len(self.state_order)):
            triangle_vxcoords = []
            for i, node in enumerate(self.tri_nodes_list[int(self.state_order[j])-1]):
                vxCoords = self.vertices_coordinates[node - 1]
                triangle_vxcoords.append(vxCoords)

            points.append(triangle_vxcoords)

        return points


    def triangleCenterPointFinder(self, triangleNum):
        if len(self.state_order) == 0:
            self.discrete_planner()

        self.center_point = np.array([0., 0.])

        for i, node in enumerate(self.tri_nodes_list[triangleNum - 1]):
            vxCoords = self.vertices_coordinates[node - 1]
            plt.scatter(vxCoords[0], vxCoords[1], marker=',')
            self.center_point += np.array(vxCoords)

        self.center_point /= 3.

        return self.center_point


    def pose2discrete_state(self, pose):
        """Mapping a position into a discrete triangle state"""
        for tri_num, tri_vrtx_num in enumerate(self.tri_nodes_list):
            v = []

            for j in range(0, 3):
                v.append(np.array(self.vertices_coordinates[tri_vrtx_num[j] - 1]))

            triangle_number = tri_num + 1
            vector0 = v[2] - v[0]
            vector1 = v[1] - v[0]
            vector2 = pose - v[0]
            dot00 = np.dot(vector0, vector0)
            dot01 = np.dot(vector0, vector1)
            dot02 = np.dot(vector0, vector2)
            dot11 = np.dot(vector1, vector1)
            dot12 = np.dot(vector1, vector2)
            invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
            u = (dot11 * dot02 - dot01 * dot12) * invDenom
            v = (dot00 * dot12 - dot01 * dot02) * invDenom

            if (u >= 0.) and (v >= 0.) and (u + v <= 1.):
                return triangle_number

        return -1


if __name__== "__main__":
    # excel_to_polygon('ColumbusCirclePoly')
    tempInitPos = [0.94, 1.00]

    Planner = HighLevelPlanner(geometry_name ='ColumbusCirclePoly', initial_position = tempInitPos, goal_file ='goals1', agent_num= 0, goal_list = [])
    Planner.plotDiscretePath()

    print()
