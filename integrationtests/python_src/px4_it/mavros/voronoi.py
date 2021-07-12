#!/usr/bin/env python2
# -*-coding: UTF-8-*-

import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *
import matplotlib.pyplot as plt
from scipy.spatial import HalfspaceIntersection, ConvexHull
import time
from gazebo_msgs.msg import ModelStates
import rospy
import tf
import math
from std_msgs.msg import Float64MultiArray
from mpl_toolkits.mplot3d import axes3d

def get_theta(v):
    if v[0] > 0 and v[1] >= 0:
        return math.atan(v[1] / v[0])
    elif v[0] > 0 > v[1]:
        return math.atan(v[1] / v[0])
    elif v[0] == 0 and v[1] > 0:
        return np.pi / 2.0
    elif v[0] == 0 and v[1] < 0:
        return (-1) * np.pi / 2.0
    elif v[0] < 0 <= v[1]:
        return math.atan(v[1] / v[0]) + np.pi
    elif v[0] < 0 and v[1] < 0:
        return math.atan(v[1] / v[0]) - np.pi

def get_theta_diff(theta_s, theta_p):
    if (theta_p - theta_s >= np.pi):
        return theta_p - theta_s - 2 * np.pi
    if (theta_p - theta_s <= (-1) * np.pi):
        return theta_p - theta_s + 2 * np.pi
    return theta_p - theta_s

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def plot_poly(vertices):
    n_v = len(vertices)

    for i in range(n_v - 1):
        p1 = vertices[i]
        p2 = vertices[i + 1]
        plt.plot([p1[1], p2[1]], [-p1[0],-p2[0]], 'r-', linewidth=2.0)

    p1 = vertices[-1]
    p2 = vertices[0]
    plt.plot([p1[1], p2[1]], [-p1[0], -p2[0]], 'r-', linewidth=2.0)


class Edge:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.length = np.linalg.norm(start - end)

    def common_edge(self, e, p):
        """
        determine whether this edge is common edge between e and p
        :param e: evader's position, np array
        :param p: pursuer's position, np array
        :return: bool value
        """

        if self.length < 0.1:
            return False

        e_sta = np.linalg.norm(e - self.start)
        e_end = np.linalg.norm(e - self.end)
        p_sta = np.linalg.norm(p - self.start)
        p_end = np.linalg.norm(p - self.end)
        # print(e_sta)
        # print(p_sta)
        # print(e_end)
        # print(p_end)
        return np.isclose(e_sta, p_sta) and np.isclose(e_end, p_end)

    def mid_point(self):
        """

        :return: return edge mid-point, in numpy 1d array
        """
        return 0.5 * (self.start + self.end)


class Polygon:
    def __init__(self, vertices):
        self.edges = []
        n_v = len(vertices)
        for i in range(n_v - 1):
            self.edges.append(Edge(vertices[i], vertices[i+1]))
        self.edges.append(Edge(vertices[-1], vertices[0]))

class Controller:
    def __init__(self):
        plt.ion()
        print("class is setting up!")
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback, queue_size=1)
        self._velpub1 = rospy.Publisher('/husky_beta/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self._velpub2 = rospy.Publisher('/husky_gamma/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self._velpub3 = rospy.Publisher('/husky_delta/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self._velpub4 = rospy.Publisher('/husky_zeta/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        # self.__pub = rospy.Publisher(robot_name + '/voronoi_velocity', Float64MultiArray, queue_size=1)
        self.__setstate = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.__stamsg = SetModelStateRequest()
        self._start = False
        self.e_pos = None
        self.p_pos = None
        self.yaw = [0., 0., 0., 0., 0.]

        # game environment, in rectangle, left bottom corner (minx, miny), right top corner (maxx, maxy)
        self.minx = 10.
        self.miny = 20.
        self.maxx = 75.
        self.maxy = 80.
        self.e_cell_vertices = None
        self.e_cell = None
        self.pursuer_v = np.ones((4, 2))
        self._vor_msg = []

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            time1 = time.time()
            if self._start:
                self.plan_motion()
                self._vor_msg = []  # each time, vor_msg need to be initiated to empty list, since in Update_Pursuers_V function, append is used to add velocity msg.
                self.Update_Pursuers_V()

                print('------------------------------')
                self._velpub1.publish(self._vor_msg[0])
                self._velpub2.publish(self._vor_msg[1])
                self._velpub3.publish(self._vor_msg[2])
                self._velpub4.publish(self._vor_msg[3])
            rate.sleep()
            print(time.time() - time1)

    def _plot_env(self):
        plt.plot([self.miny, self.maxy], [-self.minx, -self.minx], 'k-', linewidth=2.0)
        plt.plot([self.maxy, self.maxy], [-self.maxx, -self.minx], 'k-', linewidth=2.0)
        plt.plot([self.miny, self.maxy], [-self.maxx, -self.maxx], 'k-', linewidth=2.0)
        plt.plot([self.miny, self.miny], [-self.maxx, -self.minx], 'k-', linewidth=2.0)

    def _plot_agent(self):
        x = self.e_pos[1]
        y = -self.e_pos[0]
        plt.plot(x, y, 'rp', markersize=6)
        for p, act in zip(self.p_pos, self.pursuer_v):
            plt.plot(p[1], -p[0], 'g^', markersize=6)
            plt.arrow(p[1], -p[0], 4*act[1], -4*act[0])

    def _plot(self):
        plt.axis('off')
        plt.axis('equal')
        offset = 20
        plt.xlim([self.miny - offset, self.maxy + offset])
        plt.ylim([-self.maxx - offset, -self.minx + offset])
        self._plot_env()
        self._plot_agent()
        plot_poly(self.e_cell_vertices)


    def _bound_constr(self):
        """
        calculate the H = [A, b], where Ax+b<=0, linear inequality constraints representation of
        the environment boundary
        :return: H, a numpy matrix
        """
        H = np.array([[-1, 0, self.minx], [1, 0, -self.maxx],
                      [0, -1, self.miny], [0, 1, -self.maxy]])
        return H

    def _get_constr(self, e, p):
        H = self._bound_constr()
        e_pos = e
        e_pos = e_pos.reshape((-1,))  # make sure that pos is 1d vector, same for pursuer's position
        for p_pos in p:
            p_pos = p_pos.reshape((-1,))
            b = normalize(p_pos - e_pos)
            mid = 0.5 * (e_pos + p_pos)  # mid-point between evader and pursuer
            c = (-b).dot(mid)
            h = np.hstack((b, c))
            H = np.vstack((H, h))
        return H

    def _cal_evader_cell(self, e, p):
        H = self._get_constr(e, p)
        e_pos = e.reshape((-1,))
        hs = HalfspaceIntersection(H, e_pos)
        cell_vertices = hs.intersections
        evader_cell = ConvexHull(cell_vertices)
        self.e_cell_vertices = cell_vertices[evader_cell.vertices]
        self.e_cell = Polygon(self.e_cell_vertices)

    def callback(self, msg):
        model_names = msg.name
        if self._start is not True:
            self._start = True

        if self.p_pos is None:
            self.p_pos = np.zeros((4, 2))
        for i in range(len(model_names)):
            if model_names[i] == "husky_alpha":
                roll, pitch, self.yaw[0] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                x = msg.pose[i].position.x
                y = msg.pose[i].position.y
                self.e_pos = np.array([x, y])
            elif model_names[i] == "husky_beta":
                roll, pitch, self.yaw[1] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.p_pos[0] = np.array([msg.pose[i].position.x, msg.pose[i].position.y])
            elif model_names[i] == "husky_gamma":
                roll, pitch, self.yaw[2] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.p_pos[1] = np.array([msg.pose[i].position.x, msg.pose[i].position.y])
            elif model_names[i] == "husky_delta":
                roll, pitch, self.yaw[3] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.p_pos[2] = np.array([msg.pose[i].position.x, msg.pose[i].position.y])
            elif model_names[i] == "husky_zeta":
                roll, pitch, self.yaw[4] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.p_pos[3] = np.array([msg.pose[i].position.x, msg.pose[i].position.y])

    def Update_Pursuers_V(self):
        theta_ = []
        for i in range(len(self.p_pos)):
            theta_.append(get_theta(self.pursuer_v[i]))
            temp = Twist()
            theta_r = self.yaw[i + 1]
            # print('agent current orientation: {}'.format(theta_r))
            # print('agent goal orientation: {}'.format(theta_[i]))
            # print('angle difference: {}'.format(get_theta_diff(theta_r, theta_[i])))
            temp.angular.z = np.clip(0.8 / np.pi * get_theta_diff(theta_r, theta_[i]), -0.8, 0.8)
            #if np.abs(get_theta_diff(theta_r, theta_[i])) >= np.pi / 6:
            #    temp.linear.x = 0.1
            #else:
            #    temp.linear.x = 0.8 - np.abs(temp.angular.z) / 0.8
            temp.linear.x = (1.0 - np.abs(temp.angular.z))*2
            temp.linear.y = 0.0
            temp.linear.z = 0.0
            # rospy.loginfo("velocity = {0}".format(temp.linear.x))
            # vel_msg.angular.z = 0.0
            temp.angular.y = 0.0
            temp.angular.x = 0.0
            self._vor_msg.append(temp)

        # rospy.loginfo("%f %f", theta_v[0], theta_v[1])

    def plan_motion(self):
        print(self.yaw)
        e = self.e_pos.copy()
        p = self.p_pos.copy()
        self._cal_evader_cell(e, p)
        # print('evader: {}'.format(e))
        for i, p_pos in enumerate(p):
            n_common = 0    # number of common edges
            # print('pursuer {}: {}'.format(index, p_pos))
            for edge in self.e_cell.edges:
                # print('start: {}'.format(edge.start))
                # print('end: {}'.format(edge.end))

                if edge.common_edge(e, p_pos):
                    n_common += 1
                    mid = edge.mid_point()
                    act = normalize(mid - p_pos)
                    self.pursuer_v[i] = act
            # print('--------------------')

            if n_common > 1:
                raise Exception("per pursuer should have only one common edge with evader")
            if n_common == 0:
                # if no common edge, the pursuer should move towards the evader
                act = normalize(e - p_pos)
                self.pursuer_v[i] = act

            self._plot()
            plt.pause(0.001)
            plt.cla()

if __name__ =="__main__":
    rospy.init_node('controller')
    node = Controller()
    rospy.spin()

