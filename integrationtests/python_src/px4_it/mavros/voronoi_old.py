#!/usr/bin/env python
# -*-coding: UTF-8-*-

import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.msg import ModelStates
import rospy
import tf
import math
from std_msgs.msg import Float64MultiArray


##----------------------------------全局参数列表------------------------------------------##

# Exit_Pos = [5., 40., 5., 40.]
# Exit_Pos = [-10., 10., -10., 10.]
# Exit_Pos = [10., 30., 20., 40.]

# pos_x = np.array([20., 12., 10.])
# pos_y = np.array([18., 20., 12.])


def judge(p1, p2, p3):
    return abs((p3[1] - p1[1]) * (p2[0] - p1[0]) - (p2[1] - p1[1]) * (p3[0] - p1[0])) < 1e-6


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


class Controller:
    __robot_name = ''
    __index = 0
    __capture_radius = 1
    __agent_num = 5
    __pursuer_num = 4
    pos_x = [75, 30, 75, 75, 75]
    pos_y = [30, 50, 20, 60, 80]
    yaw = [0., 0., 0., 0., 0.]
    __points = []
    __Exit_Pos = [10., 75., 20., 80.]

    def __init__(self):
        print("class is setting up!")
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback, queue_size=1)
        self.__velpub1 = rospy.Publisher('/husky_beta/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.__velpub2 = rospy.Publisher('/husky_gamma/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.__velpub3 = rospy.Publisher('/husky_delta/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.__velpub4 = rospy.Publisher('/husky_zeta/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        # self.__pub = rospy.Publisher(robot_name + '/voronoi_velocity', Float64MultiArray, queue_size=1)
        self.__setstate = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.__stamsg = SetModelStateRequest()
        self.__decision_is_start = False

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.__decision_is_start:
                self.pursuer_v = self.pursuer_decision(self.__agent_num, self.__points, self.__Exit_Pos)
                self.__vor_msg = []
                self.Update_Pursuers_V()
                self.__velpub1.publish(self.__vor_msg[0])
                self.__velpub2.publish(self.__vor_msg[1])
                self.__velpub3.publish(self.__vor_msg[2])
                self.__velpub4.publish(self.__vor_msg[3])
            rate.sleep()

    def callback(self, msg):
        model_names = msg.name
        self.__points = []
        if self.__decision_is_start is not True:
            self.__decision_is_start = True
        for i in range(len(model_names)):
            if model_names[i] == "husky_alpha":
                roll, pitch, self.yaw[0] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.pos_x[0] = msg.pose[i].position.x
                self.pos_y[0] = msg.pose[i].position.y
            elif model_names[i] == "husky_beta":
                roll, pitch, self.yaw[1] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.pos_x[1] = msg.pose[i].position.x
                self.pos_y[1] = msg.pose[i].position.y
            elif model_names[i] == "husky_gamma":
                roll, pitch, self.yaw[2] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.pos_x[2] = msg.pose[i].position.x
                self.pos_y[2] = msg.pose[i].position.y
            elif model_names[i] == "husky_delta":
                roll, pitch, self.yaw[3] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.pos_x[3] = msg.pose[i].position.x
                self.pos_y[3] = msg.pose[i].position.y
            elif model_names[i] == "husky_zeta":
                roll, pitch, self.yaw[4] = tf.transformations.euler_from_quaternion(
                    [msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z,
                     msg.pose[i].orientation.w])
                self.pos_x[4] = msg.pose[i].position.x
                self.pos_y[4] = msg.pose[i].position.y

        for i in range(self.__agent_num):
            self.__points.append([self.pos_x[i], self.pos_y[i]])
        print(self.__points)

        # rospy.loginfo("%f %f %f %f %f %f", self.__points[0][0], self.__points[0][1], self.__points[1][0],
        # self.__points[1][1], self.__points[2][0],self.__points[2][1])

    def Update_Pursuers_V(self):
        theta_ = []
        for i in range(self.__pursuer_num):
            theta_.append(get_theta(self.pursuer_v[i]))
            temp = Twist()
            theta_r = self.yaw[i + 1]
            temp.angular.z = np.clip(0.8 / np.pi * get_theta_diff(theta_r, theta_[i]), -0.8, 0.8)
            #if np.abs(get_theta_diff(theta_r, theta_[i])) >= np.pi / 6:
            #    temp.linear.x = 0.1
            #else:
            #    temp.linear.x = 0.8 - np.abs(temp.angular.z) / 0.8
            temp.linear.x = 0.8 - np.abs(temp.angular.z) / 0.8
            temp.linear.x *= 1.5
            temp.linear.y = 0.0
            temp.linear.z = 0.0
            # rospy.loginfo("%f, %f", theta_r, theta_v[self.__index - 1])
            # vel_msg.angular.z = 0.0
            temp.angular.y = 0.0
            temp.angular.x = 0.0
            self.__vor_msg.append(temp)

        # rospy.loginfo("%f %f", theta_v[0], theta_v[1])

    def pursuer_decision(self, agent_num, agent_pos, bound, max_v=1, frequency=1):
        ax = []
        ay = []
        plt.ion()
        print(agent_pos)
        for i in range(0, agent_num):
            ax.append(agent_pos[i][0])
            ay.append(agent_pos[i][1])
        plt.clf()
        plt.plot([bound[0], bound[0]], [bound[2], bound[3]], 'silver')
        plt.plot([bound[0], bound[1]], [bound[3], bound[3]], 'silver')
        plt.plot([bound[1], bound[1]], [bound[3], bound[2]], 'silver')
        plt.plot([bound[1], bound[0]], [bound[2], bound[2]], 'silver')
        plt.plot(ax[0], ay[0], 'r*', markersize=3)
        plt.plot(ax[1:], ay[1:], 'bo', markersize=3)
        agent_pos = np.array(agent_pos)
        if judge(agent_pos[0], agent_pos[1], agent_pos[2]):
            return np.array([[1, 0], [1, 0]])
        else:
            tri = Delaunay(agent_pos)
            circle = []
            tri_lines = []
            bounding_box = [bound[0], bound[1], bound[2], bound[3]]

            def get_outer_circle(A, B, C):
                xa, ya = A[0], A[1]
                xb, yb = B[0], B[1]
                xc, yc = C[0], C[1]

                xab, yab = (xa + xb) / 2.0, (ya + yb) / 2.0
                xbc, ybc = (xb + xc) / 2.0, (yb + yc) / 2.0

                if (xb != xa):
                    kab = (yb - ya) / (xb - xa)
                else:
                    kab = None

                if (xc != xb):
                    kbc = (yc - yb) / (xc - xb)
                else:
                    kbc = None

                if (kab != None):
                    ab = np.arctan(kab)
                else:
                    ab = np.pi / 2

                if (kbc != None):
                    bc = np.arctan(kbc)
                else:
                    bc = np.pi / 2

                if (ab == 0):
                    kabm = None
                    b1 = 0
                    x = xab
                else:
                    kabm = np.tan(ab + np.pi / 2)
                    b1 = yab * 1.0 - xab * kabm * 1.0

                if (bc == 0):
                    kbcm = None
                    b2 = 0
                    x = xbc
                else:
                    kbcm = np.tan(bc + np.pi / 2)
                    b2 = ybc * 1.0 - xbc * kbcm * 1.0

                if (kabm != None and kbcm != None):
                    x = (b2 - b1) * 1.0 / (kabm - kbcm)

                if (kabm != None):
                    y = kabm * x * 1.0 + b1 * 1.0
                else:
                    y = kbcm * x * 1.0 + b2 * 1.0

                r = np.sqrt((x - xa) ** 2 + (y - ya) ** 2)
                return (x, y, r)

            for num in range(0, tri.simplices.shape[0]):
                plt.axis('equal')
                plt.axis('off')
                x, y, r = get_outer_circle(agent_pos[tri.simplices[num][0]], agent_pos[tri.simplices[num][1]],
                                           agent_pos[tri.simplices[num][2]])
                circle.append([x, y])
                tri.simplices[num].sort()
                tup = (tri.simplices[num][0], tri.simplices[num][1])
                tri_lines.append(tup)
                tup = (tri.simplices[num][0], tri.simplices[num][2])
                tri_lines.append(tup)
                tup = (tri.simplices[num][1], tri.simplices[num][2])
                tri_lines.append(tup)

            i = 0
            dic = dict()
            for tri_line in tri_lines:
                if tri_line in dic.keys():
                    dic[tri_lines[i]].append(int(i) // int(3))
                    i = i + 1
                else:
                    dic[tri_lines[i]] = [int(i) // int(3)]
                    i = i + 1

            voronoi_graph = dict()

            def get_intersect_point(a, b, c, bound):
                flag = 0
                x1 = y1 = x2 = y2 = 0

                if b == 0:
                    x1 = x2 = -c / a
                    y1 = bound[2]
                    y2 = bound[3]
                else:
                    # 斜率存在
                    if (-c - a * bound[0]) / b <= bound[3] and (-c - a * bound[0]) / b >= bound[2]:
                        # print("线和x=bound[0]存在符合要求的交点")
                        if flag == 0:
                            x1 = bound[0]
                            y1 = (-c - a * bound[0]) / b
                            flag = 1
                        else:
                            x2 = bound[0]
                            y2 = (-c - a * bound[0]) / b
                            flag = 2

                    if (-c - a * bound[1]) / b <= bound[3] and (-c - a * bound[1]) / b >= bound[2]:
                        # print("线和x=bound[1]存在符合要求的交点")
                        if flag == 0:
                            x1 = bound[1]
                            y1 = (-c - a * bound[1]) / b
                            flag = 1
                        else:
                            # 找到过符合要求的交点
                            x2 = bound[1]
                            y2 = (-c - a * bound[1]) / b
                            flag = 2

                    if (-c - b * bound[2]) / a <= bound[1] and (-c - b * bound[2]) / a >= bound[0]:
                        # print("线和y=bound[2]存在符合要求的交点")
                        if flag == 0:
                            y1 = bound[2]
                            x1 = (-c - b * bound[2]) / a
                            flag = 1
                        else:
                            y2 = bound[2]
                            x2 = (-c - b * bound[2]) / a
                            flag = 2

                    if (-c - b * bound[3]) / a <= bound[1] and (-c - b * bound[3]) / a >= bound[0]:
                        # print("线和y=bound[3]存在符合要求的交点")
                        if flag == 0:
                            y1 = bound[3]
                            x1 = (-c - b * bound[3]) / a
                            flag = 1
                        else:
                            y2 = bound[3]
                            x2 = (-c - b * bound[3]) / a
                            flag = 2
                    if flag == 1:
                        # 只存在一个交点
                        x2 = x1
                        y2 = y1

                return flag, x1, y1, x2, y2

            def intersect(A, B, bound):
                C = [0, 0]
                if A[0] >= bound[0] and A[0] <= bound[1] and A[1] >= bound[2] and A[1] <= bound[3]:
                    if B[0] >= bound[0] and B[0] <= bound[1] and B[1] >= bound[2] and B[1] <= bound[3]:
                        flag = 1
                        return A[0], A[1], B[0], B[1], flag
                    else:
                        flag = 1
                        if (A[0] == B[0]):
                            if (B[1] > bound[3]):
                                x = A[0]
                                y = bound[3]
                            else:
                                x = A[0]
                                y = bound[2]
                            C[0] = x
                            C[1] = y
                        else:
                            a = A[1] - B[1]
                            b = B[0] - A[0]
                            c = B[1] * A[0] - A[1] * B[0]
                            num, x1, y1, x2, y2 = get_intersect_point(a, b, c, bound)
                            if x1 >= min(A[0], B[0]) and x1 <= max(A[0], B[0]) and y1 >= min(A[1],
                                                                                             B[1]) and y1 <= max(
                                A[1],
                                B[1]):
                                C[0] = x1
                                C[1] = y1
                            else:
                                C[0] = x2
                                C[1] = y2
                        return A[0], A[1], C[0], C[1], flag
                else:
                    if B[0] >= bound[0] and B[0] <= bound[1] and B[1] >= bound[2] and B[1] <= bound[3]:
                        flag = 1
                        if (A[0] == B[0]):
                            if (A[1] > bound[3]):
                                x = B[0]
                                y = bound[3]
                            else:
                                x = B[0]
                                y = bound[2]
                            C = [x, y]
                        else:
                            a = A[1] - B[1]
                            b = B[0] - A[0]
                            c = B[1] * A[0] - A[1] * B[0]
                            num, x1, y1, x2, y2 = get_intersect_point(a, b, c, bound)
                            if x1 >= min(A[0], B[0]) and x1 <= max(A[0], B[0]) and y1 >= min(A[1],
                                                                                             B[1]) and y1 <= max(
                                A[1],
                                B[1]):
                                C[0] = x1
                                C[1] = y1
                            else:
                                C[0] = x2
                                C[1] = y2
                        return B[0], B[1], C[0], C[1], flag
                    else:
                        flag = 0
                        if (A[0] == B[0]):
                            return A[0], A[1], B[0], B[1], flag
                        else:
                            a = A[1] - B[1]
                            b = B[0] - A[0]
                            c = B[1] * A[0] - A[1] * B[0]
                            num, x1, y1, x2, y2 = get_intersect_point(a, b, c, bound)
                            if num > 0:
                                return x1, y1, x2, y2, flag
                            else:
                                return A[0], A[1], B[0], B[1], flag

            def IsIntersec(p1, p2, p3, p4):
                a = p2[1] - p1[1]
                b = p1[0] - p2[0]
                c = p2[0] * p1[1] - p1[0] * p2[1]
                # print(a, b, c)
                if (a * p3[0] + b * p3[1] + c) * (a * p4[0] + b * p4[1] + c) <= 0:
                    return 1
                else:
                    return 0

            def midline(A, B, C, bound):
                a = 2 * (B[0] - A[0])
                b = 2 * (B[1] - A[1])
                c = A[0] ** 2 - B[0] ** 2 + A[1] ** 2 - B[1] ** 2
                num, x1, y1, x2, y2 = get_intersect_point(a, b, c, bound)
                D = [x1, y1]
                if IsIntersec(A, B, C, D):
                    D = [x1, y1]
                else:
                    D = [x2, y2]
                return D

            for key, value in dic.items():
                if len(value) == 2:
                    x1, y1, x2, y2, flag = intersect(circle[value[0]], circle[value[1]], bounding_box)
                    voronoi_graph[key] = [[x1, y1], [x2, y2], flag]
                    if key[0] == 0 or key[1] == 0:
                        plt.plot([x1, x2], [y1, y2], 'b')
                else:
                    for i in range(0, 3):
                        if (tri.simplices[value[0]][i] != key[0] and tri.simplices[value[0]][i] != key[1]):
                            peak = [agent_pos[tri.simplices[value[0]][i]][0],
                                    agent_pos[tri.simplices[value[0]][i]][1]]
                            break
                    if circle[value[0]][0] < bounding_box[0] or circle[value[0]][0] > bounding_box[1] or \
                            circle[value[0]][
                                1] < \
                            bounding_box[2] or circle[value[0]][1] > bounding_box[3]:
                        x1, y1, x2, y2, flag = intersect(circle[value[0]],
                                                         midline(agent_pos[key[0]], agent_pos[key[1]], peak,
                                                                 bounding_box),
                                                         bounding_box)
                    else:
                        x1, y1 = circle[value[0]][0], circle[value[0]][1]
                        x2, y2 = midline(agent_pos[key[0]], agent_pos[key[1]], peak, bounding_box)
                        flag = 1
                    voronoi_graph[key] = [[x1, y1], [x2, y2], flag]
                    if key[0] == 0 or key[1] == 0:
                        plt.plot([x1, x2], [y1, y2], 'b')

            neighbor = []
            unneighbor = []

            for tri_line in tri_lines:
                if (tri_line[0] == 0 or tri_line[1] == 0):
                    if tri_line[1] + tri_line[0] not in neighbor:
                        if voronoi_graph[tri_line][2] != 0:
                            if voronoi_graph[tri_line][0][0] != voronoi_graph[tri_line][1][0] or \
                                    voronoi_graph[tri_line][0][
                                        1] != voronoi_graph[tri_line][1][1]:
                                neighbor.append(tri_line[1] + tri_line[0])
            # print(neighbor)

            for i in range(1, agent_num):
                if i not in neighbor:
                    unneighbor.append(i)

            vp = []
            for i in range(1, agent_num):
                if i in neighbor:
                    mid = np.array([(voronoi_graph[(0, i)][0][0] + voronoi_graph[(0, i)][1][0]) / 2.0,
                                    (voronoi_graph[(0, i)][0][1] + voronoi_graph[(0, i)][1][1]) / 2.0])
                    # print(i)
                    # print(mid)
                    vp.append((mid - agent_pos[i]) * max_v / (np.sqrt(np.sum(np.square(mid - agent_pos[i]))) * 1))
                else:
                    vp.append((agent_pos[0] - agent_pos[i]) * max_v / (
                            np.sqrt(np.sum(np.square(agent_pos[0] - agent_pos[i]))) * 1))

            pursuer_v = np.array(vp)
            plt.pause(0.0001)
            plt.ioff

            return pursuer_v

if __name__ == "__main__":
    rospy.init_node('controller')
    node = Controller()
    rospy.spin()
