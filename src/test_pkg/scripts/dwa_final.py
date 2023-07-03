#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import queue
import std_msgs.msg as std_msgs
"""
rostopic pub -r 20 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: -2.2
    y: -0.8
    z: 0.0
  orientation:
    x: 0.0
    w: 1.0" 
"""

global goal_flag
goal_flag = False
goal_s = Int32()
amcl_x = 0.0
amcl_y = 0.0
amcl_yaw = 0.0

class Config(object):
    """
    Parameters used for simulation
    """
    def __init__(self):
        # robot parameter
        self.max_speed = 0.4 # [m/s] # 最大速度
        # self.min_speed = -0.5 # [m/s] # min v Set to reverse
        self.min_speed = -0.4 # [m/s] # min v Set to can not reverse
        self.max_yawrate = 120 * math.pi / 180.0 # [rad/s] # 最大角速度
        self.max_accel = 0.15 # [m/ss] # 最大加速度
        self.max_dyawrate = 60.0 * math.pi / 180.0 # [rad/ss] # 最大角加速度
        self.v_reso = 0.01 # [m/s] 速度分辨率
        self.yawrate_reso = 0.2 * math.pi / 180.0 # [rad/s] 角度分辨率
        self.dt = 0.1 # [s] # 采样时间
        self.predict_time = 1.5 # [s] # 模拟时间
        #权重参数
        self.to_goal_cost_gain = 1.0 # Target cost gain
        self.speed_cost_gain = 1.0 # Target cost reduction
        self.ob_cost_gain = 1.0 #障碍物权重（没用到）
        # self.to_goal_angle_cost = 5.0
        self.robot_radius = 0.1 # [m] # robor radius


def amcl_cb(amcl_msg):
    global amcl_x,amcl_y,amcl_yaw
    amcl_x = amcl_msg.pose.pose.position.x
    amcl_y = amcl_msg.pose.pose.position.y

    amcl_pose = amcl_msg.pose.pose
    (r, p, y) = tf.transformations.euler_from_quaternion([amcl_pose.orientation.x, amcl_pose.orientation.y, amcl_pose.orientation.z, amcl_pose.orientation.w])
    amcl_yaw = y


def calc_to_goal_cost(x, goal, config):
    # calc to goal cost. It is 2D norm.

    dx = goal[0] - x[0]
    dy = goal[1] - x[1]
    goal_dis = math.sqrt(dx ** 2 + dy ** 2)
    cost = config.to_goal_cost_gain * goal_dis

    return cost


def calc_dynamic_window(x, config):
    vs = [config.min_speed, config.max_speed,
    -config.max_yawrate, config.max_yawrate]

    # max_v, min_v
    vd = [x[3] - config.max_accel * config.dt,
    x[3] + config.max_accel * config.dt,
    x[4] - config.max_dyawrate * config.dt,
    x[4] + config.max_dyawrate * config.dt]
    # print(Vs, Vd)

    #
    vr = [max(vs[0], vd[0]), min(vs[1], vd[1]),
    max(vs[2], vd[2]), min(vs[3], vd[3])]
    # if start == 1:
    # print(vr)
    return vr


def motion(x, u, dt):
    # The velocity updating formula is simple, and the vehicle displacement changes greatly in a very short time
    #
    x[0] += u[0] * math.cos(x[2]) * dt # x
    x[1] += u[0] * math.sin(x[2]) * dt # y
    x[2] += u[1] * dt # heading
    x[3] = u[0] # v
    x[4] = u[1] # w
    # print(x)

    return x


def calc_trajectory(x_init, v, w, config):
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, w], config.dt)
        trajectory = np.array(x) # vertical
        time += config.dt

    # print(trajectory)
    return trajectory


def calc_final_input(x, u, vr, config, goal):
    x_init = x[:]
    min_cost = 10000.0
    min_u = u

    # evaluate all trajectory with sampled input in dynamic window
    # v,w
    for v in np.arange(vr[0], vr[1], config.v_reso):
        for w in np.arange(vr[2], vr[3], config.yawrate_reso):

            trajectory = calc_trajectory(x_init, v, w, config)
            # calc cost
            to_goal_cost = calc_to_goal_cost(trajectory , goal , config)
            # to_goal_angle_cost = calc_to_goal_angle_cost(trajectory, goal, config)
            speed_cost = config.speed_cost_gain * (config.max_speed - abs(trajectory[3]))
            # print(ob_cost)

            #
            #
            final_cost = to_goal_cost + speed_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, w]

                # print(min_u)
                # input()

    return min_u


def dwa_control(x, u, config, goal):
    # Dynamic Window control

    vr = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, vr, config, goal)

    return u


def goal_cb(goal_msg):
    # print("------------------huidiaohanshu-----------------------")
    global goal_s
    goal_s = goal_msg
    global goal_flag
    goal_flag = True
    # print(goal_flag)
    # print("-----------jieshu------------------")
    print('-------In the goal Callback---',goal_s.pose.position.x,goal_s.pose.position.y)

Queue=queue.Queue()
def LoacateCB(msg):
    if msg.data==1:
        Queue.put(1)


def main():
    global queue_out
    global amcl_x,amcl_y,amcl_yaw
    rospy.init_node("DWA2")
    tf_listener = tf.TransformListener()
    amcl_sub = rospy.Subscriber("/amcl_pose" , PoseWithCovarianceStamped , amcl_cb, queue_size=1)
    goal_sub = rospy.Subscriber("parking" , Int32 , goal_cb, queue_size=1)
    # dwa_contril_pub = rospy.Publisher("/dwa_control" , Twist , queue_size=1)
    dwa_contril_pub = rospy.Publisher("/dwa_cmd_vel" , Twist , queue_size=1)
    dwa_control_twist = Twist()
    config = Config()

    #transformPose
    print("-------------here---------------")

    count = 0
    while not goal_flag:
        
        if count == 0:
            print("-------wait for goal--------")
            count = 1
        if goal_flag == True:
            print("-------------break of while----------------")
            break
        #
    # goal_temp = tf_listener.transformPose('odom' , goal_s)
    # goal = np.array([goal_temp.pose.position.x , goal_temp.pose.position.y])


    #goal的值固定死



    goal1 = np.array([goal_s.pose.position.x , goal_s.pose.position.y])
    print("----------------goal received!-------------",goal1[0],goal1[1])
    
    #x的速度和角速度可能要改
    x = np.array([amcl_x , amcl_y , amcl_yaw , 0 , 0])

    #初始速度和角速度可能要改
    u = np.array([0.4, 0.0])

    while not rospy.is_shutdown():
        u = dwa_control(x, u, config, goal1)
        dwa_control_twist.linear.x = u[0]
        dwa_control_twist.angular.z = u[1]
        dwa_contril_pub.publish(dwa_control_twist)
        #通过定位更新x
        x = np.array([amcl_x , amcl_y , amcl_yaw , u[0] , u[1]])

        if ((x[0] - goal1[0]) ** 2 + (x[1] - goal1[1]) ** 2) <= config.robot_radius ** 2:
            dwa_control_twist.linear.x = 0
            dwa_control_twist.angular.z = 0
            dwa_contril_pub.publish(dwa_control_twist)
            print("------------Goal Reached!--------------",goal1[0],goal1[1])
            break
    


    #目标点写死，并且这个goal2根据char文字识别结果变化
    goal2 = np.array([goal_s.pose.position.x , goal_s.pose.position.y])
    #x的速度和角速度可能要改
    x = np.array([amcl_x , amcl_y , amcl_yaw , 0 , 0])

    #初始速度和角速度可能要改
    u = np.array([-0.4, 0.0])

    while not rospy.is_shutdown():
        u = dwa_control(x, u, config, goal2)
        dwa_control_twist.linear.x = u[0]
        dwa_control_twist.angular.z = u[1]
        dwa_contril_pub.publish(dwa_control_twist)
        #通过定位更新x
        x = np.array([amcl_x , amcl_y , amcl_yaw , u[0] , u[1]])

        if ((x[0] - goal2[0]) ** 2 + (x[1] - goal2[1]) ** 2) <= config.robot_radius ** 2:
            dwa_control_twist.linear.x = 0
            dwa_control_twist.angular.z = 0
            dwa_contril_pub.publish(dwa_control_twist)
            print("------------Goal Reached!--------------",goal2[0],goal2[1])
            break
###########结尾再发一个定位：RoadLine to Start
        

if __name__ == '__main__':
    locate_sub=rospy.Subscriber("/dwa_flag",std_msgs.Int32,LoacateCB)
    main()



