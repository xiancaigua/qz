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
import Queue
import std_msgs.msg as std_msgs
"""
ackermann_msgs/AckermannDrive.h
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
#该flag调试用
p_flag_ = False

goal_flag = False
park_flag = Int32()
amcl_x = 0.0
amcl_y = 0.0
amcl_yaw = 0.0
u0 = 0.0
u1 = 0.0
#1.0 -0.62
goal1_x = 1.25  #原本1.19
goal1_y = -1.00 #原本是-1.01 
# 0.55 0.85
goal2_x = 1.30
goal2_y = 0.70 #0.80s

class Config(object):
    """
    Parameters used for simulation
    """
    def __init__(self):
        # robot parameter
        self.max_speed = 0.8 # [m/s] # 最大速度    原来的0.4
        # self.min_speed = -0.5 # [m/s] # min v Set to reverse
        self.min_speed = -0.8 # [m/s] # min v Set to can not reverse
        self.max_yawrate = 120 * 3.1415926 / 180.0 # [rad/s] # 最大角速度
        self.max_accel = 0.4 # [m/ss] # 最大加速度0.15
        self.max_dyawrate = 3 * 3.1415926 / 180.0 # [rad/ss] # 最大角加速度45  原来是5 速度为4的时候 速度为8的时候为4 
        self.v_reso = 0.01 # [m/s] 速度分辨率
        self.yawrate_reso = 0.2 * 3.1415926 / 180.0 # [rad/s] 角度分辨率 原来的是0.2
        self.dt = 0.05 # [s] # 采样时间
        self.predict_time = 0.8  # [s] # 模拟时间 1.0
        #权重参数
        self.to_goal_cost_gain = 0.8 # Target cost gain
        self.speed_cost_gain = 0.8 # Target cost reduction 1.0
        self.ob_cost_gain = 1.0 #障碍物权重（没用到）
        self.angle_cost_gain = 0.4 #朝向的权重3.0 原来0.19
        self.dx_cost_gain = 1.9 #水平线的权重 原来1.7   8的时候是1.9
        # self.to_goal_angle_cost = 5.0
        self.robot_radius = 0.3 # [m] # robor radius

    def set_angle_cost_gain(self,angle_cost_gain):
        self.angle_cost_gain = angle_cost_gain


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

def calc_angle_cost(trajectory,config):
    cost = abs(-3.1415926 / 2 - trajectory[2])
    return config.angle_cost_gain * cost

def calc_dx_cost(trajectory,goal,config):
    cost  = abs(trajectory[0] - goal[0])
    return config.dx_cost_gain * cost

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
            angle_cost = calc_angle_cost(trajectory,config)
            dx_cost = calc_dx_cost(trajectory,goal,config)
            speed_cost = config.speed_cost_gain * (config.max_speed - abs(trajectory[3]))
            # print(ob_cost)

            #
            #
            final_cost = to_goal_cost + speed_cost + angle_cost + dx_cost
            #########调试用
            global p_flag_
            if not p_flag_:
                p_flag_ = True
                print("-------------goal,speed,angle,dx----------",to_goal_cost,speed_cost,angle_cost,dx_cost)

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


def park_cb(park_flag_msg):
    # print("------------------in the park callback-----------------------")
    global park_flag
    park_flag = park_flag_msg
    global goal_flag
    goal_flag = True
    # print(goal_flag)
    # print("-----------park callback over------------------")
    print('-------In the park Callback---')


def cmdCB(qz_cmd_msg):
    global u0 , u1
    u0 = qz_cmd_msg.linear.x
    u1 = qz_cmd_msg.angular.z


Queue_=Queue.Queue()
def FlagCB(msg):
    if msg.data==1:
        Queue_.put(1)


def main():
    global amcl_x,amcl_y,amcl_yaw

    # dwa_control_pub = rospy.Publisher("/dwa_control" , Twist , Queue_size=1)
    dwa_control_twist = Twist()
    config = Config()

    #transformPose

    #goal的值固定死
    global goal1_x,goal1_y
    goal1 = np.array([goal1_x , goal1_y])
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp.secs = 0
    msg.header.frame_id = 'map'
    msg.pose.position.x = goal1[0]
    msg.pose.position.y = goal1[1]
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 1.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 0.0
    move_base_pub.publish(msg)
    # goal_temp = tf_listener.transformPose('odom' , park_flag)
    # goal = np.array([goal_temp.pose.position.x , goal_temp.pose.position.y])
    
    #x的速度和角速度可能要改
    x = np.array([amcl_x , amcl_y , amcl_yaw , 0 , 0])

    while not rospy.is_shutdown():
        u = np.array([u0, u1])
        u = dwa_control(x, u, config, goal1)
        dwa_control_twist.linear.x = u[0]
        dwa_control_twist.angular.z = u[1]
        dwa_control_pub.publish(dwa_control_twist)
        # print("--------------twist pub-----------",u[0],u[1])
        #通过定位更新x(x中的u[0],u[1]好像没用)
        x = np.array([amcl_x , amcl_y , amcl_yaw , u[0] , u[1]])

        if ((x[0] - goal1[0]) ** 2 + (x[1] - goal1[1]) ** 2) <= config.robot_radius ** 2:
            dwa_control_twist.linear.x = 0
            dwa_control_twist.angular.z = 0
            dwa_control_pub.publish(dwa_control_twist)
            print("------------Goal1 Reached!--------------")
            break
    

    #目标点写死，并且这个goal2根据park_flag文字识别结果变化（目前设定为p2）
    # goal2 = np.array([0.55 , 0.85])
    global goal2_x,goal2_y
    config.set_angle_cost_gain(2.0)
    goal2 = np.array([goal2_x , goal2_y])
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp.secs = 0
    msg.header.frame_id = 'map'
    msg.pose.position.x = goal2[0]
    msg.pose.position.y = goal2[1]
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 1.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 0.0
    move_base_pub.publish(msg)
    #x的速度和角速度可能要改
    x = np.array([amcl_x , amcl_y , amcl_yaw , 0 , 0])

    #初始速度和角速度可能要改
    u = np.array([-config.max_speed, 0.0])

    while not rospy.is_shutdown():
        u = dwa_control(x, u, config, goal2)
        dwa_control_twist.linear.x = u[0]
        dwa_control_twist.angular.z = u[1]
        dwa_control_pub.publish(dwa_control_twist)
        u = np.array([u0, u1])
        #通过定位更新x
        x = np.array([amcl_x , amcl_y , amcl_yaw , u[0] , u[1]])

        if ((x[0] - goal2[0]) ** 2 + (x[1] - goal2[1]) ** 2) <= config.robot_radius ** 2:
            dwa_locate_pub.publish(5)
            dwa_control_twist.linear.x = 0
            dwa_control_twist.angular.z = 0
            dwa_control_pub.publish(dwa_control_twist)
            print("------------Goal2 Reached!--------------")
            break


if __name__ == '__main__':
    rospy.init_node("DWA")
    #/dwa_cmd_vel
    dwa_control_pub = rospy.Publisher("/qz_cmd_vel" , Twist , queue_size=1)#dwa速度消息发布者
    dwa_locate_pub = rospy.Publisher("/qingzhou_locate" , Int32 , queue_size=1)
    move_base_pub = rospy.Publisher("/move_base_simple/goal" , PoseStamped , queue_size=1)
    tf_listener = tf.TransformListener()
    amcl_sub = rospy.Subscriber("/amcl_pose" , PoseWithCovarianceStamped , amcl_cb, queue_size=1)
    park_sub = rospy.Subscriber("/park_flag" , Int32 , park_cb, queue_size=1)#接收停车点的位置P1或P2
    locate_sub=rospy.Subscriber("/dwa_flag" , Int32 , FlagCB)#开启dwa算法的接收者
    qz_cmd_sub=rospy.Subscriber("/qz_cmd_vel" , Twist , cmdCB)
    print("--------------------")
    while not rospy.is_shutdown():
        queue_out=Queue_.get()
        print("----------queue get success----------")
        if queue_out==1:
            main()


