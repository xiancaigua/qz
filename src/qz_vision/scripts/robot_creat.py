#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

from ShiftFrom2D import *
from openvideo import *
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf


theta = 0

def amcl_cb(msg):
    global theta
    msg = PoseWithCovarianceStamped()
    w = msg.pose.pose.orientation.w
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    (r,p,y) = tf.transformations.euler_from_quaternion([x,y,z,w])
    theta = y

if __name__ == "__main__":
    rospy.init_node("robot_create")

    amcl_pose_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,amcl_cb)

    cam=cv2.VideoCapture(gstreamer_pipeline(flip_method=0),cv2.CAP_GSTREAMER)
    while not rospy.is_shutdown:
        while cam.isOpened():
            ret,frame = cam.read()
            if ret:
                result = getPoint(frame,theta)
                
