#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped



if __name__ == "__main__":
    rospy.init_node("qingzhou_reset")
    broadcaster = tf2_ros.TransformBroadcaster()
    #         4-2.创建 广播的数据(通过 pose 设置)
    tfs1 = TransformStamped()
    tfs1.header.frame_id = "map"
    tfs1.header.stamp = rospy.Time.now()
    tfs1.child_frame_id = "odom"
    tfs1.transform.translation.x = 0.0
    tfs1.transform.translation.y = 0.0
    tfs1.transform.translation.z = 0.0
    qtn1 = tf.transformations.quaternion_from_euler(0,0,0)
    tfs1.transform.rotation.x = qtn1[0]
    tfs1.transform.rotation.y = qtn1[1]
    tfs1.transform.rotation.z = qtn1[2]
    tfs1.transform.rotation.w = qtn1[3]
    #         4-3.广播器发布数据
    broadcaster.sendTransform(tfs1)
    rospy.logerr("[RESET]---ODOM IS RESETED!")

    tfs2 = TransformStamped()
    tfs2.header.frame_id = "odom"
    tfs2.header.stamp = rospy.Time.now()
    tfs2.child_frame_id = "base_link"
    tfs2.transform.translation.x = 0.0
    tfs2.transform.translation.y = 0.0
    tfs2.transform.translation.z = 0.0
    qtn2 = tf.transformations.quaternion_from_euler(0,0,0)
    tfs2.transform.rotation.x = qtn2[0]
    tfs2.transform.rotation.y = qtn2[1]
    tfs2.transform.rotation.z = qtn2[2]
    tfs2.transform.rotation.w = qtn2[3]
    #         4-3.广播器发布数据
    broadcaster.sendTransform(tfs2)
    rospy.logerr("[RESET]---BASE_LINK IS RESETED!")


