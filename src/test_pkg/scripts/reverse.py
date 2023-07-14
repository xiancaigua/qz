import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseWithCovarianceStamped

def amcl_callback(amcl_msg):
    global odom
    odom = amcl_msg

if __name__=="__main__":
    #调试的时候是x
    rospy.init_node("reverse")
    reverse_pub=rospy.Publisher("/qz_cmd_vel_l1",AckermannDrive,queue_size=1)
    amcl_sub=rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,amcl_callback)
    limit_min = 0.2613
    limit_max = 0.2813
    goal_x_min = 0.9
    goal_x_max = 1.0
    goal_y_min = -0.957
    goal_y_max = -0.757
    ackermann_cmd = AckermannDrive()
    ackermann_cmd.speed = 0.3
    ackermann_cmd.steering_angle = 0.0
    reverse_pub.publish(ackermann_cmd)
    if odom.pose.pose.position.x > limit_min and odom.pose.pose.position.x < limit_max:
        ackermann_cmd.steering_angle = 120.0
        ackermann_cmd.speed = -0.3
        reverse_pub.publish(ackermann_cmd)
    if odom.pose.pose.position.x > goal_x_min and odom.pose.pose.position.x < goal_x_max and odom.pose.pose.position.y > goal_y_min and odom.pose.pose.position.y < goal_y_max:
        ackermann_cmd.speed = 0.0
        ackermann_cmd.steering_angle = 0.0
        reverse_pub.publish(ackermann_cmd)
