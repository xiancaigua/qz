#!/usr/bin/env python

from cmd import Cmd
import rospy
import math
import curses
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Twist

msg = """
Control The qingzhou Robot!
---------------------------
Moving around:
        W     
   A    S    D

W/S : Forward and backward
A/D : Turn left and turn right

CTRL-C to quit
"""

# GLOBAL KEYS' DICTIONARY: Associates a keyboard key to a vector (x,y,z,theta)
vel_bindings = {
    'w': [3, 3, 3, 3],      # Forward 2m/s
    's': [-3, -3, -3, -3]   # Backward 2m/s
}

steer_bindings = {
    'a': 0.085,  # 10grad anti-clock-wise
    'd': -0.085  # 10grad clock-wise
}

# GLOBAL: MODEL GEOMETRY
L = 0.3  # Distance between wheels axes
W = 0.3  # Distance between wheels


# FUNCTION start_curses(): Start a curses terminal app
def start_curses():
    app = curses.initscr()  # Create a terminal window
    curses.noecho()         # Makes input invisible
    app.addstr(msg)         # Print the start message
    return app


# MAIN FUNCTION
def move():

    pub_vel = rospy.Publisher(
        '/qz_cmd_vel', Twist, queue_size=10)


    rospy.init_node('ackrm_robot_teleop', anonymous=True)


    rate = rospy.Rate(100)

    theta = 0.0
    linear = 0.0


    app = start_curses()


    while not rospy.is_shutdown():


        key = app.getkey()


        if key in vel_bindings.keys():
            if key == 'w':
                # rospy.loginfo("ws")
                linear += 0.1
            elif key == 's':
                linear -= 0.1

            if linear > 1:
                linear = 1
            elif linear < -1:
                linear = -1

        elif key in steer_bindings.keys():
            # theta = theta + steer_bindings[key]
            if key == 'a':
                theta+=0.17
            elif key== 'd':
                theta+=-0.17
                rospy.loginfo("ad")
            if theta > 0.51:
                theta = 0.51
            elif theta < -0.51:
                theta = -0.51
            


        else:
            # Incorrect key => Robot stop
            linear = 0.0
            theta = 0.0
            # q => exit from loop
            if (key == 'q'):
                curses.endwin()  # End Curses application
                break

     
        cmd = Twist()
        cmd.angular.z = theta
        cmd.linear.x = linear

        pub_vel.publish(cmd)


        rate.sleep()



if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
    except curses.error:
        curses.endwin()