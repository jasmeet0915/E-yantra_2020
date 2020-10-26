#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np


def waypoints(res):

    # create a numpy array of 'res' points in range 0 to 2*pi
    x = np.linspace(0, 2*np.pi, num = res, endpoint=True)

    # create a numpy array of given path function 
    y = 2 * np.sin(x) * np.sin(x/2)

    return [x, y]


def laser_callback(msg):
    pass


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    # create velocity_msg Twist object to publish messages to /cmd_vel topic
    velocity_msg = Twist()

    # publish 0 linear.x and 0 angular.z velocity to stop the bot initially
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    # get the list of x and y coords for the waypoints
    path_x, path_y = waypoints(50)

    rospy.loginfo(path_x)
    rospy.loginfo(path_y)

    path_waypoints = zip(path_x, path_y)
    rospy.loginfo(path_waypoints)


    while not rospy.is_shutdown():

        for (x, y) in path_waypoints:
            rospy.loginfo(str(x) + ", " + str(y))
            velocity_msg.linear.x = 0.1
            velocity_msg.angular.y = 0.1
            pub.publish(velocity_msg)

        break
        rate.sleep()
        

if __name__ == "__main__":
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
