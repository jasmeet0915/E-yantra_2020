#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np

state = 0
pose = []

# create velocity_msg Twist object to publish messages to /cmd_vel topic
velocity_msg = Twist()

# declare a global publisher variable
pub = None


def waypoints(res):

    # create a numpy array of 'res' points in range 0 to 2*pi
    x = np.linspace(0, 2*np.pi, num = res, endpoint=True)[1:]

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


def fix_yaw(error, P):
    global velocity_msg, pub

    velocity_msg.linear.x = 0.1 
    velocity_msg.angular.z = P * -error
    pub.publish(velocity_msg)


def move_straight(error, P):
    global velocity_msg
    
    velocity_msg.angular.z = 0
    velocity_msg.linear.x = P * error
    
    pub.publish(velocity_msg)


def goto(dest_x, dest_y):
    global state, pose

    theta_precision = 0.5  
    dist_precision = 0.6


    while state != 2:

        theta_goal = np.arctan((dest_y - pose[1])/(dest_x - pose[0]))
        bot_theta = pose[2]

        theta_error = round(bot_theta - theta_goal, 2)
        rospy.loginfo("STATE: " + str(state))
        rospy.loginfo("THETA ERROR:" + str(theta_error))

        if state == 0:
            if np.abs(theta_error) > theta_precision:   
                rospy.loginfo("Fixing Yaw")
                fix_yaw(theta_error, 0.5)
            else:
                rospy.loginfo("Yaw Fixed! Moving Towards Goal Now")
                state = 1

        elif state == 1:
            position_error = np.sqrt(pow(dest_y - pose[1], 2) + pow(dest_x - pose[0], 2))
            rospy.loginfo("POSITION ERROR: " + str(position_error))

            if position_error > dist_precision and np.abs(theta_error) < theta_precision:
                rospy.loginfo("Moving Straight")
                move_straight(position_error, 0.1)
            elif np.abs(theta_error) > theta_precision: 
                rospy.loginfo("Going out of line!")
                state = 0

            else:
                rospy.loginfo("GOAL REACHED")
                state = 2


def control_loop():
    global pub, velocity_msg, state

    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    # publish 0 linear.x and 0 angular.z velocity to stop the bot initially
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    # get the list of x and y coords for the waypoints
    path_x, path_y = waypoints(10)

    path_waypoints = zip(path_x, path_y)
    rospy.loginfo(path_waypoints)
    #path_waypoints = np.delete(path_waypoints, 0)

    while not rospy.is_shutdown():

        if pose:
            for (x, y) in path_waypoints:
                state = 0
                rospy.loginfo("Moving to point: " + str(x) + "," + str(y))
                goto(round(x, 2), round(y, 2))

        rate.sleep()
        

if __name__ == "__main__":
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
