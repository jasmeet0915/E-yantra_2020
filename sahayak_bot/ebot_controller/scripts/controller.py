#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np

# create a global state variable to store the current state of the goto state machine
state = 0

# global pose variable to store bot pose data
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
    global regions
    regions = {
        'bright': min(msg.ranges[0:143]),
        'fright': min(msg.ranges[144:287]),
        'front':  min(msg.ranges[288:431])	,
        'fleft':  min(msg.ranges[432:575])	,
        'bleft':  min(msg.ranges[576:713])	,
    }


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


# function to orient the bot towards the destination using Proportional controller
def fix_yaw(error_a, P):
    global velocity_msg, pub

    velocity_msg.linear.x = 0.2 * np.abs(error_a) 
    velocity_msg.angular.z = P * -error_a

    pub.publish(velocity_msg)


# function to move on a strainght line towards the goal using Proportional controller
def move_straight(error, P):
    global velocity_msg
    
    velocity_msg.angular.z = 0
    velocity_msg.linear.x = P * error
    
    pub.publish(velocity_msg)


'''
    This function moves the bot towards the goal coordinates.
    The function uses a state machine with three states: 
    1) state = 0; fixing yaw 2) state = 1; moving straight 3) state = 2; goal reached 
'''
def goto(dest_x, dest_y):
    global state, pose

    # the required precision va;ues for required theta and distance from goal
    theta_precision = 0.16  
    dist_precision = 0.35


    # while current state is not 2 (goal is not reached)
    while state != 2:

        theta_goal = np.arctan((dest_y - pose[1])/(dest_x - pose[0]))
        if theta_goal>0:
        	theta_goal+=0.04
        elif theta_goal<0:
        	theta_goal-=0.04
        bot_theta = pose[2]

        theta_error = round(bot_theta - theta_goal, 2)
        rospy.loginfo("STATE: " + str(state))
        rospy.loginfo("THETA ERROR:" + str(theta_error))

        # if current state is 0 means bot is not correctly oriented
        if state == 0:

            # if theta_error is greated than the required precision then fix the yaw by rotating the bot
            # if required precision is reached then change current state to 1

            if np.abs(theta_error) > theta_precision:   
                rospy.loginfo("Fixing Yaw")
                fix_yaw(theta_error, 1.7)
            else:
                rospy.loginfo("Yaw Fixed! Moving Towards Goal Now")
                state = 1

        # if current state is 1 means the bot is correctly oriented so it move towards goal
        elif state == 1:

            # calculate error w.r.t to destination
            position_error = np.sqrt(pow(dest_y - pose[1], 2) + pow(dest_x - pose[0], 2))
            rospy.loginfo("POSITION ERROR: " + str(position_error))

            # if position error is less than required precision & bot is facing the goal, move towards goal in straight line
            # else if it is not correctly oriented change state to 1
            # if theta_precision and dist_precision are reached change state to 2 (goal reached)
            if position_error > dist_precision and np.abs(theta_error) < theta_precision:
                rospy.loginfo("Moving Straight")
                move_straight(position_error, 0.2)
            elif np.abs(theta_error) > theta_precision: 
                rospy.loginfo("Going out of line!")
                state = 0
            else:
                rospy.loginfo("GOAL REACHED")
                state = 2


def control_loop():
    global pub, velocity_msg, state

    rospy.init_node('ebot_controller')

    # initialize publisher to publish to /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # subscribe to /odom and /ebot/laser/scan
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    # publish 0 linear.x and 0 angular.z velocity to stop the bot initially
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    # get the list of 10 x and y coords for the waypoints
    path_x, path_y = waypoints(50)

    path_waypoints = zip(path_x, path_y)
    rospy.loginfo(path_waypoints)

    while not rospy.is_shutdown():

        # is global pose list is not empty
        if pose:

            # loop to move the bot to every point in the path_waypoints list
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
