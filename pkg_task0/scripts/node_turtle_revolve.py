#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

stop = 0


def pose_callback(msg):
    
    rospy.loginfo("theta: " + str(msg.theta))

    # round off msg.theta for easy comparing
    theta = round(msg.theta, 1)

    rospy.loginfo("Rounded off theta value: {}".format(theta))

    if msg.theta < 0 and theta == 0.0:
        rospy.loginfo("Goal Reached!!") 

        # update the value of global variable stop to 1
        global stop
        stop = 1


def main():

    # 1. Create a handle to publish Twist type messages to /turtle1/cmd_vel topic
    revl_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # initialize a ROS node named node_turtle_revolve
    rospy.init_node('node_turtle_revolve', anonymous=True)

    # Create a handle to subscribe to the /turtle1/cmd_vel topic
    rospy.Subscriber('turtle1/pose', Pose, pose_callback)

    # set a rate for publishing messages at 10hz
    pub_rate = rospy.Rate(10)

    # 3. create Twist message object
    revl_msg = Twist()

    while not rospy.is_shutdown():
    
        # check if the turtle has reached the starting point and break the loop
        if stop == 1:
            rospy.loginfo("CIRCLE COMPLETED")
            break
        
        # set the x linear vel as 0.6 & z angular vel as 0.8
        revl_msg.linear.x = 0.6 
        revl_msg.angular.z = 0.8

        # publish the created revl_msg to the topic turtle1/cmd_vel
        revl_pub.publish(revl_msg)

        pub_rate.sleep()
    
    # stop the turlte when it has reached the starting position
    revl_msg.linear.x = 0
    revl_msg.angular.z = 0
    revl_pub.publish(revl_msg)

    # wait for the user to stop the node by pressing Ctrl+c
    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

