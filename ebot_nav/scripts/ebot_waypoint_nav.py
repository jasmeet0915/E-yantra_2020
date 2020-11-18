#! /usr/bin/env python
import rospy
import numpy as np

# import actionlib & actionlib_msgs to create a SimpleActionClient object 
from actionlib import SimpleActionClient, SimpleGoalState
from actionlib_msgs.msg import GoalStatus 

# import geometry_msgs to create Pose objects with goal coords
from geometry_msgs.msg import Pose, Point, Quaternion

# import nav_msgs for Odometry
from nav_msgs.msg import Odometry

# import quaternion_from_euler function
from tf.transformations import quaternion_from_euler

# import the move_base messages to send goals 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

pose = []

class NavWaypoints():

    def __init__(self):
        
        goals = [(-9.1, -1.2),
                 (10.7, 10.5),
                 (12.3, -1.4),
                 (18.2, -1.4),
                 (-2, 4)]
        yaw = [0, 180, 0, 0, 0]
        self.waypoints = list()

        # append waypoints list with goal Pose & Quaternions
        for (x, y) in goals:
            n = 0
            self.waypoints.append(Pose(Point(x, y, 0), Quaternion(*(quaternion_from_euler(0, 0, yaw[n]*np.pi/180, axes='sxyz')))))
            n = n+1
        self.curr = 0
    
    def get_waypoint(self):
        next_waypoint = self.waypoints[self.curr]
        self.curr = self.curr + 1
        self.curr_waypoint = self.waypoints[self.curr-1]
        return next_waypoint 

    def get_curr_goal(self):
        return self.curr_waypoint


class MoveBase():

    def __init__(self):

        # create ROS node named 'ebot_nav'
        rospy.init_node('ebot_nav')

        rospy.Subscriber('/odom', Odometry, odom_callback)

        # create NavWaypoints object to supply waypoints
        self.waypoint_supplier = NavWaypoints()

        # create SimpleActionClient object named 'move_base'
        self.action_client = SimpleActionClient('move_base', MoveBaseAction)

        # wait for action server to start
        self.action_client.wait_for_server()

        # get teh initial waypoint an call move_to() for it
        initial_waypoint = self.waypoint_supplier.get_waypoint()
        rospy.loginfo(initial_waypoint)
        self.move_to(initial_waypoint)
        
    def move_to(self, goal):

        # initialize a MoveBaseGoal object to create a message
        goal_msg = MoveBaseGoal()

        # set message headers with frame and time
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        # set the goal coordinates in message
        goal_msg.target_pose.pose = goal

        # send the message to Action Server in move_base node
        self.action_client.send_goal(goal_msg)
        rospy.loginfo("next goal sent")
        rospy.loginfo(goal)
        self.action_client._set_simple_state(SimpleGoalState.ACTIVE)


def odom_callback(data):
    global pose
    pose = [data.pose.pose.position.x, data.pose.pose.position.y]

                
def control_loop():
    move_base = MoveBase()
    rate = rospy.Rate(10)


    while True:
        global pose
        curr_goal = move_base.waypoint_supplier.get_curr_goal()
        rospy.loginfo("waypoint from waypoint supplier: " + str(round(curr_goal.position.x))+","+str(round(curr_goal.position.y)))
        rospy.loginfo("waypoint from odom callback: " + str(round(np.abs(pose[0]), 2)) + "," + str(round(np.abs(pose[1]), 2)))
        position_error = np.sqrt(pow(curr_goal.position.y - pose[1], 2) + pow(curr_goal.position.x - pose[0], 2))
        rospy.loginfo("position error: " + str(position_error))

        if position_error < 0.6:
            move_base.move_to(move_base.waypoint_supplier.get_waypoint())
        rate.sleep() 


if __name__ == "__main__":
    try:
        control_loop() 
    except rospy.ROSInterruptException:
        pass
