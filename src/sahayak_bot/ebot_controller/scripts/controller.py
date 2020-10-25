import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import math


def Waypoints(t):
    pass


def laser_callback(msg):
    pass


def odom_callback(data):
    pass


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


    while not rospy.is_shutdown():
        rate.sleep()
        

if __name__ == "__main__":
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
