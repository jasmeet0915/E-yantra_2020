#!/usr/bin/env python

import rospy

def main():

    # make the script a ros node
    rospy.init_node('node_hello_ros', anonymous=True)

    # print hello world in the console
    rospy.loginfo("Hello World!")

    # keep the node alive till it is killed by the user
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

