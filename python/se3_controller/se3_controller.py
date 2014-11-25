#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench


def wrenchCallback(data):
    '''
    Wrench callback.
    '''
    rospy.loginfo("Force = %s", str(data))

    return


def main():
    '''
    Main method.
    '''
    print "Running se3_controller..."

    rospy.init_node('se3_controller')

    rospy.Subscriber("wrench_out", Wrench, wrenchCallback)

    rospy.spin()

    return


if __name__ == '__main__':
    main()
