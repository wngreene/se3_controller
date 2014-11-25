#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench


def wrenchCallback(data):
    '''
    Wrench callback.
    '''
    print "I received a Wrench message!"


def main():
    '''
    Main method.
    '''
    rospy.init_node('se3_controller')

    rospy.Subscriber("wrench_out", Wrench, wrenchCallback)

    rospy.spin()

    return


if __name__ == '__main__':
    main()
