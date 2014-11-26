#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header


class SE3Controller(object):
    """SE3 Controller for Quadrotors.

    Based on Lee et al. 2010.
    """
    def __init__(self):
        super(SE3Controller, self).__init__()

        self.curr_state = None
        self.state_subscriber = rospy.Subscriber("/ground_truth/state",
                                                 Odometry, self.stateCallback)
        self.twist_publisher = rospy.Publisher("command/twist", TwistStamped)

        return

    def stateCallback(self, data):
        """ Callback for current state.

        Callback for /ground_truth/state.
        """
        self.curr_state = data

        self.update()

        return

    def update(self):
        twist_cmd = TwistStamped()
        twist_cmd.header = Header()
        twist_cmd.header.stamp = rospy.Time.now()

        twist_cmd.twist.linear.x = 0
        twist_cmd.twist.linear.y = 0

        error = (self.curr_state.pose.pose.position.z - 1)
        twist_cmd.twist.linear.z = -1.0*error

        twist_cmd.twist.angular.x = 0
        twist_cmd.twist.angular.y = 0
        twist_cmd.twist.angular.z = 0

        rospy.loginfo("Twist = %s", str(twist_cmd))
        self.twist_publisher.publish(twist_cmd)

        return


def main():
    '''
    Main method.
    '''
    print "Running se3_controller..."

    rospy.init_node('se3_controller')

    se3_controller = SE3Controller()

    rospy.spin()

    return


if __name__ == '__main__':
    main()
