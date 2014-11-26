#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry


class SE3Controller(object):
    """SE3 Controller for Quadrotors.

    Based on Lee et al. 2010.
    """
    def __init__(self):
        super(SE3Controller, self).__init__()

        self.curr_state = None

        return

    def stateCallback(self, data):
        """ Callback for current state.

        Callback for /ground_truth/state.
        """
        self.curr_state = data

        self.update()

        return

    def update(self):
        rospy.loginfo("State = %s", str(self.curr_state))

        return


def main():
    '''
    Main method.
    '''
    print "Running se3_controller..."

    rospy.init_node('se3_controller')

    se3_controller = SE3Controller()
    rospy.Subscriber("/ground_truth/state",
                     Odometry, se3_controller.stateCallback)

    rospy.spin()

    return


if __name__ == '__main__':
    main()
