#!/usr/bin/env python

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from hector_uav_msgs.msg import MotorCommand, Supply

from python_utils import Geometry


class ForwardFacingTrajectory(object):
    """Trajectory class for forward facing trajectories.

    Class that represents a desired trajectory where the desired forward vector
    lies along the desired velocity vector.
    """
    def __init__(self, x_des):
        self.x_des = x_des

        return

    def position(self, t):
        return self.x_des(t)

    def velocity(self, t, dt=1e-2):
        return (self.x_des(t-dt) - self.x_des(t+dt))/dt

    def forward(self, t, dt=1e-2):
        v = self.velocity(t, dt)
        return v/np.linalg.norm(v)


class SE3Controller(object):
    """SE3 Controller for Quadrotors.

    Based on Lee et al. 2010.
    """
    def __init__(self, trajectory):
        rospy.init_node('se3_controller')

        self.update_rate = 30
        self.time_start = rospy.Time.now()
        self.trajectory = trajectory
        self.curr_state = None
        self.max_voltage = None

        # Subscribers.
        self.state_sub = rospy.Subscriber("/ground_truth/state",
                                          Odometry, self.stateCallback)
        self.motor_supply_sub = rospy.Subscriber("/supply", Supply,
                                                 self.supplyCallback)

        # Publishers.
        self.twist_pub = rospy.Publisher("/command/twist", TwistStamped,
                                         queue_size=10)
        self.motor_pub = rospy.Publisher("/command/motor", MotorCommand,
                                         queue_size=10)

        # Gains.
        self.k_x = 1.0
        self.k_v = 1.0
        self.k_R = 1.0
        self.k_w = 1.0

        return

    def stateCallback(self, data):
        """Callback for current state.

        Callback for /ground_truth/state.
        """
        self.curr_state = data

        return

    def supplyCallback(self, data):
        """Call back for motor supply.

        Callback for /supply.
        """
        self.max_voltage = data.voltage[0]

        return

    def getErrors(self, state):
        msg_time = self.curr_state.header.stamp
        t = (msg_time - self.time_start).to_sec()

        # Desired states.
        x_des = self.trajectory.position(t)
        v_des = self.trajectory.velocity(t)
        fwd_des = self.trajectory.forward(t)

        # Errors.
        e_x = np.zeros(3)
        e_v = np.zeros(3)
        e_R = np.zeros(3)
        e_w = np.zeros(3)

        e_x[0] = state.pose.pose.position.x - x_des[0]
        e_x[1] = state.pose.pose.position.y - x_des[1]
        e_x[2] = state.pose.pose.position.z - x_des[2]

        e_v[0] = state.twist.twist.linear.x - v_des[0]
        e_v[1] = state.twist.twist.linear.y - v_des[1]
        e_v[2] = state.twist.twist.linear.z - v_des[2]

        q_curr = Geometry.Quaternion(state.pose.pose.orientation.x,
                                     state.pose.pose.orientation.y,
                                     state.pose.pose.orientation.z,
                                     state.pose.pose.orientation.w)
        R_curr = q_curr.getRotationMatrix()

        return e_x, e_v, e_R, e_w

    def run(self):
        """Node mainloop.

        Receives messages and publishes commands.
        """
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.twistUpdate()
            r.sleep()

        return

    def twistUpdate(self):
        if self.curr_state:
            e_x, e_v, e_R, e_w = self.getErrors(self.curr_state)

            twist_cmd = TwistStamped()
            twist_cmd.header = Header()
            twist_cmd.header.stamp = rospy.Time.now()

            twist_cmd.twist.linear.x = -self.k_x * e_x[0]
            twist_cmd.twist.linear.y = -self.k_x * e_x[1]
            twist_cmd.twist.linear.z = -self.k_x * e_x[2]

            twist_cmd.twist.angular.x = 0
            twist_cmd.twist.angular.y = 0
            twist_cmd.twist.angular.z = 0

            rospy.logdebug("Twist = %s", str(twist_cmd))
            self.twist_pub.publish(twist_cmd)

        return

    def voltageUpdate(self):
        if self.max_voltage:
            motor_cmd = MotorCommand()
            motor_cmd.header = Header()
            motor_cmd.header.stamp = rospy.Time.now()

            for ii in range(0, 4):
                motor_cmd.voltage.append(1 * self.max_voltage)

            self.motor_pub.publish(motor_cmd)

        return


def main():
    """Main method.

    Runs SE3 controller on a predetermined trajectory.
    """

    try:
        print "Running se3_controller..."
        R = 4
        w = 2*np.pi/5.0
        x_des = lambda t, R=R, w=w: np.array([R*np.cos(w*t), R*np.sin(w*t), 5])
        trajectory = ForwardFacingTrajectory(x_des)
        se3_controller = SE3Controller(trajectory)
        se3_controller.run()

    except rospy.ROSInterruptException:
        pass

    return


if __name__ == '__main__':
    main()
