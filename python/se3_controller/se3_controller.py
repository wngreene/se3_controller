#!/usr/bin/env python

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from hector_uav_msgs.msg import MotorCommand, Supply

from python_utils import Geometry


class Trajectory(object):
    """Trajectory class.

    Class that represents a desired trajectory.
    """
    def __init__(self, x_des, v_des, fwd_des, w_des):
        self.x_des = x_des
        self.v_des = v_des
        self.fwd_des = fwd_des
        self.w_des = w_des

        return

    def position(self, t):
        return self.x_des(t)

    def velocity(self, t):
        return self.v_des(t)

    def forward(self, t):
        return self.fwd_des(t)

    def angularVelocity(self, t):
        return self.w_des(t)


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
        self.mass = 1.477

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
        self.k_x = 1.0 * self.mass
        self.k_v = 1.0 * self.mass
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
        w_des = self.trajectory.angularVelocity(t)

        # Errors.
        e_x = np.zeros(3)
        e_v = np.zeros(3)
        e_R = np.zeros(3)
        e_w = np.zeros(3)

        # Position.
        e_x[0] = state.pose.pose.position.x - x_des[0]
        e_x[1] = state.pose.pose.position.y - x_des[1]
        e_x[2] = state.pose.pose.position.z - x_des[2]

        # Velocity.
        e_v[0] = state.twist.twist.linear.x - v_des[0]
        e_v[1] = state.twist.twist.linear.y - v_des[1]
        e_v[2] = state.twist.twist.linear.z - v_des[2]

        # Orientation.
        q_curr = Geometry.Quaternion(state.pose.pose.orientation.x,
                                     state.pose.pose.orientation.y,
                                     state.pose.pose.orientation.z,
                                     state.pose.pose.orientation.w)
        R_curr = q_curr.getRotationMatrix()

        up = R_curr[:, 2]
        right_des = np.cross(fwd_des, up)
        right_des /= np.linalg.norm(right_des)
        proj_fwd_des = np.cross(right_des, up)

        R_des = np.zeros((3, 3))
        R_des[:, 0] = right_des
        R_des[:, 1] = proj_fwd_des
        R_des[:, 2] = up

        e_R = 0.5 * Geometry.veemap(np.dot(R_des.T, R_curr) -
                                    np.dot(R_curr.T, R_des))

        # Angular velocity.
        e_w = np.zeros(3)
        w_curr = np.zeros(3)
        w_curr[0] = state.twist.twist.angular.x
        w_curr[1] = state.twist.twist.angular.y
        w_curr[2] = state.twist.twist.angular.z

        e_w = w_curr - np.dot(R_curr.T, np.dot(R_des, w_des))

        rospy.loginfo("R_curr = %s", str(R_curr))

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

            twist_cmd.twist.linear.x = -self.k_x * e_x[0] - self.k_v * e_v[0]
            twist_cmd.twist.linear.y = -self.k_x * e_x[1] - self.k_v * e_v[1]
            twist_cmd.twist.linear.z = -self.k_x * e_x[2] - self.k_v * e_v[2]

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


class HorizontalCircle(object):
    """Horizontal circle trajectory."""
    def __init__(self, R=4, w=2*np.pi/10, wait_time=30):
        super(HorizontalCircle, self).__init__()
        self.R = R
        self.w = w
        self.wait_time = wait_time

    def position(self, t):
        ret = None
        tt = t - self.wait_time

        if t < self.wait_time:
            ret = np.array([self.R, 0, 5])
        else:
            ret = np.array([self.R*np.cos(self.w*tt),
                            self.R*np.sin(self.w*tt),
                            5])
        return ret

    def velocity(self, t):
        ret = None
        tt = t - self.wait_time

        if t < self.wait_time:
            ret = np.array([0, 0, 0])
        else:
            ret = np.array([-self.R*self.w*np.sin(self.w*tt),
                            self.R*self.w*np.cos(self.w*tt),
                            0])
        return ret

    def forward(self, t):
        ret = None
        tt = t - self.wait_time

        if t < self.wait_time:
            ret = np.array([0, 1, 0])
        else:
            ret = np.array([-np.sin(self.w * tt),
                            np.cos(self.w * tt),
                            0])

        return ret

    def angularVelocity(self, t):
        ret = None
        tt = t - self.wait_time

        if t < self.wait_time:
            ret = np.zeros(3)
        else:
            f = np.array([-np.sin(self.w*tt),
                          np.cos(self.w*tt),
                          0])
            fdot = np.array([-self.w*np.cos(self.w*tt),
                             -self.w*np.sin(self.w*tt),
                             0])

            ret = np.cross(f, fdot)

        return ret


def main():
    """Main method.

    Runs SE3 controller on a predetermined trajectory.
    """

    try:
        print "Running se3_controller..."

        hcircle = HorizontalCircle()
        trajectory = Trajectory(hcircle.position,
                                hcircle.velocity,
                                hcircle.forward,
                                hcircle.angularVelocity)

        se3_controller = SE3Controller(trajectory)
        se3_controller.run()

    except rospy.ROSInterruptException:
        pass

    return


if __name__ == '__main__':
    main()
