#!/usr/bin/env python

import numpy as np

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header

from hector_uav_msgs.msg import MotorCommand, Supply

from python_utils import Geometry


class Trajectory(object):
    """Trajectory class.

    Class that represents a desired trajectory.
    """
    def __init__(self, x_des, v_des, fwd_des, w_des, wait_time, period_time):
        self.x_des = x_des
        self.v_des = v_des
        self.fwd_des = fwd_des
        self.w_des = w_des
        self.wait_time = wait_time
        self.period_time = period_time

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

        self.update_rate = 100
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
        self.goal_pub = rospy.Publisher("/goal_pose", PoseStamped,
                                        queue_size=10)

        # Gains.
        self.k_x = 4.0 * self.mass  # 1.0
        self.k_v = 2.5 * self.mass  # 0.7
        self.k_R = 15.0
        self.k_w = 20.0

        # Publish full path.
        self.path = Path()
        self.path.header = Header()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"
        t = np.linspace(self.trajectory.wait_time, trajectory.wait_time +
                        trajectory.period_time, 100)
        for ii in range(len(t)):
            x_des = trajectory.position(t[ii])

            pose_ii = PoseStamped()
            pose_ii.header = Header()
            pose_ii.header.stamp = rospy.Time.from_sec(self.path.header.stamp.to_sec() + t[ii] -
                                                       trajectory.wait_time)

            pose_ii.pose.position.x = x_des[0]
            pose_ii.pose.position.y = x_des[1]
            pose_ii.pose.position.z = x_des[2]

            self.path.poses.append(pose_ii)

        self.goal_path_pub = rospy.Publisher("/goal_path", Path,
                                             queue_size=10)
        self.goal_path_pub_factor = 100

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

    def getGoalState(self, state):
        """Compute goal state (note depends on current state). """

        msg_time = state.header.stamp
        t = (msg_time - self.time_start).to_sec()

        x_des = self.trajectory.position(t)
        v_des = self.trajectory.velocity(t)
        fwd_des = self.trajectory.forward(t)
        w_des = self.trajectory.angularVelocity(t)

        # Compute R_des.
        q_curr = Geometry.Quaternion(state.pose.pose.orientation.x,
                                     state.pose.pose.orientation.y,
                                     state.pose.pose.orientation.z,
                                     state.pose.pose.orientation.w)
        R_curr = q_curr.getRotationMatrix()

        up = R_curr[:, 2]
        right_des = np.cross(fwd_des, up)
        right_des /= np.linalg.norm(right_des)
        proj_fwd_des = np.cross(up, right_des)

        R_des = np.zeros((3, 3))
        R_des[:, 0] = right_des
        R_des[:, 1] = proj_fwd_des
        R_des[:, 2] = up

        return x_des, v_des, R_des, w_des

    def getErrors(self, state):
        """Compute current state errors. """

        # Desired states.
        x_des, v_des, R_des, w_des = self.getGoalState(state)

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

        e_R = 0.5 * Geometry.veemap(np.dot(R_des.T, R_curr) -
                                    np.dot(R_curr.T, R_des))

        # Angular velocity.
        e_w = np.zeros(3)
        w_curr = np.zeros(3)
        w_curr[0] = state.twist.twist.angular.x
        w_curr[1] = state.twist.twist.angular.y
        w_curr[2] = state.twist.twist.angular.z

        e_w = w_curr - np.dot(R_curr.T, np.dot(R_des, w_des))

        # rospy.loginfo("R_curr = \n%s", str(R_curr))
        # rospy.loginfo("R_des = \n%s", str(R_des))
        # rospy.loginfo("e_R = \n%s", str(e_R))

        return e_x, e_v, e_R, e_w

    def publishGoal(self, state):
        # Desired states.
        x_des, v_des, R_des, w_des = self.getGoalState(state)

        q_des = Geometry.Quaternion()
        q_des.setFromRotationMatrix(R_des)

        pose_msg = PoseStamped()
        pose_msg.header = state.header

        pose_msg.pose.position.x = x_des[0]
        pose_msg.pose.position.y = x_des[1]
        pose_msg.pose.position.z = x_des[2]

        pose_msg.pose.orientation.x = q_des.x
        pose_msg.pose.orientation.y = q_des.y
        pose_msg.pose.orientation.z = q_des.z
        pose_msg.pose.orientation.w = q_des.w

        self.goal_pub.publish(pose_msg)

        if pose_msg.header.seq % self.goal_path_pub_factor == 0:
            self.goal_path_pub.publish(self.path)

        return

    def twistUpdate(self):
        if self.curr_state:
            x_des, v_des, R_des, w_des = self.getGoalState(self.curr_state)
            e_x, e_v, e_R, e_w = self.getErrors(self.curr_state)

            twist_cmd = TwistStamped()
            twist_cmd.header = Header()
            twist_cmd.header.stamp = rospy.Time.now()

            twist_cmd.twist.linear.x = -self.k_x * e_x[0] - self.k_v * e_v[0] + v_des[0]
            twist_cmd.twist.linear.y = -self.k_x * e_x[1] - self.k_v * e_v[1] + v_des[1]
            twist_cmd.twist.linear.z = -self.k_x * e_x[2] - self.k_v * e_v[2] + v_des[2]

            twist_cmd.twist.angular.x = -self.k_R * e_R[0] - self.k_w * e_w[0] + w_des[0]
            twist_cmd.twist.angular.y = -self.k_R * e_R[1] - self.k_w * e_w[1] + w_des[1]
            twist_cmd.twist.angular.z = -self.k_R * e_R[2] - self.k_w * e_w[2] + w_des[2]

            self.twist_pub.publish(twist_cmd)
            self.publishGoal(self.curr_state)

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

    def run(self):
        """Node mainloop.

        Receives messages and publishes commands.
        """
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.twistUpdate()
            r.sleep()

        return


class HorizontalCircle(object):
    """Horizontal circle trajectory."""
    def __init__(self, R, w, period_time, wait_time=20):
        super(HorizontalCircle, self).__init__()
        self.R = R
        self.w = w
        self.wait_time = wait_time
        self.period_time = period_time

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
            f = self.forward(t)
            fdot = np.array([-self.w*np.cos(self.w*tt),
                             -self.w*np.sin(self.w*tt),
                             0])

            ret = np.cross(f, fdot)

        return ret


class Lissajous(object):
    """Lissajous trajectory."""
    def __init__(self, A, B, C, D, a, b, c, period_time, wait_time=10):
        super(Lissajous, self).__init__()
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.a = a
        self.b = b
        self.c = c
        self.wait_time = wait_time
        self.period_time = period_time

    def position(self, t):
        ret = None
        tt = t - self.wait_time

        if t < self.wait_time:
            ret = np.array([self.A, 0, self.D])
        else:
            ret = np.array([self.A * np.cos(self.a * tt),
                            self.B * np.sin(self.b * tt),
                            self.C * np.sin(self.c * tt) + self.D])
        return ret

    def velocity(self, t):
        ret = None
        tt = t - self.wait_time

        if t < self.wait_time:
            ret = np.array([0, 0, 0])
        else:
            ret = np.array([-self.A * self.a * np.sin(self.a * tt),
                            self.B * self.b * np.cos(self.b * tt),
                            self.C * self.c * np.cos(self.c * tt)])
        return ret

    def forward(self, t):
        ret = None

        if t < self.wait_time:
            ret = np.array([0, 1, 0])
        else:
            v = self.velocity(t)
            ret = v/np.linalg.norm(v)

        return ret

    def angularVelocity(self, t):
        ret = None
        dt = 1e-2

        if t < self.wait_time:
            ret = np.zeros(3)
        else:
            f = self.forward(t)
            fdot = (self.forward(t + dt) - self.forward(t - dt)) / (2*dt)
            ret = np.cross(f, fdot)

        return ret


def main():
    """Main method.

    Runs SE3 controller on a predetermined trajectory.
    """

    try:
        print "Running se3_controller..."

        # traj = HorizontalCircle(6, 2*np.pi/9, 9, 9)
        traj = Lissajous(6, 6, 4, 5, 3*2*np.pi/25.0, 2*np.pi/12.5, 2*np.pi/25, 25)
        trajectory = Trajectory(traj.position,
                                traj.velocity,
                                traj.forward,
                                traj.angularVelocity,
                                traj.wait_time,
                                traj.period_time)

        se3_controller = SE3Controller(trajectory)
        se3_controller.run()

    except rospy.ROSInterruptException:
        pass

    return


if __name__ == '__main__':
    main()
