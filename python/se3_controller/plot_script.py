#!/usr/bin/env python

import sys

import numpy as np
from matplotlib import lines
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rosbag
import tf.transformations as trans


def loadData(bag_file):
    """Load and parse bag files."""

    # Load bag files.
    bag = rosbag.Bag(bag_file)

    true_topic = '/ground_truth_to_tf/pose'
    goal_topic = '/goal_pose'

    # Extract pose info.
    pose = None
    true_pose = {'time': [], 'seq': [], 'position': [], 'orientation': []}
    goal_pose = {'time': [], 'seq': [], 'position': [], 'orientation': []}
    for topic, msg, t in bag.read_messages(topics=[true_topic, goal_topic]):
        if topic == true_topic:
            pose = true_pose
        else:
            pose = goal_pose

        pose['time'].append(msg.header.stamp.to_sec())
        pose['seq'].append(msg.header.seq)
        pose['position'].append([msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z])
        pose['orientation'].append([msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w])

    for key in true_pose:
        true_pose[key] = np.asarray(true_pose[key])
        goal_pose[key] = np.asarray(goal_pose[key])

    # Remove startup time (assume 10 seconds).
    start_time = 10
    true_start_idx = np.argmin(np.abs(true_pose['time'] - start_time))
    goal_start_idx = np.argmin(np.abs(goal_pose['time'] - start_time))

    for key in ['time', 'seq']:
        true_pose[key] = true_pose[key][true_start_idx:]
        goal_pose[key] = goal_pose[key][goal_start_idx:]

    for key in ['position', 'orientation']:
        true_pose[key] = true_pose[key][true_start_idx:, :]
        goal_pose[key] = goal_pose[key][goal_start_idx:, :]

    true_pose['time'] -= start_time
    goal_pose['time'] -= start_time

    bag.close()

    return true_pose, goal_pose


def plotPath(true_pose, goal_pose):
    """Plot the true and goal path."""

    f = plt.figure()
    ax = f.add_subplot(111, projection='3d')

    ax.plot(true_pose['position'][:, 0],
            true_pose['position'][:, 1],
            true_pose['position'][:, 2],
            'r-', label='Actual')

    ax.plot(goal_pose['position'][:, 0],
            goal_pose['position'][:, 1],
            goal_pose['position'][:, 2],
            'b-', label='Goal')

    ax.auto_scale_xyz([-7, 7], [-7, 7], [-7, 7])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    ax.set_title('Position Tracking')

    # Need this proxy stuff because legends don't work on 3D plots.
    true_proxy = lines.Line2D([0], [0], c='r')
    goal_proxy = lines.Line2D([0], [0], c='b')
    plt.legend([true_proxy, goal_proxy], ['Actual', 'Goal'])

    return f


def plotPositionComponents(true_pose, goal_pose):
    """Plot the position components."""

    f = plt.figure()

    axs = []
    h = []
    ylabel_strs = ['X [m]', 'Y [m]', 'Z [m]']
    for ii in range(3):
        ax = f.add_subplot(3, 1, ii+1)
        p_goal, = ax.plot(goal_pose['time'], goal_pose['position'][:, ii], 'b')
        p_true, = ax.plot(true_pose['time'], true_pose['position'][:, ii], 'r')

        h.append(p_goal)
        h.append(p_true)

        ax.grid(True)
        ax.autoscale(axis='x', tight=True)
        # ax.set_aspect('equal')
        ax.set_ylabel(ylabel_strs[ii])
        # ax.set_xlabel('Time [sec]')
        # ax_x.legend([])

        axs.append(ax)

    axs[2].set_xlabel('Time [sec]')
    axs[2].legend(h[-2:], ['Goal', 'Actual'])

    f.suptitle('Position Components')

    return f


def plotPositionErrors(true_pose, goal_pose):
    """Plot the position errors."""

    f = plt.figure()

    axs = []
    ylabel_strs = ['X [m]', 'Y [m]', 'Z [m]']
    rmse = []
    for ii in range(3):
        ax = f.add_subplot(3, 1, ii+1)

        goal_int = np.interp(true_pose['time'],
                             goal_pose['time'], goal_pose['position'][:, ii])

        error = true_pose['position'][:, ii] - goal_int
        p_true, = ax.plot(true_pose['time'], error, 'r')

        rmse.append(np.sqrt(np.mean(error**2)))

        ax.grid(True)
        ax.autoscale(axis='x', tight=True)
        # ax.set_aspect('equal')
        ax.set_ylabel(ylabel_strs[ii])
        # ax.set_xlabel('Time [sec]')
        # ax_x.legend([])

        axs.append(ax)

    axs[2].set_xlabel('Time [sec]')

    f.suptitle('Position Component Errors')

    return rmse, f


def plotYawPitchRoll(true_pose, goal_pose):
    """Plot the orientation components (Yaw-Pitch-Roll)."""

    true_ypr = np.zeros((true_pose['time'].size, 3))
    for ii in range(true_ypr.shape[0]):
        true_ypr[ii, :] = trans.euler_from_quaternion(true_pose['orientation'][ii, :], axes='rzyx')

    goal_ypr = np.zeros((goal_pose['time'].size, 3))
    for ii in range(goal_ypr.shape[0]):
        goal_ypr[ii, :] = trans.euler_from_quaternion(goal_pose['orientation'][ii, :], axes='rzyx')

    f = plt.figure()

    axs = []
    h = []
    ylabel_strs = ['Yaw [deg]', 'Pitch [deg]', 'Roll [deg]']
    for ii in range(3):
        ax = f.add_subplot(3, 1, ii+1)

        p_goal, = ax.plot(goal_pose['time'], 180.0/np.pi*goal_ypr[:, ii], 'b')
        p_true, = ax.plot(true_pose['time'], 180.0/np.pi*true_ypr[:, ii], 'r')

        h.append(p_goal)
        h.append(p_true)

        ax.grid(True)
        ax.autoscale(axis='x', tight=True)
        # ax.set_aspect('equal')
        ax.set_ylabel(ylabel_strs[ii])
        # ax.set_xlabel('Time [sec]')
        # ax_x.legend([])

        axs.append(ax)

    axs[2].set_xlabel('Time [sec]')
    axs[2].legend(h[-2:], ['Goal', 'Actual'])

    f.suptitle('Yaw-Pitch-Roll')

    return f


def plotForwardError(true_pose, goal_pose):
    """Plot the angular error between the actual and goal forward vectors."""

    true_fwd = np.zeros((true_pose['orientation'].shape[0], 3))
    for ii in range(true_fwd.shape[0]):
        true_fwd[ii, :] = trans.quaternion_matrix(true_pose['orientation'][ii, :])[0:3, 1]

    goal_fwd = np.zeros((goal_pose['orientation'].shape[0], 3))
    for ii in range(goal_fwd.shape[0]):
        goal_fwd[ii, :] = trans.quaternion_matrix(goal_pose['orientation'][ii, :])[0:3, 1]

    goal_fwd_int = np.zeros(true_fwd.shape)
    for ii in range(3):
        goal_fwd_int[:, ii] = np.interp(true_pose['time'], goal_pose['time'], goal_fwd[:, ii])

    error_deg = 180.0/np.pi * np.arccos(np.sum(true_fwd * goal_fwd_int, 1))
    rmse = np.sqrt(np.mean(error_deg**2))

    f = plt.figure()
    ax = f.add_subplot(111)

    ax.plot(true_pose['time'], error_deg, 'r')

    ax.grid(True)
    ax.autoscale(axis='x', tight=True)
    # ax.set_aspect('equal')
    ax.set_ylabel('Forward [deg]')
    # ax.set_xlabel('Time [sec]')
    # ax_x.legend([])

    ax.set_xlabel('Time [sec]')
    # axs[2].legend(h[-2:], ['Goal', 'Actual'])

    f.suptitle('Forward Error')

    return rmse, f


def main():
    # Load data.
    bag_file = sys.argv[1]
    true_pose, goal_pose = loadData(bag_file)

    # Plot stuff.
    plt.ion()
    plotPath(true_pose, goal_pose)
    plotPositionComponents(true_pose, goal_pose)
    plotYawPitchRoll(true_pose, goal_pose)

    position_rmse, _ = plotPositionErrors(true_pose, goal_pose)
    print "Position RMSE = "
    print position_rmse

    fwd_rmse, _ = plotForwardError(true_pose, goal_pose)
    print "Forward RMSE = %f [deg]" % fwd_rmse

    return


if __name__ == '__main__':
    main()
