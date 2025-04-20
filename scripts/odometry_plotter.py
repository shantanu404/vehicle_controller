#!/usr/bin/env python3

import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as anim  # Import FuncAnimation
import numpy as np
import csv
from tf_transformations import euler_from_quaternion
import sys


class OdometryPlotterNode(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('odometry_plotter')

        # Initialize data lists
        self.x_data = []
        self.y_data = []
        self.o_data = []
        self.v_data = []
        self.w_data = []
        self.times = []
        self._lock = threading.Lock()
        self.csv_file_path = os.path.join(os.getenv('HOME'), '.ros/odometry_data.csv')
        self.animation_interval = 100

        # Set up the Matplotlib figure and axes (5 subplots)
        self.fig, self.ax = plt.subplots(5, 1, figsize=(10, 12))  # Adjust figure size as needed
        self.xpoints, = self.ax[0].plot([], [], 'r-')
        self.ypoints, = self.ax[1].plot([], [], 'r-')
        self.opoints, = self.ax[2].plot([], [], 'r-')
        self.vpoints, = self.ax[3].plot([], [], 'r-')
        self.wpoints, = self.ax[4].plot([], [], 'r-')

        for a in self.ax:
            a.set_xlabel('Time (s)')
            a.grid(True)

        self.ax[0].set_ylabel('X Coordinate (m)')
        self.ax[1].set_ylabel('Y Coordinate (m)')
        self.ax[2].set_ylabel('Yaw (rad)')
        self.ax[3].set_ylabel('Speed (m/s)')
        self.ax[4].set_ylabel('Angular velocity (rad/s)')
        self.fig.suptitle('Robot Odometry')

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/base_pose_ground_truth',
            self.odom_callback,
            qos_profile=2
        )
        self.get_logger().info("OdometryPlotter initialized.  Subscribed to /base_pose_ground_truth topic.")

        # Initialize FuncAnimation
        self.ani = anim.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=self.animation_interval,
            blit=False,  # Blitting can cause issues with some backends, set to False
        )

    def odom_callback(self, msg):
        # Extract data from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, o = euler_from_quaternion((q.x, q.y, q.z, q.w))
        v = np.linalg.norm((msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
        w = msg.twist.twist.angular.z
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Append data to lists using the lock
        with self._lock:
            self.x_data.append(x)
            self.y_data.append(y)
            self.o_data.append(o)
            self.v_data.append(v)
            self.w_data.append(w)
            self.times.append(timestamp)

    def update_plot(self, frame):
        # Get the data and convert to numpy arrays *within* the lock
        with self._lock:
            t_data_n = np.array(self.times)
            x_data_n = np.array(self.x_data)
            y_data_n = np.array(self.y_data)
            o_data_n = np.array(self.o_data)
            v_data_n = np.array(self.v_data)
            w_data_n = np.array(self.w_data)

        # Update the Matplotlib plot data
        self.xpoints.set_data(t_data_n, x_data_n)
        self.ypoints.set_data(t_data_n, y_data_n)
        self.opoints.set_data(t_data_n, o_data_n)
        self.vpoints.set_data(t_data_n, v_data_n)
        self.wpoints.set_data(t_data_n, w_data_n)

        # Auto-scale the axes
        for a in self.ax:
            a.relim()
            a.autoscale_view()

        # Adjust the plot limits to show the latest data, and add a small margin.
        if self.times:
            t_min = np.min(t_data_n)
            t_max = np.max(t_data_n)
            x_min = np.min(x_data_n)
            x_max = np.max(x_data_n)
            y_min = np.min(y_data_n)
            y_max = np.max(y_data_n)
            o_min = np.min(o_data_n)
            o_max = np.max(o_data_n)
            v_min = np.min(v_data_n)
            v_max = np.max(v_data_n)
            w_min = np.min(w_data_n)
            w_max = np.max(w_data_n)

            t_range = t_max - t_min
            x_range = x_max - x_min
            y_range = y_max - y_min
            o_range = o_max - o_min
            v_range = v_max - v_min
            w_range = w_max - w_min

            # Add a small margin (10%) around the data
            t_margin = t_range * 0.1
            x_margin = x_range * 0.1
            y_margin = y_range * 0.1
            o_margin = o_range * 0.1
            v_margin = v_range * 0.1
            w_margin = w_range * 0.1

            for a in self.ax:
                a.set_xlim(t_min - t_margin, t_max + t_margin)
            self.ax[0].set_ylim(x_min - x_margin, x_max + x_margin)
            self.ax[1].set_ylim(y_min - y_margin, y_max + y_margin)
            self.ax[2].set_ylim(o_min - o_margin, o_max + o_margin)
            self.ax[3].set_ylim(v_min - v_margin, v_max + v_margin)
            self.ax[4].set_ylim(w_min - w_margin, w_max + w_margin)

        return (
            self.xpoints,
            self.ypoints,
            self.opoints,
            self.vpoints,
            self.wpoints,
        )  # Return the updated plot elements


    def write_to_csv(self):
        with open(self.csv_file_path, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'X', 'Y', 'Yaw', 'Velocity', 'Angular Velocity'])
            min_length = min(len(self.times), len(self.x_data), len(self.y_data), len(self.o_data), len(self.v_data), len(self.w_data))
            for i in range(min_length):
                writer.writerow([self.times[i], self.x_data[i], self.y_data[i], self.o_data[i], self.v_data[i], self.w_data[i]])

    def _plt(self):
        plt.show()

        # Write to CSV file
        self.write_to_csv()



def main(args=None):
    rclpy.init(args=args)
    node = OdometryPlotterNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    node._plt()

if __name__ == '__main__':
    main()

