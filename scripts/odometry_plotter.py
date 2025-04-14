#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import threading
import numpy as np
import sys
from tf_transformations import euler_from_quaternion

class OdometryPlotterNode(Node):
    def __init__(self):
        super().__init__('odometry_plotter')

        self.x_data = []
        self.y_data = []
        self.w_data = []
        self.times = []
        self._lock = threading.Lock()

        self.fig, self.ax = plt.subplots(3, 1)
        self.xpoints, = self.ax[0].plot([], [], 'r-')
        self.ypoints, = self.ax[1].plot([], [], 'r-')
        self.wpoints, = self.ax[2].plot([], [], 'r-')

        for a in self.ax:
            a.set_xlabel('Time (s)')

        self.ax[0].set_ylabel('X Coordinate (m)')
        self.ax[1].set_ylabel('Y Coordinate (m)')
        self.ax[2].set_ylabel('Angular Orientation (rad)')

        self.fig.suptitle('Robot Odometry (X, Y, \\Omega) over Time')
        for a in self.ax:
            a.grid(True)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=2
        )
        self.get_logger().info("OdometryPlotter initialized.  Subscribed to /odom topic.")

        self.ani = anim.FuncAnimation(self.fig, self.update_plot, interval=100)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, w = euler_from_quaternion((q.x, q.y, q.z, q.w))
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._lock:
            self.x_data.append(x)
            self.y_data.append(y)
            self.w_data.append(w)
            self.times.append(timestamp)

    def update_plot(self, frame):
        with self._lock:
            t_data = np.array(self.times)
            x_data = np.array(self.x_data)
            y_data = np.array(self.y_data)
            w_data = np.array(self.w_data)

        self.xpoints.set_data(t_data, x_data)
        self.ypoints.set_data(t_data, y_data)
        self.wpoints.set_data(t_data, w_data)

        for a in self.ax:
            a.relim()
            a.autoscale_view()

        # Adjust the plot limits to show the latest data, and add a small margin.
        if self.times:
            t_min = np.min(t_data)
            t_max = np.max(t_data)

            x_min = np.min(x_data)
            x_max = np.max(x_data)

            y_min = np.min(y_data)
            y_max = np.max(y_data)

            w_min = np.min(w_data)
            w_max = np.max(w_data)

            t_range = t_max - t_min
            x_range = x_max - x_min
            y_range = y_max - y_min
            w_range = w_max - w_min

            # Add a small margin (10%) around the data
            t_margin = t_range * 0.1
            x_margin = x_range * 0.1
            y_margin = y_range * 0.1
            w_margin = w_range * 0.1

            for ax in self.ax:
                ax.set_xlim(t_min - t_margin, t_max + t_margin)

            self.ax[0].set_ylim(x_min - x_margin, x_max + x_margin)
            self.ax[1].set_ylim(y_min - y_margin, y_max + y_margin)
            self.ax[2].set_ylim(w_min - w_margin, w_max + w_margin)

        return (self.xpoints, self.ypoints)

    def _plt(self):
        plt.rcParams['text.usetex'] = True
        plt.show() #show

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPlotterNode() # Change to the new Node class
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node._plt() #start the plot
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

