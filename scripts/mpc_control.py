#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__("mpc_controller")
        self.declare_parameter("velocity", 2.5)

        self.bridge = CvBridge()
        self.error_ = 0
        self.frame = None

        # Hard-coded warp matrix from notebook
        self.mtx = np.array(
            [
                [-6.12500000e00, -3.80000000e01, 2.28000000e03],
                [-2.96825540e-14, -6.40000000e01, 3.84000000e03],
                [-1.00417506e-16, -1.18750000e-01, 1.00000000e00],
            ],
            dtype=np.float32,
        )

        # Marker array for lane markers
        self.left_fit = np.array([0, 0, 0])
        self.right_fit = np.array([0, 0, 0])
        self.mid_fit = np.array([0, 0, 0])

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, "/camera1/image_raw", self.image_callback, 1
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "lane_markers", 1)

        # Timer for publishing commands
        self.create_timer(0.01, self.cmd_timer_callback)
        self.create_timer(1, self.marker_timer_callback)

        # OpenCV windows for debugging
        for win in ["Original", "HSV_V", "Binary", "Morphed", "Warped", "Windows"]:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def image_callback(self, msg: Image):
        # 1. Convert ROS image to OpenCV BGR and resize
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (640, 480))
        cv2.imshow("Original", frame)

        # 2. Perspective warp
        warped = cv2.warpPerspective(frame, self.mtx, (640, 480))
        cv2.imshow("Warped", warped)

        # 3. HSV threshold on V channel
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        _, _, V = cv2.split(hsv)
        cv2.imshow("HSV_V", V)

        _, binary = cv2.threshold(V, 30, 255, cv2.THRESH_BINARY)
        cv2.imshow("Binary", binary)

        # 4. Morphological opening to clean small blobs
        kernel = np.ones((5, 5), np.uint8)
        morphed = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        cv2.imshow("Morphed", morphed)

        # 5. Sliding-window lane detection
        h, w = morphed.shape
        histogram = np.sum(morphed[h // 2 :, :], axis=0)
        midpoint = w // 2
        leftx_current = np.argmax(histogram[:midpoint])
        rightx_current = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 7
        margin = 30
        minpix = 50
        window_height = int(h * 0.50) // nwindows

        nonzero = morphed.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds, right_lane_inds = [], []

        output = warped
        for window in range(1, nwindows):
            y_low = h - (window + 1) * window_height
            y_high = h - window * window_height
            x_left_low = leftx_current - margin
            x_left_high = leftx_current + margin
            x_right_low = rightx_current - margin
            x_right_high = rightx_current + margin

            # Draw windows for visualization
            cv2.rectangle(
                output, (x_left_low, y_low), (x_left_high, y_high), (0, 255, 0), 2
            )
            cv2.rectangle(
                output, (x_right_low, y_low), (x_right_high, y_high), (255, 0, 0), 2
            )

            # Identify nonzero pixels in window
            good_left = (
                (nonzeroy >= y_low)
                & (nonzeroy < y_high)
                & (nonzerox >= x_left_low)
                & (nonzerox < x_left_high)
            ).nonzero()[0]

            good_right = (
                (nonzeroy >= y_low)
                & (nonzeroy < y_high)
                & (nonzerox >= x_right_low)
                & (nonzerox < x_right_high)
            ).nonzero()[0]

            left_lane_inds.append(good_left)
            right_lane_inds.append(good_right)

            # Recenter if enough pixels found
            if len(good_left) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left]))
            if len(good_right) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right]))

        # Concatenate all indices and extract pixel positions
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Generate rviz markers for left and right lane
        bias = 1.3 + window_height / 63.297478
        real_left_x = (w / 2 - leftx) / 53.735893
        real_left_y = (h - lefty) / 63.297478 + bias
        real_right_x = (w / 2 - rightx) / 53.735893
        real_right_y = (h - righty) / 63.297478 + bias

        self.lane_points = np.concatenate(
            [
                np.stack([real_left_y, real_left_x], axis=1),
                np.stack([real_right_y, real_right_x], axis=1),
            ],
            axis=0,
        )

        # 6. Fit 2nd-degree (quadratic) polynomials
        self.left_fit = np.polyfit(real_left_y, real_left_x, 3)
        self.right_fit = np.polyfit(real_right_y, real_right_x, 3)
        self.mid_fit = (self.left_fit + self.right_fit) / 2

        # 7. Project midpoint back at the bottom of image (y = h)
        y_eval = 1.75
        mid_x = (
            self.mid_fit[0] * y_eval**3
            + self.mid_fit[1] * y_eval**2
            + self.mid_fit[2] * y_eval
            + self.mid_fit[3]
        )

        # 8. Compute error (pixels) relative to image center
        self.error_ = mid_x

        # 9. Unwarp the output and show it
        output = cv2.warpPerspective(
            output, np.linalg.inv(self.mtx), (640, 480), flags=cv2.INTER_LINEAR
        )
        cv2.imshow("Windows", output)
        cv2.waitKey(1)

    def cmd_timer_callback(self):
        # Publish control command
        vel = self.get_parameter("velocity").get_parameter_value().double_value
        cmd = Twist()
        cmd.linear.x = vel
        cmd.angular.z = (np.pi / 3) * self.error_
        # self.get_logger().info(f"Lane error: {self.error_:.1f}px")
        self.cmd_pub.publish(cmd)

    def marker_timer_callback(self):
        markers = []

        y = np.linspace(0, 5, 10)

        left_lane_x = (
            self.left_fit[0] * y**3
            + self.left_fit[1] * y**2
            + self.left_fit[2] * y
            + self.left_fit[3]
        )
        right_lane_x = (
            self.right_fit[0] * y**3
            + self.right_fit[1] * y**2
            + self.right_fit[2] * y
            + self.right_fit[3]
        )

        mid_lane_x = (left_lane_x + right_lane_x) / 2

        left_lane_points = np.stack([y, left_lane_x], axis=1)
        right_lane_points = np.stack([y, right_lane_x], axis=1)
        mid_lane_points = np.stack([y, mid_lane_x], axis=1)

        for i, (x, y) in enumerate(
            np.concatenate([left_lane_points, right_lane_points, mid_lane_points])
        ):
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "lane_marker"
            marker.id = i
            marker.type = Marker.SPHERE

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime = Duration(seconds=0)

            markers.append(marker)

        marker_array = MarkerArray(markers=markers)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
