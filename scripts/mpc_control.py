#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


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

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, "/camera1/image_raw", self.image_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Timer for publishing commands
        self.create_timer(0.01, self.cmd_timer_callback)

        # OpenCV windows for debugging
        for win in ["Original", "Warped", "HSV_V", "Binary", "Morphed", "Windows"]:
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

        nwindows = 10
        margin = 50
        minpix = 50
        window_height = h // nwindows

        nonzero = morphed.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds, right_lane_inds = [], []

        output = cv2.cvtColor(morphed, cv2.COLOR_GRAY2BGR)
        for window in range(nwindows):
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

        # 6. Fit 2nd-degree (quadratic) polynomials
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        mid_fit = (left_fit + right_fit) / 2

        # 7. Project midpoint back at the bottom of image (y = h)
        y_eval = h
        # mid_fit = [A, B, C] for Ax^2 + Bx + C
        mid_x = mid_fit[0] * y_eval**2 + mid_fit[1] * y_eval + mid_fit[2]

        # 8. Compute error (pixels) relative to image center
        image_center = w / 2
        self.error_ = float(image_center - mid_x)

        cv2.imshow("Windows", output)
        cv2.waitKey(1)

    def cmd_timer_callback(self):
        # Publish control command
        vel = self.get_parameter("velocity").get_parameter_value().double_value
        cmd = Twist()
        cmd.linear.x = vel
        cmd.angular.z = self.error_ / 100.0
        self.get_logger().info(f"Lane error: {self.error_:.1f}px")
        # self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
