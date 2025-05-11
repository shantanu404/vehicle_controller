#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace Eigen;

class MPCControllerNode : public rclcpp::Node
{
public:
  MPCControllerNode()
  : Node("mpc_controller")
  {
    this->declare_parameter<double>("velocity", 2.5);

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image_raw", 10,
      std::bind(&MPCControllerNode::imageCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    update_timer_ = this->create_wall_timer(10ms, std::bind(&MPCControllerNode::cmdCallback, this));

    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::namedWindow("Gray", cv::WINDOW_NORMAL);
    cv::namedWindow("Binary", cv::WINDOW_NORMAL);
    cv::namedWindow("Warped", cv::WINDOW_NORMAL);
    cv::namedWindow("Windows", cv::WINDOW_NORMAL);
  }

  ~MPCControllerNode() override
  {
    cv::destroyAllWindows();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  int error_ = 0;
  cv::Mat frame_;

  // Fit 2nd-degree polynomial to lane points using Eigen
  Eigen::VectorXd polyfit(const std::vector<cv::Point>& pts) {
    int n = pts.size();

    // Create matrices A (design matrix) and b (output vector)
    MatrixXd A(n, 3);
    VectorXd b(n);
    
    for (int i = 0; i < n; ++i) {
      double y = pts[i].y;
      A(i, 0) = y * y;
      A(i, 1) = y;
      A(i, 2) = 1.0;
      b(i) = pts[i].x;
    }

    // Solve the least squares problem A * coeff = b
    VectorXd coeff = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

    return coeff; // Returns [a, b, c] for the quadratic equation y = a*x^2 + b*x + c
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // 1) Original
    frame_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("Original", frame_);

    // 2) Grayscale
    cv::Mat gray;
    cv::cvtColor(frame_, gray, cv::COLOR_BGR2GRAY);
    cv::imshow("Gray", gray);

    // 3) Binary threshold
    cv::Mat binary;
    cv::threshold(gray, binary, 160, 255, cv::THRESH_BINARY);
    cv::imshow("Binary", binary);

    // 4) Perspective transform using full binary image
    std::vector<cv::Point2f> src = {{0,400},{200,300},{440,300},{640,400}};
    std::vector<cv::Point2f> dst = {{0,0},{0,480},{640,480},{640,0}};
    cv::Mat H = cv::getPerspectiveTransform(src, dst);
    cv::Mat warped;
    cv::warpPerspective(binary, warped, H, binary.size());
    cv::imshow("Warped", warped);

    // 5) Sliding-window rectangle drawing
    int nwindows = 9, margin = 100;
    int w = warped.cols, h = warped.rows;
    int window_h = h / nwindows;
    cv::Mat hist;
    cv::reduce(warped(cv::Range(h/2, h), cv::Range::all()), hist, 0, cv::REDUCE_SUM, CV_32S);
    std::vector<int> histogram(w);
    for (int i = 0; i < w; ++i) histogram[i] = hist.at<int>(0, i);
    int mid = w / 2;
    int leftx = std::distance(histogram.begin(), std::max_element(histogram.begin(), histogram.begin() + mid));
    int rightx = std::distance(histogram.begin() + mid, std::max_element(histogram.begin() + mid, histogram.end())) + mid;
    cv::Mat win_draw;
    cv::cvtColor(warped, win_draw, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < nwindows; ++i) {
      int y_low = h - (i+1)*window_h;
      int y_high = h - i*window_h;
      int xleft_low = leftx - margin;
      int xleft_high = leftx + margin;
      int xright_low = rightx - margin;
      int xright_high = rightx + margin;
      cv::rectangle(win_draw, cv::Point(xleft_low, y_low), cv::Point(xleft_high, y_high), cv::Scalar(0,255,0), 2);
      cv::rectangle(win_draw, cv::Point(xright_low, y_low), cv::Point(xright_high, y_high), cv::Scalar(255,0,0), 2);
    }
    cv::imshow("Windows", win_draw);

    // 6) Collect lane points
    std::vector<cv::Point> left_pts, right_pts;
    for (int i = 0; i < h; ++i) {
      left_pts.push_back(cv::Point(leftx, i));
      right_pts.push_back(cv::Point(rightx, i));
    }

    // 7) Fit polynomials for left and right lanes using Eigen
    Eigen::VectorXd left_fit = polyfit(left_pts);
    Eigen::VectorXd right_fit = polyfit(right_pts);

    // 8) Compute error at y = -1.0 (for both left and right lane polynomials)
    double left_x_at_neg1 = left_fit(0) * (-1.0) * (-1.0) + left_fit(1) * (-1.0) + left_fit(2);
    double right_x_at_neg1 = right_fit(0) * (-1.0) * (-1.0) + right_fit(1) * (-1.0) + right_fit(2);

    int lane_center = (left_x_at_neg1 + right_x_at_neg1) / 2;  // Average x at y = -1.0
    int image_center = frame_.cols / 2;
    error_ = image_center - lane_center;

    cv::waitKey(1);
  }

  void cmdCallback()
  {
    auto cmd = geometry_msgs::msg::Twist();
    // Forward velocity from parameter
    cmd.linear.x = this->get_parameter("velocity").as_double();
    // Simple angular control based on error
    cmd.angular.z = ((error_ * 90.0) / frame_.cols) / 15;
    cmd_vel_pub_->publish(cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCControllerNode>());
  rclcpp::shutdown();
  return 0;
}
