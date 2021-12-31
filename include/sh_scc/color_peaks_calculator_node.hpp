#pragma once

#include "sh_common/heartbeat_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sh_common_interfaces/msg/color.hpp"
#include "sh_scc_interfaces/msg/color_peaks_telem.hpp"
#include "sh_scc_interfaces/srv/request_screen_calibration.hpp"
#include "sh_scc_interfaces/srv/set_screen_calibration_points_of_homography.hpp"

#include <string>
#include <queue>

namespace sh {

struct ColorPeakWindow
{
protected:
    const static size_t MAX_ELEMS = 10;
    unsigned int size;
    unsigned int total_r;
    unsigned int total_g;
    unsigned int total_b;
    std::queue<unsigned char> queue_r;
    std::queue<unsigned char> queue_g;
    std::queue<unsigned char> queue_b;

public:
    void push_and_eval(unsigned char r,
                       unsigned char g,
                       unsigned char b,
                       sh_common_interfaces::msg::Color& color);
};

class ColorPeaksCalculatorNode : public HeartbeatNode
{
protected:
    const static int WORLD_HEIGHT = 300;
    const static int WORLD_WIDTH = 400;
    const static std::vector<cv::Point2f> WORLD_PLANE;
    const static cv::Size WORLD_SIZE;

    rclcpp::Service<sh_scc_interfaces::srv::RequestScreenCalibration>::SharedPtr
        screen_calibration_request_srv;
    rclcpp::Service<sh_scc_interfaces::srv::SetScreenCalibrationPointsOfHomography>::SharedPtr
        screen_calibration_set_homography_points_srv;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cap_image_raw_sub;
    rclcpp::Publisher<sh_common_interfaces::msg::Color>::SharedPtr color_peak_left_pub;
    rclcpp::Publisher<sh_common_interfaces::msg::Color>::SharedPtr color_peak_right_pub;
    rclcpp::Publisher<sh_scc_interfaces::msg::ColorPeaksTelem>::SharedPtr color_peaks_telem_pub;

    cv_bridge::CvImagePtr last_received_frame;

    bool homog_set;
    cv::Mat homog;

    ColorPeakWindow window_left;
    ColorPeakWindow window_right;

public:
    ColorPeaksCalculatorNode();

    void screen_calibration_request_callback(
        const sh_scc_interfaces::srv::RequestScreenCalibration::Request::SharedPtr req,
        sh_scc_interfaces::srv::RequestScreenCalibration::Response::SharedPtr res);

    void screen_calibration_set_homography_points_callback(
        const sh_scc_interfaces::srv::SetScreenCalibrationPointsOfHomography::Request::SharedPtr req,
        sh_scc_interfaces::srv::SetScreenCalibrationPointsOfHomography::Response::SharedPtr res);

    void cap_image_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

}
