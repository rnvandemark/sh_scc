#pragma once

#include "sh_common/heartbeat_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sh_common_interfaces/msg/color.hpp"
#include "sh_scc_interfaces/msg/color_peaks_telem.hpp"

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
    image_transport::Subscriber cap_image_raw_sub;
    rclcpp::Publisher<sh_common_interfaces::msg::Color>::SharedPtr color_peak_left_pub;
    rclcpp::Publisher<sh_common_interfaces::msg::Color>::SharedPtr color_peak_right_pub;
    rclcpp::Publisher<sh_scc_interfaces::msg::ColorPeaksTelem>::SharedPtr color_peaks_telem_pub;

    ColorPeakWindow window_left;
    ColorPeakWindow window_right;

public:
    ColorPeaksCalculatorNode();

    void cap_image_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
};

}
