#include "sh_common/ros_names.hpp"

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <memory>
#include <string>

static const char* WINDOW_NAME = "TestImage";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("sh_cv_bridge", "/smart_home");
    cv::namedWindow(WINDOW_NAME);
    RCLCPP_INFO(node->get_logger(), "Created named CV window: '%s'.", WINDOW_NAME);
    auto image_sub = image_transport::create_subscription(
        node.get(),
        sh::names::topics::SCC_CAMERA_IMAGE,
        [&](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                std::cerr << "CvBridge exception: " << e.what() << std::endl;
                return;
            }
            cv::imshow(WINDOW_NAME, cv_ptr->image);
            cv::waitKey(3);
        },
        "raw",
        rmw_qos_profile_sensor_data
    );

    RCLCPP_INFO(node->get_logger(), "Starting spin.");
    rclcpp::spin(node);

    cv::destroyWindow(WINDOW_NAME);
    return 0;
}
