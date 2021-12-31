#include "sh_scc/color_peaks_calculator_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <memory>

int main(int argc, char** argv) 
{ 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sh::ColorPeaksCalculatorNode>());
    return 0;
}
