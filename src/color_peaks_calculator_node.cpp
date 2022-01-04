#include "sh_scc/color_peaks_calculator_node.hpp"
#include "sh_common/ros_names.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sh_common_interfaces/msg/point.hpp"

namespace sh {

void get_kmeans_of(cv::Mat& img, int k, int a, double& c, cv::Mat& kmeans_labels, cv::Mat& kmeans_centers)
{
    cv::Mat img_float(img.rows*img.cols, 3, CV_32F);
    for (int j = 0; j < img.rows; j++)
        for (int i = 0; i < img.cols; i++)
            for (int h = 0; h < 3; h++)
                img_float.at<float>(j+(i*img.rows), h) = img.at<cv::Vec3b>(j,i)[h];
    c = cv::kmeans(
        img_float,
        k,
        kmeans_labels,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
        a,
        cv::KMEANS_PP_CENTERS,
        kmeans_centers
    );
}

double get_color_score(float r, float g, float b, float c)
{
    cv::Mat3f hsv, rgb(cv::Vec3f(r, g, b));
    cv::cvtColor(rgb, hsv, CV_RGB2HSV);
    cv::Vec3b hsv_px = hsv.at<cv::Vec3b>(0,0);
    double s = hsv_px.val[1], v = hsv_px.val[2];
    return (1-c) * ((0.5*s)/255.0) * ((10*v)/255.0);
}

void set_best_peak(cv::Mat& labels, cv::Mat& centers, unsigned char& r, unsigned char& g, unsigned char& b)
{
    const size_t num_labels = labels.rows;
    std::vector<unsigned int> cluster_counts(centers.rows, 0);
    for (int j = 0; j < num_labels; j++)
        cluster_counts[labels.at<cv::Vec3b>(j,0)[0]]++;

    double max_score = 0;
    for (int i = 0; i < centers.rows; i++)
    {
        float rf = centers.at<float>(i,0);
        float gf = centers.at<float>(i,1);
        float bf = centers.at<float>(i,2);
        double score = get_color_score(rf, gf, bf, static_cast<double>(cluster_counts[i])/num_labels);
        if (score > max_score)
        {
            max_score = score;
            r = static_cast<unsigned char>(rf);
            g = static_cast<unsigned char>(gf);
            b = static_cast<unsigned char>(bf);
        }
    }
}

void ColorPeakWindow::push_and_eval(unsigned char r,
                                    unsigned char g,
                                    unsigned char b,
                                    sh_common_interfaces::msg::Color& color)
{
    total_r += static_cast<unsigned int>(r);
    total_g += static_cast<unsigned int>(g);
    total_b += static_cast<unsigned int>(b);
    queue_r.push(r);
    queue_g.push(g);
    queue_b.push(b);
    size++;

    while (size > MAX_ELEMS)
    {
        total_r -= static_cast<unsigned int>(queue_r.front());
        total_g -= static_cast<unsigned int>(queue_g.front());
        total_b -= static_cast<unsigned int>(queue_b.front());
        queue_r.pop();
        queue_g.pop();
        queue_b.pop();
        size--;
    }

    color.channels[0] = total_r / size;
    color.channels[1] = total_g / size;
    color.channels[2] = total_b / size;
}

ColorPeaksCalculatorNode::ColorPeaksCalculatorNode() :
        HeartbeatNode("sh_color_peaks_calculator"),
        window_left(),
        window_right()
{
    cap_image_raw_sub = image_transport::create_subscription(
        this,
        ::sh::names::topics::SCC_CAMERA_IMAGE,
        std::bind(
            &sh::ColorPeaksCalculatorNode::cap_image_raw_callback,
            this,
            std::placeholders::_1
        ),
        "raw",
        rmw_qos_profile_sensor_data
    );
    color_peak_left_pub = create_publisher<sh_common_interfaces::msg::Color>(
        ::sh::names::topics::LEFT_COLOR_PEAK,
        1
    );
    color_peak_right_pub = create_publisher<sh_common_interfaces::msg::Color>(
        ::sh::names::topics::RIGHT_COLOR_PEAK,
        1
    );
    color_peaks_telem_pub = create_publisher<sh_scc_interfaces::msg::ColorPeaksTelem>(
        ::sh::names::topics::COLOR_PEAKS_TELEM,
        1
    );

    RCLCPP_INFO(get_logger(), "Initialized.");
}

void ColorPeaksCalculatorNode::cap_image_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
        sh_scc_interfaces::msg::ColorPeaksTelem color_peaks_telem_msg;

        const cv_bridge::CvImageConstPtr img = cv_bridge::toCvShare(msg, msg->encoding);

        const int k = 4, attempts = 4;
        double compactness_left, compactness_right;
        cv::Mat kmeans_labels_left, kmeans_labels_right, kmeans_centers_left, kmeans_centers_right;
        cv::Range range_height(0, img->image.rows);
        int width = img->image.cols;
        int half_width = width / 2;
        cv::Mat cv_img_world_left = img->image(range_height, cv::Range(0, half_width));
        cv::Mat cv_img_world_right = img->image(range_height, cv::Range(half_width, width));
        get_kmeans_of(cv_img_world_left, k, attempts, compactness_left, kmeans_labels_left, kmeans_centers_left);
        get_kmeans_of(cv_img_world_right, k, attempts, compactness_right, kmeans_labels_right, kmeans_centers_right);

        unsigned char lr, lg, lb, rr, rg, rb;
        set_best_peak(kmeans_labels_left, kmeans_centers_left, lr, lg, lb);
        set_best_peak(kmeans_labels_right, kmeans_centers_right, rr, rg, rb);

        sh_common_interfaces::msg::Color color_peak_left_msg;
        color_peak_left_msg.channels.resize(3, 0);
        window_left.push_and_eval(lr, lg, lb, color_peak_left_msg);

        sh_common_interfaces::msg::Color color_peak_right_msg;
        color_peak_right_msg.channels.resize(3, 0);
        window_right.push_and_eval(rr, rg, rb, color_peak_right_msg);

        color_peak_left_pub->publish(color_peak_left_msg);
        color_peak_right_pub->publish(color_peak_right_msg);
        color_peaks_telem_pub->publish(color_peaks_telem_msg);
}

}

int main(int argc, char** argv) 
{ 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sh::ColorPeaksCalculatorNode>());
    return 0;
}
