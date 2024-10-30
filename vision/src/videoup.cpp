#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::VideoCapture cap("/home/addienze/MagangBanyubramanta/Main/vision/src/second.mp4");


class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher")
    {
        raw_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera", 10);
        masked_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/mask", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {

        cv::Mat video;        

        if (!cap.read(video))
        {
            RCLCPP_INFO(this->get_logger(), "Video has reach its end.");
            rclcpp::shutdown();
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(video, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar minHSV(0, 0, 0); 
        cv::Scalar maxHSV(40, 255, 255); 

        cv::Mat mask;
        cv::inRange(hsv, minHSV, maxHSV, mask);

        cv::imshow("frame", video);
        cv::imshow("mask", mask);

        auto raw_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", video).toImageMsg();
        auto masked_image = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        raw_image_pub_->publish(*raw_image);
        masked_image_pub_->publish(*masked_image);

        if (cv::waitKey(1) == 's')
        {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr masked_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
