#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class JoystickOutput: public rclcpp::Node {
public:
    JoystickOutput() : Node("joy_output") {
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&JoystickOutput::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "\nx = %.2f \ny = %.2f\nyaw = %.2f\ndepth = %.2f",
                   msg->linear.x, msg->linear.y, msg->angular.z, msg->linear.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickOutput>());
    rclcpp::shutdown();
    return 0;
}