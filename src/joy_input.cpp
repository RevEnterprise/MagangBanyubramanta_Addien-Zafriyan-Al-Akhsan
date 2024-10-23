#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>

class JoystickInput : public rclcpp::Node {
public:
    JoystickInput() : Node("joy_Input") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoystickInput::joy, this, std::placeholders::_1));
        
        cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
        //vel, input
        float x_cmd = value(msg->axes[0] * 250, 250);  
        float y_cmd = valuey(msg->axes[1] * 250, 250);  
        float yaw = value(msg->axes[3] * 180, 180);     

        //button, depth
        static float depth = 0.0f;
        if (msg->buttons[0] && depth < 10.0f) {
            depth += 0.5f; 
        }
        if (msg->buttons[3] && depth > 0.0f) {
            depth -= 0.5f;  
        }


        // vel msg
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = x_cmd;
        twist_msg.linear.y = y_cmd;
        twist_msg.angular.z = yaw;
        twist_msg.linear.z = depth;

        cmd_vel->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Published: \nx_cmd: %.2f, \ny_cmd: %.2f, \nyaw: %.2f, \ndepth: %.2f",
                     x_cmd, y_cmd, yaw, depth);
    }
    float valuey(float value, float max) {
        return std::max(-max, std::min(max, value));
    }
    float value(float value, float max) {
        return std::max(-max, std::min(max, -value));
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickInput>());
    rclcpp::shutdown();
    return 0;
}