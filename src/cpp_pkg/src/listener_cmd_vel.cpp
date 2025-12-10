#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using geometry_msgs::msg::Twist;
 

class ListerVelNode : public rclcpp::Node{
public:

    ListerVelNode():Node("listener_cmd_vel"){
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "catamaran/cmd_vel",                                                         // topic
            10,                                                                          // tamaño de la cola
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { callback(msg);}     // función callback
        );
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        std::string linear_x = std::to_string(msg->linear.x);
        std::string angular_z = std::to_string(msg->angular.z);
        RCLCPP_INFO(this->get_logger(), "(linear_x:%s, angular_z:%s)",linear_x.c_str(),angular_z.c_str());
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ListerVelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}