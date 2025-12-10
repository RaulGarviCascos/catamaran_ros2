#include "rclcpp/rclcpp.hpp"
#include "catamaran_interfaces/msg/velocity.hpp"


using catamaran_interfaces::msg::Velocity;
 

class ListerVelocityNode : public rclcpp::Node{
public:

    ListerVelocityNode():Node("listener_velocity"){
        subscriber_ = this->create_subscription<catamaran_interfaces::msg::Velocity>(
            "velocity",                                                                              // topic
            10,                                                                                      // tamaño de la cola
            [this](const catamaran_interfaces::msg::Velocity::SharedPtr msg) { callbackRobotNews(msg);}     // función callback
        );
    }

private:
    void callbackRobotNews(const catamaran_interfaces::msg::Velocity::SharedPtr msg){
        std::string vleft = std::to_string(msg->v_left);
        std::string vright = std::to_string(msg->v_right);
        RCLCPP_INFO(this->get_logger(), "(v_left:%s, v_right:%s)",vleft.c_str(),vright.c_str());
    }
    rclcpp::Subscription<catamaran_interfaces::msg::Velocity>::SharedPtr subscriber_;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ListerVelocityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}