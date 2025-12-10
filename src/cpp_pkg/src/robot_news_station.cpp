#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node{
public:
    RobotNewsStationNode():Node("robot_news_station"),counter_(0){
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news",10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),[this]{publishNews();});
        RCLCPP_INFO(this->get_logger(),"Robot news station node has been started");
    }   

private:
    void publishNews(){
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hello ")+std::to_string(counter_);
        publisher_ -> publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: Hello %d", counter_);
        counter_++;
    }
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv){
    
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}