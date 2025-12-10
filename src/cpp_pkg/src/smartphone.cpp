#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//creo mi nodo personalizado
class SmartphoneNode : public rclcpp::Node{

public:
    SmartphoneNode():Node("smartphone"){
        //inicializo la variable suscriptor
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "robot_news",                               // topic
            10,                                         // tamaño de la cola
            [this](const std_msgs::msg::String::SharedPtr msg) { callbackRobotNews(msg);} // función callback
        );
    }

private:
    // Callback
    void callbackRobotNews(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    //declaro la variable compartida suscriptor al topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv){
    // Inicializar ROS2
    rclcpp::init(argc,argv);
    // Crear el nodo 
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}