#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;
using geometry_msgs::msg::Twist;


class MappingCmdVelNode : public rclcpp::Node{
public:
    MappingCmdVelNode():Node("mapping_cmd_to_override"){

        publisher_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            "/mavros/rc/override", 10);

        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "catamaran/cmd_vel",                                                         // topic
            10,                                                                          // tamaño de la cola
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { callback(msg);}     // función callback
        );

        RCLCPP_INFO(this->get_logger(),"Mapping cmd_vel node has been started");
    }   

private:

    int sat(int pwm) {
        if (pwm < 1000) return 1000;
        if (pwm > 2000) return 2000;
        return pwm;
    }

    void publishOverride(float v,float w){

        mavros_msgs::msg::OverrideRCIn msg;

        int K_V = 250;
        int K_W = 333.3;    

        int ch1 = sat(1500 + K_W * w);
        int ch3 = sat(1500 + K_V * v);
       

        // Todos los canales sin override:
        for (int i = 0; i < 18; i++)
            msg.channels[i] = 0;

        //modifico solo los de throttle y giro
        msg.channels[0] = ch1;   
        msg.channels[2] = ch3;
          
        RCLCPP_INFO(this->get_logger(), "(ch1:%d - w:%lf, ch3:%d - v:%lf)",ch1,w,ch3,v);
        publisher_->publish(msg);
        
    }

    void callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        std::string linear_x = std::to_string(msg->linear.x);
        std::string angular_z = std::to_string(msg->angular.z);
        publishOverride(msg->linear.x,msg->angular.z);
    }

    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

    std::string command;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char **argv){
    
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MappingCmdVelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}