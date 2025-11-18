#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "catamaran_interfaces/msg/velocity.hpp"
#include <string> 

using catamaran_interfaces::msg::Velocity;
using namespace std::chrono_literals;

class ControlVelocityNode : public rclcpp::Node{
public:
    ControlVelocityNode():Node("control_velocity"){

        publisher_ = this->create_publisher<catamaran_interfaces::msg::Velocity>("velocity", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        [this]{publishVelocity();});
        RCLCPP_INFO(this->get_logger(),"Robot news station node has been started");
    }   

private:

     void publishVelocity(){
        std::cout << "Introduce la velocidad (vL,vR): ";
        std::getline(std::cin, command);

        // Buscar la coma
        auto comma_pos = command.find(',');
        std::string left_str;
        std::string right_str;

        if (comma_pos == std::string::npos) {
            // No hay coma: todo va a la izquierda, derecha = 0
            left_str = command;
            right_str = "0";
        } else {
            left_str  = command.substr(0, comma_pos);
            right_str = command.substr(comma_pos + 1);
        }

        try {
            Velocity msg;
            msg.v_left  = std::stoi(left_str);
            msg.v_right = std::stoi(right_str);

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", command.c_str());
        }
        catch (const std::exception & e) {
            RCLCPP_WARN(this->get_logger(),
                        "Entrada no válida: '%s' (%s)",
                        command.c_str(), e.what());
        }
    }
    rclcpp::Publisher<catamaran_interfaces::msg::Velocity>::SharedPtr publisher_;
    std::string command;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char **argv){
    
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ControlVelocityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}