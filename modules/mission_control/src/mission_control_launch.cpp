#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <iostream>
#include <mission_control/mode_start.hpp>
#include <mission_control/mode_run.hpp>
#include <mission_control/mode_pause.hpp>
#include <mission_control/mode_park.hpp>
#include <mission_control/mode_emergency.hpp>
#include <string>

using namespace std::chrono_literals;

class MissionController: public rclcpp::Node {
    
    public:
    MissionController() : Node("mission_controller") {
        RCLCPP_INFO(this->get_logger(), "Starting Mission Controller");
        mode_publisher_ = create_publisher<std_msgs::msg::UInt8>("/vehicle_mode", 10);
        publisher_timer_ = create_wall_timer(500ms, std::bind(&MissionController::publish_mode, this));
        loop_timer_ = create_wall_timer(500ms, std::bind(&MissionController::control_loop, this));

    }

    void init() {
        auto shared_this = shared_from_this();
        modes[MODE_START] = std::make_shared<StartMode>(shared_this);
        modes[MODE_RUN] = std::make_shared<RunMode>(shared_this);
        modes[MODE_PAUSE] = std::make_shared<PauseMode>(shared_this);
        modes[MODE_PARK] = std::make_shared<ParkMode>();
        modes[MODE_EMERGENCY] = std::make_shared<EmergencyMode>();

        CURRENT_MODE = MODE_START;
        last_mode_=CURRENT_MODE;
        RCLCPP_INFO(this->get_logger(), "Vehicle Mode Set to %u", CURRENT_MODE);
    }

    private:
    uint8_t CURRENT_MODE=MODE_START;
    uint8_t last_mode_;
    std::map<unsigned int, std::shared_ptr<ModeBase>> modes;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_publisher_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    void publish_mode() {
        auto message = std_msgs::msg::UInt8();
        message.data = CURRENT_MODE;
        mode_publisher_->publish(message);
    }

    void control_loop() {  
        if(last_mode_ != CURRENT_MODE) {
            RCLCPP_INFO(this->get_logger(), "Vehicle Mode Set to %u", CURRENT_MODE);
            last_mode_=CURRENT_MODE;
        }
        last_mode_=CURRENT_MODE;
        CURRENT_MODE = modes[CURRENT_MODE]->execute();
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionController>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}