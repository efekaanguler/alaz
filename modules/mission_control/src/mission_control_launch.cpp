#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <mission_control/mode_start.hpp>
#include <mission_control/mode_run.hpp>
#include <mission_control/mode_pause.hpp>
#include <mission_control/mode_park.hpp>
#include <mission_control/mode_emergency.hpp>
#include <string>
#include <memory>

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
        modes[MODE_PARK] = std::make_shared<ParkMode>(shared_this);
        modes[MODE_EMERGENCY] = std::make_shared<EmergencyMode>(shared_this);

        // NEW: Subscribe to a manual parking trigger
        manual_park_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission_control/manual_park", 10, 
            std::bind(&MissionController::manual_park_callback, this, std::placeholders::_1)
        );

        CURRENT_MODE = MODE_START;
        last_mode_=CURRENT_MODE;
        RCLCPP_INFO(this->get_logger(), "Vehicle Mode Set to %u", CURRENT_MODE);
    }

    private:
    uint8_t CURRENT_MODE=MODE_START;
    uint8_t last_mode_;
    std::map<unsigned int, std::shared_ptr<ModeBase>> modes;
    
    bool manual_park_requested_ = false;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_park_sub_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    void manual_park_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Manual park sequence requested by operator.");
            manual_park_requested_ = true;
        }
    }

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

        // 1. Check manual overrides before executing the normal state
        if (manual_park_requested_ && CURRENT_MODE != MODE_EMERGENCY && CURRENT_MODE != MODE_PARK) {
            CURRENT_MODE = MODE_PARK;
            manual_park_requested_ = false; // reset the latch
        }

        // 2. Execute the current state
        last_mode_=CURRENT_MODE;
        CURRENT_MODE = modes[CURRENT_MODE]->execute();
        
        // 3. Side-effect free emergency watchdog check via dynamic cast
        auto emergency_mode = std::dynamic_pointer_cast<EmergencyMode>(modes[MODE_EMERGENCY]);
        if(CURRENT_MODE != MODE_EMERGENCY && CURRENT_MODE != MODE_START && emergency_mode && emergency_mode->isEmergencyTriggered()) {
            last_mode_ = CURRENT_MODE;
            CURRENT_MODE = MODE_EMERGENCY;
        }
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