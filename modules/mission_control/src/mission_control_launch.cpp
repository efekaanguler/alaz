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
        auto start_mode = std::make_shared<StartMode>(shared_this);
        auto run_mode = std::make_shared<RunMode>(shared_this);
        auto pause_mode = std::make_shared<PauseMode>(shared_this);
        auto park_mode = std::make_shared<ParkMode>(shared_this);
        auto emergency_mode = std::make_shared<EmergencyMode>(shared_this);

        modes[MODE_START] = start_mode;
        modes[MODE_RUN] = run_mode;
        modes[MODE_PAUSE] = pause_mode;
        modes[MODE_PARK] = park_mode;
        modes[MODE_EMERGENCY] = emergency_mode;

        // Cache concrete pointers once instead of dynamic_pointer_cast'ing every tick
        run_mode_ = run_mode;
        pause_mode_ = pause_mode;
        park_mode_ = park_mode;
        emergency_mode_ = emergency_mode;

        // Manual operator overrides
        manual_park_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission_control/manual_park", 10,
            std::bind(&MissionController::manual_park_callback, this, std::placeholders::_1)
        );
        manual_resume_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission_control/manual_resume", 10,
            std::bind(&MissionController::manual_resume_callback, this, std::placeholders::_1)
        );

        CURRENT_MODE = MODE_START;
        last_mode_ = CURRENT_MODE;
        previous_executed_mode_ = MODE_START;
        RCLCPP_INFO(this->get_logger(), "Vehicle Mode Set to %u", CURRENT_MODE);
    }
    private:
    uint8_t CURRENT_MODE=MODE_START;
    uint8_t last_mode_;
    uint8_t previous_executed_mode_ = MODE_START;
    std::map<unsigned int, std::shared_ptr<ModeBase>> modes;

    std::shared_ptr<RunMode> run_mode_;
    std::shared_ptr<PauseMode> pause_mode_;
    std::shared_ptr<ParkMode> park_mode_;
    std::shared_ptr<EmergencyMode> emergency_mode_;
    
    bool manual_park_requested_ = false;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_park_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_resume_sub_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    void manual_park_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Manual park sequence requested by operator.");
            manual_park_requested_ = true;
        }
    }
    void manual_resume_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Manual resume/reset requested by operator.");
            if (park_mode_) park_mode_->requestResume();
            if (pause_mode_) pause_mode_->requestResume();
            if (emergency_mode_) emergency_mode_->requestReset();
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
            if (park_mode_) {
                park_mode_->requestManualHold();
            }
            CURRENT_MODE = MODE_PARK;
            manual_park_requested_ = false; // reset the latch
        }

        // 2. Publish health before executing RUN so the vehicle interface can
        //    apply its safe command in the same mission-control cycle.
        if (emergency_mode_) {
            const bool emergency_triggered = emergency_mode_->isEmergencyTriggered();
            const bool emergency_latched = CURRENT_MODE == MODE_EMERGENCY;
            emergency_mode_->publishEmergencyStop(emergency_triggered || emergency_latched);

            // START already owns readiness handling. All other modes are
            // preempted immediately when a monitored input becomes stale.
            if (emergency_triggered && CURRENT_MODE != MODE_START) {
                CURRENT_MODE = MODE_EMERGENCY;
            }
        }

        // 3. If we're entering RUN this tick from a different mode, let RunMode
        //    know so it re-publishes engage=true rather than assuming it's
        //    still engaged from a previous RUN session.
        if (run_mode_ && CURRENT_MODE == MODE_RUN && previous_executed_mode_ != MODE_RUN) {
            run_mode_->onEnter();
        }

        // 4. Execute the current state
        unsigned int mode_being_executed = CURRENT_MODE;
        last_mode_=CURRENT_MODE;
        CURRENT_MODE = modes[CURRENT_MODE]->execute();
        previous_executed_mode_ = mode_being_executed;
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
