#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cstdlib>
#include <unistd.h>
#include <csignal>
#include <atomic>

/* README
- Propeller ESC Controller Node
- to start from terminal
    --> Start/stop propeller with speed (0-100%)
        ros2 topic pub --once /propellers/speed_command std_msgs/msg/Float32 "data: 50.0"
*/

class PropellerControlNode : public rclcpp::Node
{
public:
    // Configuration constants for Propeller
    static constexpr int GPIO_PIN = 19;
    static constexpr int PWM_CLOCK = 19;
    static constexpr int PWM_RANGE = 20000;
    static constexpr int PWM_MIN = 1100;   // 0% = 1100µs
    static constexpr int PWM_MAX = 1940;   // 100% = 1940µs
    static constexpr int INIT_DELAY_SEC = 3;
    
    PropellerControlNode() : Node("propeller_controller_node"), shutdown_requested_(false)
    {
        // Initialize GPIO and Propeller ESC
        if (!initialize_propeller_esc()) {
            RCLCPP_ERROR(this->get_logger(), "Propeller ESC initialization failed!");
            return;
        }
        
        // ROS2 subscribers
        stop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/peripherie/stop", 10,
            std::bind(&PropellerControlNode::stop_callback, this, std::placeholders::_1));
        
        speed_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/propellers/speed_command", 10,
            std::bind(&PropellerControlNode::speed_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Propeller control node initialized and ready");
    }
    
    ~PropellerControlNode()
    {
        cleanup();
    }
    
    bool is_shutdown_requested() const
    {
        return shutdown_requested_;
    }
    
private:
    bool initialize_propeller_esc()
    {
        std::cout << "Initializing Propeller ESC on GPIO " << GPIO_PIN << "..." << std::endl;
        
        // Configure GPIO 19 as PWM
        if (system("gpio -g mode 19 pwm") != 0) {
            std::cerr << "Error: Failed to set GPIO mode for propeller" << std::endl;
            return false;
        }
        
        system("gpio pwm-ms");
        
        char cmd[100];
        sprintf(cmd, "gpio pwmc %d", PWM_CLOCK);
        system(cmd);
        
        sprintf(cmd, "gpio pwmr %d", PWM_RANGE);
        system(cmd);
        
        // Send zero position (1100µs) for propeller
        std::cout << "Sending propeller zero position (1100µs)..." << std::endl;
        set_pwm_value(PWM_MIN);
        
        std::cout << "Waiting for propeller ESC initialization..." << std::endl;
        sleep(INIT_DELAY_SEC);
        
        std::cout << "Propeller ESC ready!" << std::endl;
        
        return true;
    }
    
    void set_pwm_value(int pwm_value)
    {
        char cmd[100];
        sprintf(cmd, "gpio -g pwm %d %d", GPIO_PIN, pwm_value);
        system(cmd);
    }
    
    void set_propeller_speed_percent(double percent)
    {
        // Clamp to valid range
        if (percent < 0.0) percent = 0.0;
        if (percent > 100.0) percent = 100.0;
        
        // Mapping: 0% = 1100µs, 100% = 1940µs
        int pwm_value = PWM_MIN + static_cast<int>((percent / 100.0) * (PWM_MAX - PWM_MIN));
        
        RCLCPP_DEBUG(this->get_logger(), "Setting propeller to %.1f%% (PWM: %dµs)", percent, pwm_value);
        
        set_pwm_value(pwm_value);
    }
    
    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Stop signal received → shutting down propeller control node");
            shutdown_requested_ = true;
        }
    }
    
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        double speed_percent = static_cast<double>(msg->data);
        set_propeller_speed_percent(speed_percent);
    }
    
    void cleanup()
    {
        std::cout << "Shutting down propeller ESC safely..." << std::endl;
        
        // Set propeller to 0% before exit
        set_pwm_value(PWM_MIN);
        sleep(1);
        
        std::cout << "Propeller control node stopped" << std::endl;
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscriber_;
    std::atomic<bool> shutdown_requested_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PropellerControlNode>();
    
    try {
        while (rclcpp::ok() && !node->is_shutdown_requested()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception in propeller control main loop: " << e.what() << std::endl;
    }
    
    // Cleanup is handled in destructor
    node.reset();
    
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    
    return 0;
}