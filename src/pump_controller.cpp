#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cstdlib>
#include <unistd.h>
#include <csignal>
#include <atomic>

/* README
- Pump ESC Controller Node
- to start from terminal
    --> Start/stop pump with absolute PWM value (1000-2000µs)
        ros2 topic pub --once /pump/pwm_command std_msgs/msg/Float32 "data: 1500.0"
*/

class PumpControlNode : public rclcpp::Node
{
public:
    // Configuration constants for Pump
    static constexpr int GPIO_PIN = 18;
    static constexpr int PWM_CLOCK = 19;
    static constexpr int PWM_RANGE = 20000;
    static constexpr int PWM_MIN = 1000;   // Minimum = 1000µs
    static constexpr int PWM_MAX = 2000;   // Maximum = 2000µs
    static constexpr int INIT_DELAY_SEC = 3;
    
    PumpControlNode() : Node("pump_controller_node"), shutdown_requested_(false)
    {
        // Initialize GPIO and Pump ESC
        if (!initialize_pump_esc()) {
            RCLCPP_ERROR(this->get_logger(), "Pump ESC initialization failed!");
            return;
        }
        
        // ROS2 subscribers
        stop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/peripherie/stop", 10,
            std::bind(&PumpControlNode::stop_callback, this, std::placeholders::_1));
        
        pwm_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/pump/pwm_command", 10,
            std::bind(&PumpControlNode::pwm_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Pump control node initialized and ready");
    }
    
    ~PumpControlNode()
    {
        cleanup();
    }
    
    bool is_shutdown_requested() const
    {
        return shutdown_requested_;
    }
    
private:
    bool initialize_pump_esc()
    {
        std::cout << "Initializing Pump ESC on GPIO " << GPIO_PIN << "..." << std::endl;
        
        // Configure GPIO 18 as PWM
        if (system("gpio -g mode 18 pwm") != 0) {
            std::cerr << "Error: Failed to set GPIO mode for pump" << std::endl;
            return false;
        }
        
        system("gpio pwm-ms");
        
        char cmd[100];
        sprintf(cmd, "gpio pwmc %d", PWM_CLOCK);
        system(cmd);
        
        sprintf(cmd, "gpio pwmr %d", PWM_RANGE);
        system(cmd);
        
        // Send zero position (1000µs) for pump
        std::cout << "Sending pump zero position (1000µs)..." << std::endl;
        set_pwm_value(PWM_MIN);
        
        std::cout << "Waiting for pump ESC initialization..." << std::endl;
        sleep(INIT_DELAY_SEC);
        
        std::cout << "Pump ESC ready!" << std::endl;
        
        return true;
    }
    
    void set_pwm_value(int pwm_value)
    {
        char cmd[100];
        sprintf(cmd, "gpio -g pwm %d %d", GPIO_PIN, pwm_value);
        system(cmd);
    }
    
    void set_pump_pwm_absolute(double pwm_value_us)
    {
        // Clamp to valid range (1000-2000µs)
        if (pwm_value_us < PWM_MIN) pwm_value_us = PWM_MIN;
        if (pwm_value_us > PWM_MAX) pwm_value_us = PWM_MAX;
        
        int pwm_value = static_cast<int>(pwm_value_us);
        
        RCLCPP_DEBUG(this->get_logger(), "Setting pump to %dµs", pwm_value);
        
        set_pwm_value(pwm_value);
    }
    
    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Stop signal received → shutting down pump control node");
            shutdown_requested_ = true;
        }
    }
    
    void pwm_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        double pwm_value = static_cast<double>(msg->data);
        set_pump_pwm_absolute(pwm_value);
    }
    
    void cleanup()
    {
        std::cout << "Shutting down pump ESC safely..." << std::endl;
        
        // Set pump to minimum (1000µs) before exit
        set_pwm_value(PWM_MIN);
        sleep(1);
        
        std::cout << "Pump control node stopped" << std::endl;
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pwm_subscriber_;
    std::atomic<bool> shutdown_requested_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PumpControlNode>();
    
    try {
        while (rclcpp::ok() && !node->is_shutdown_requested()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception in pump control main loop: " << e.what() << std::endl;
    }
    
    // Cleanup is handled in destructor
    node.reset();
    
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    
    return 0;
}