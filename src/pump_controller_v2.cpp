#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <wiringPi.h>
#include <unistd.h>
#include <signal.h>

// Global für Signal Handler
int g_pwm_pin = 1;

void signal_handler(int signum) {
    printf("\nCtrl+C received - stopping pump safely...\n");
    pwmWrite(g_pwm_pin, 100);  // 1000µs neutral
    usleep(500000);
    pwmWrite(g_pwm_pin, 0);    // PWM off
    printf("Pump stopped safely\n");
    exit(0);
}

class PumpController : public rclcpp::Node {
public:
    PumpController() : Node("pump_controller"), pwm_pin_(1), pump_enabled_(false) {
        g_pwm_pin = pwm_pin_;  // Für Signal Handler
        
        RCLCPP_INFO(this->get_logger(), "=== PUMP CONTROLLER STARTING ===");
        
        init_pwm();
        RCLCPP_INFO(this->get_logger(), "PWM initialized successfully");
        
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/pump/speed_command", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if (!pump_enabled_) {
                    RCLCPP_WARN(this->get_logger(), "Speed command received but pump not enabled - waiting for /spray/start");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Speed command received: %.1f", msg->data);
                set_speed((int)msg->data);
            });
            
        start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/spray/start", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    RCLCPP_INFO(this->get_logger(), "=== SPRAY START RECEIVED - PUMP ENABLED ===");
                    pump_enabled_ = true;
                    set_speed(1000);  // Start with neutral
                } else {
                    RCLCPP_INFO(this->get_logger(), "=== SPRAY DISABLED ===");
                    pump_enabled_ = false;
                    set_speed(1000);  // Back to neutral
                }
            });
            
        stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/spray/stop", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Stop message received: %s", msg->data ? "true" : "false");
                if (msg->data) {
                    RCLCPP_INFO(this->get_logger(), "Stop signal received - shutting down pump");
                    shutdown_pump();
                    rclcpp::shutdown();
                }
            });
            
        RCLCPP_INFO(this->get_logger(), "Subscriptions created:");
        RCLCPP_INFO(this->get_logger(), "  - /pump/speed_command");
        RCLCPP_INFO(this->get_logger(), "  - /spray/start");
        RCLCPP_INFO(this->get_logger(), "  - /spray/stop");
        RCLCPP_INFO(this->get_logger(), "=== PUMP CONTROLLER READY - WAITING FOR START COMMAND ===");
    }
    
    ~PumpController() {
        shutdown_pump();
    }
    
    void shutdown_pump() {
        RCLCPP_INFO(this->get_logger(), "Stopping pump...");
        set_speed(1000);  // Neutral
        usleep(500000);   // 500ms wait
        pwmWrite(pwm_pin_, 0);  // PWM off
        RCLCPP_INFO(this->get_logger(), "Pump stopped safely");
    }

private:
    void init_pwm() {
        wiringPiSetupPinType(WPI_PIN_WPI);
        
        // Clean reset
        pinMode(pwm_pin_, OUTPUT);
        digitalWrite(pwm_pin_, LOW);
        usleep(50000);
        
        // PWM setup
        pinMode(pwm_pin_, PWM_OUTPUT);
        pwmWrite(pwm_pin_, 0);
        usleep(10000);
        
        pwmSetMode(PWM_MODE_MS);
        usleep(10000);
        pwmSetClock(192);
        usleep(10000);
        pwmSetRange(2000);
        usleep(10000);
        
        set_speed(1000);  // Start neutral
    }
    
    void set_speed(int us) {
        if (!pump_enabled_) {
            // Pumpe nicht enabled - immer neutral
            pwmWrite(pwm_pin_, 100);  // 1000µs neutral
            return;
        }
        
        if (us < 1000) us = 1000;
        if (us > 2000) us = 2000;
        pwmWrite(pwm_pin_, us / 10);
        RCLCPP_DEBUG(this->get_logger(), "Speed set to: %d us", us);
    }
    
    int pwm_pin_;
    bool pump_enabled_;  // Standardmäßig false
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
};

int main(int argc, char * argv[]) {
    // Signal Handler VOR rclcpp::init
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PumpController>());
    rclcpp::shutdown();
    return 0;
}