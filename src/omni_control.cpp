#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wiringPi.h>
#include <softPwm.h>

#include <algorithm>
#include <array>
#include <chrono>

// Drives the 6 thruster H-bridges (2 GPIO pins each: forward, backward) with
// software PWM from /thrust_control (Float64MultiArray[6], -100..100).
//
// Safety: this node is the LAST hop before the motors, so it carries its own
// watchdog. The upstream thruster_bridge publishes continuously (commands at
// 50 Hz when armed, zeros from its own watchdog when idle), so a silent
// /thrust_control means the bridge itself died -- after 0.25 s of silence all
// PWM outputs are zeroed until fresh commands arrive.

class OmniControllerNode : public rclcpp::Node
{
public:
    OmniControllerNode(const std::array<int,12>& pins)
    : Node("omni_controller_node"), pins_(pins)
    {
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize WiringPi");
            throw std::runtime_error("WiringPi init failed");
        }

        for (int pin : pins_) {
            softPwmCreate(pin, 0, 100);
        }

        sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/thrust_control", 10,
            std::bind(&OmniControllerNode::thrustControlCallback, this, std::placeholders::_1)
        );

        watchdog_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OmniControllerNode::watchdogTick, this));
    }

    ~OmniControllerNode()
    {
        zeroAll();
    }

private:
    void zeroAll()
    {
        for (int pin : pins_) {
            softPwmWrite(pin, 0);
        }
    }

    void thrustControlCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "/thrust_control had %zu elements (<6); ignoring", msg->data.size());
            return;
        }

        for (size_t i = 0; i < 6; ++i) {
            double pwm = msg->data[i];

            int forward_pin = pins_[i*2];
            int backward_pin = pins_[i*2 + 1];

            if (pwm >= 0) {
                softPwmWrite(forward_pin, std::min(static_cast<int>(pwm), 100));
                softPwmWrite(backward_pin, 0);
            } else {
                softPwmWrite(forward_pin, 0);
                softPwmWrite(backward_pin, std::min(static_cast<int>(-pwm), 100));
            }
        }

        got_cmd_ = true;
        stale_ = false;
        last_cmd_ = this->now();
    }

    void watchdogTick()
    {
        if (!got_cmd_ || stale_) {
            return;  // nothing commanded yet (outputs still at init zeros) or already zeroed
        }
        if ((this->now() - last_cmd_).seconds() > 0.25) {
            zeroAll();
            stale_ = true;
            RCLCPP_WARN(this->get_logger(),
                "/thrust_control silent >0.25s: all PWM zeroed (upstream bridge dead?)");
        }
    }

    std::array<int,12> pins_;
    rclcpp::Time last_cmd_;
    bool got_cmd_ = false;
    bool stale_ = false;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::array<int,12> pins = {17,18, 5,6, 4,14, 20,26, 9,22, 23,25};
    if (argc == 13) {
        for (int i = 0; i < 12; ++i) {
            pins[i] = std::stoi(argv[i+1]);
        }
    }

    auto node = std::make_shared<OmniControllerNode>(pins);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
