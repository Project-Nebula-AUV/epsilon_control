#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wiringPi.h>
#include <softPwm.h>
#include <vector>

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
    }

    ~OmniControllerNode()
    {
        for (int pin : pins_) {
            softPwmWrite(pin, 0);
        }
    }

private:
    void thrustControlCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> cmd = msg->data;

        for (size_t i = 0; i < 6; ++i) {
            double pwm = cmd[i];

            int forward_pin = pins_[i*2];
            int backward_pin = pins_[i*2 + 1];

            if (pwm >= 0) {
                softPwmWrite(forward_pin, std::min(static_cast<int>(pwm),100));
                softPwmWrite(backward_pin, 0);
            } else {
                softPwmWrite(forward_pin, 0);
                softPwmWrite(backward_pin, std::min(static_cast<int>(-pwm),100));
            }

            RCLCPP_INFO(this->get_logger(), "Thruster %d PWM: %f", static_cast<int>(i), pwm);
        }
    }

    std::array<int,12> pins_;
    std::array<std::array<int,6>,6> allocation_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::array<int,12> pins = {17,18, 22,23, 4,14, 20,26, 9,25, 6,12};
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