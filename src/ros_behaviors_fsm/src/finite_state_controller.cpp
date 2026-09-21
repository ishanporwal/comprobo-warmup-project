#include <memory>
#include <functional>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "neato2_interfaces/msg/bump.hpp"

class FiniteStateController : public rclcpp::Node {
public:
    FiniteStateController()
        : Node("finite_state_controller"),
          obstacle_detected_state_(false),
          bump_detected_state_(false),
          neato_state_("move_forward")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FiniteStateController::run_loop, this)
        );
        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10
            );
        bump_subscriber_ =
            this->create_subscription<neato2_interfaces::msg::Bump>(
                "/bump",
                10,
                std::bind(
                    &FiniteStateController::bump_callback,
                    this,
                    std::placeholders::_1
                )
            );
        scan_subscriber_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &FiniteStateController::scan_callback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(this->get_logger(), "Finite state controller started");
    }

private:
    void bump_callback(
        const neato2_interfaces::msg::Bump::ConstSharedPtr msg
    ) {
        if (
            msg->left_front ||
            msg->right_front ||
            msg->left_side ||
            msg->right_side
        ) {
            bump_detected_state_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Bump detected"
            );
        }
    }

    void scan_callback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr msg
    ) {
        if (msg->ranges.empty()) {
            return;
        }
        int samples_each_side = 30;
        int num_ranges = static_cast<int>(msg->ranges.size());

        double closest = msg->range_max;

        for (int i = 0; i < samples_each_side; ++i) {
            double first = msg->ranges[i];
            double last = msg->ranges[num_ranges - 1 - i];

            if (first < closest) {
                closest = first;
            }

            if (last < closest) {
                closest = last;
            }
        }
        obstacle_detected_state_ =
            closest > 0.3 && closest < 0.5;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Closest front range: %.3f",
            closest
        );
    }

    void move_forward() {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.1;
        command.angular.z = 0.0;
        cmd_vel_publisher_->publish(command);
    }

    void rotate() {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.0;
        command.angular.z = -0.1;
        cmd_vel_publisher_->publish(command);
    }

    void stop() {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.0;
        command.angular.z = 0.0;
        cmd_vel_publisher_->publish(command);
    }

    void run_loop() {
        if (bump_detected_state_ && neato_state_ != "stop") {
            RCLCPP_INFO(
                this->get_logger(),
                "State: %s -> stop",
                neato_state_.c_str()
            );

            neato_state_ = "stop";
        }
        if (neato_state_ == "move_forward") {
            move_forward();

            if (obstacle_detected_state_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "State: move_forward -> rotate"
                );

                neato_state_ = "rotate";
            }
        }
        else if (neato_state_ == "rotate") {
            rotate();
            if (!obstacle_detected_state_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "State: rotate -> move_forward"
                );

                neato_state_ = "move_forward";
            }
        }
        else {
            stop();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_publisher_;
    rclcpp::Subscription<neato2_interfaces::msg::Bump>::SharedPtr
        bump_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscriber_;
    bool obstacle_detected_state_;
    bool bump_detected_state_;
    std::string neato_state_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FiniteStateController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}