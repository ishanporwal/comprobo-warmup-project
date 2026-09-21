#include <memory>
#include <functional>
#include <chrono>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "neato2_interfaces/msg/bump.hpp"

/**
 * @brief Finite-state controller for autonomous Neato movement.
 *
 * The controller moves the Neato forward until an obstacle is detected
 * using the LiDAR. It then rotates until the path ahead is clear.
 * A bumper press causes the robot to enter the stop state.
 */
class FiniteStateController : public rclcpp::Node {
public:
    /**
     * @brief Initializes the controller, ROS interfaces, and control timer.
     */
    FiniteStateController()
        : Node("finite_state_controller"),
          obstacle_detected_state_(false),
          bump_detected_state_(false),
          neato_state_("move_forward")
    {
        // Run the finite-state control loop at 10 Hz.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FiniteStateController::run_loop, this)
        );

        // Publish velocity commands used to control the Neato's motion.
        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10
            );

        // Monitor the Neato's physical bump sensors for collisions.
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

        // Monitor LiDAR measurements for obstacles in front of the Neato.
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

        std::cout << "Finite state controller started" << std::endl;
        std::cout << "Initial state: move_forward" << std::endl;
    }

private:
    /**
     * @brief Processes messages from the Neato's bump sensors.
     *
     * Sets the bump-detected flag when any front or side bumper is pressed.
     *
     * @param msg Most recent bump sensor message.
     */
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

            std::cout << "Bump detected" << std::endl;
        }
    }

    /**
     * @brief Processes LiDAR data to detect obstacles in front of the Neato.
     *
     * Searches the first and last 30 samples of the LaserScan message,
     * which correspond to the region directly in front of the robot,
     * and determines the closest measured range.
     *
     * @param msg Most recent LiDAR scan.
     */
    void scan_callback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr msg
    ) {
        // Ignore scans that contain no range measurements.
        if (msg->ranges.empty()) {
            return;
        }

        // Examine 30 LiDAR samples on each side of the forward direction.
        int samples_each_side = 30;
        int num_ranges = static_cast<int>(msg->ranges.size());

        double closest = msg->range_max;

        // Find the closest range in the selected forward-facing region.
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

        // Treat objects between 0.3 m and 0.5 m as detected obstacles.
        obstacle_detected_state_ =
            closest > 0.3 && closest < 0.5;
    }

    /**
     * @brief Commands the Neato to drive straight forward.
     */
    void move_forward() {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.1;
        command.angular.z = 0.0;

        cmd_vel_publisher_->publish(command);
    }

    /**
     * @brief Commands the Neato to rotate in place.
     */
    void rotate() {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.0;
        command.angular.z = -0.1;

        cmd_vel_publisher_->publish(command);
    }

    /**
     * @brief Commands the Neato to stop moving.
     */
    void stop() {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.0;
        command.angular.z = 0.0;

        cmd_vel_publisher_->publish(command);
    }

    /**
     * @brief Executes one iteration of the finite-state control loop.
     *
     * The robot normally moves forward. When an obstacle is detected,
     * it transitions to the rotate state. Once the obstacle is no longer
     * detected, it returns to the move-forward state. A bumper press
     * transitions the robot to the stop state.
     */
    void run_loop() {
        // A bumper collision overrides normal navigation and stops the robot.
        if (bump_detected_state_ && neato_state_ != "stop") {
            std::cout
                << "State: "
                << neato_state_
                << " -> stop"
                << std::endl;

            neato_state_ = "stop";
        }

        // Move forward until the LiDAR detects an obstacle.
        if (neato_state_ == "move_forward") {
            move_forward();

            if (obstacle_detected_state_) {
                std::cout
                    << "Obstacle detected" << std::endl;

                std::cout
                    << "State: move_forward -> rotate"
                    << std::endl;

                neato_state_ = "rotate";
            }
        }

        // Rotate until the LiDAR indicates that the path is clear.
        else if (neato_state_ == "rotate") {
            rotate();

            if (!obstacle_detected_state_) {
                std::cout
                    << "Path clear" << std::endl;

                std::cout
                    << "State: rotate -> move_forward"
                    << std::endl;

                neato_state_ = "move_forward";
            }
        }

        // The stop state publishes a zero-velocity command.
        else {
            stop();
        }
    }

    // ROS interfaces used by the controller.
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_publisher_;

    rclcpp::Subscription<neato2_interfaces::msg::Bump>::SharedPtr
        bump_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscriber_;

    // Sensor flags and current finite-state-machine state.
    bool obstacle_detected_state_;
    bool bump_detected_state_;
    std::string neato_state_;
};

/**
 * @brief Initializes ROS 2 and runs the finite-state controller node.
 */
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FiniteStateController>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}