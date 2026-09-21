#include <memory>
#include <functional>
#include <chrono>
#include <string>

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

        // Log when the controller has successfully started.
        RCLCPP_INFO(
            this->get_logger(),
            "Finite state controller started"
        );
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
        // Check whether any of the monitored bump sensors have been pressed.
        if (
            msg->left_front ||
            msg->right_front ||
            msg->left_side ||
            msg->right_side
        ) {
            bump_detected_state_ = true;

            // Report the detected collision for debugging.
            RCLCPP_INFO(
                this->get_logger(),
                "Bump detected"
            );
        }
    }

    /**
     * @brief Processes LiDAR data to detect obstacles in front of the Neato.
     *
     * Searches the first and last 30 samples of the LaserScan message
     * and determines the closest measured range in that region.
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

        // Examine 30 LiDAR samples from each end of the scan.
        int samples_each_side = 30;
        int num_ranges = static_cast<int>(msg->ranges.size());

        // Begin with the maximum measurable range as the closest distance.
        double closest = msg->range_max;

        // Search the selected LiDAR samples for the closest measurement.
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

        // Set the obstacle flag when the closest object is within
        // the desired detection range.
        obstacle_detected_state_ =
            closest > 0.3 && closest < 0.5;

        // Report the closest detected range once per second for debugging.
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Closest front range: %.3f",
            closest
        );
    }

    /**
     * @brief Commands the Neato to drive straight forward.
     */
    void move_forward() {
        geometry_msgs::msg::Twist command;

        // Set positive linear velocity with no rotation.
        command.linear.x = 0.1;
        command.angular.z = 0.0;

        cmd_vel_publisher_->publish(command);
    }

    /**
     * @brief Commands the Neato to rotate in place.
     */
    void rotate() {
        geometry_msgs::msg::Twist command;

        // Set zero linear velocity and a negative angular velocity.
        command.linear.x = 0.0;
        command.angular.z = -0.1;

        cmd_vel_publisher_->publish(command);
    }

    /**
     * @brief Commands the Neato to stop moving.
     */
    void stop() {
        geometry_msgs::msg::Twist command;

        // Set both linear and angular velocity to zero.
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
            RCLCPP_INFO(
                this->get_logger(),
                "State: %s -> stop",
                neato_state_.c_str()
            );

            neato_state_ = "stop";
        }

        // Move forward until the LiDAR detects an obstacle.
        if (neato_state_ == "move_forward") {
            move_forward();

            // Transition to rotating when an obstacle is detected.
            if (obstacle_detected_state_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "State: move_forward -> rotate"
                );

                neato_state_ = "rotate";
            }
        }

        // Rotate until the LiDAR indicates that the path is clear.
        else if (neato_state_ == "rotate") {
            rotate();

            // Return to forward movement once the obstacle is gone.
            if (!obstacle_detected_state_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "State: rotate -> move_forward"
                );

                neato_state_ = "move_forward";
            }
        }

        // Any other state commands the robot to remain stopped.
        else {
            stop();
        }
    }

    // Timer used to execute the finite-state control loop.
    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher for sending velocity commands to the Neato.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_publisher_;

    // Subscriber for receiving physical bumper sensor messages.
    rclcpp::Subscription<neato2_interfaces::msg::Bump>::SharedPtr
        bump_subscriber_;

    // Subscriber for receiving LiDAR scan measurements.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscriber_;

    // Flags representing obstacle and bump detection.
    bool obstacle_detected_state_;
    bool bump_detected_state_;

    // Stores the current state of the finite-state machine.
    std::string neato_state_;
};

/**
 * @brief Initializes ROS 2 and runs the finite-state controller node.
 */
int main(int argc, char ** argv) {
    // Initialize ROS 2.
    rclcpp::init(argc, argv);

    // Create the finite-state controller node.
    auto node = std::make_shared<FiniteStateController>();

    // Process callbacks until the node is shut down.
    rclcpp::spin(node);

    // Cleanly shut down ROS 2.
    rclcpp::shutdown();

    return 0;
}