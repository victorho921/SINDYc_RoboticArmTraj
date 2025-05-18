#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <fstream>
#include <vector>
#include <sstream>

class JointStateListener : public rclcpp::Node
{
public:
  JointStateListener()
  : Node("joint_state_listener"), recording_duration_(30.0) // Set recording duration (seconds)
  {
    // Create a subscription to the joint_states topic
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&JointStateListener::jointStateCallback, this, std::placeholders::_1));

    // Start the recording timer
    start_time_ = this->now();
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Check if the recording duration has passed
    if ((this->now() - start_time_).seconds() <= recording_duration_) {
      // Record the joint states
      recorded_data_.push_back(*msg);
      RCLCPP_INFO(this->get_logger(), "Recording joint states: [%s]", msg->name[0].c_str());
    } else {
      // Stop recording and save data after duration
      saveDataToFile();
      rclcpp::shutdown();
    }
  }

  void saveDataToFile()
  {
    std::ofstream outfile("joint_states_recorded.csv");
    if (outfile.is_open()) {
      outfile << "Time, Name, Position, Velocity, Effort\n";
      for (const auto& joint_state : recorded_data_) {
        std::stringstream ss;
        for (size_t i = 0; i < joint_state.name.size(); ++i) {
          ss << joint_state.header.stamp.sec << "." << joint_state.header.stamp.nanosec << ", " 
             << joint_state.name[i] << ", "
             << joint_state.position[i] << ", "
             << joint_state.velocity[i] << ", "
             << joint_state.effort[i] << "\n";
          outfile << ss.str();
          ss.str(""); // Clear the stringstream
        }
      }
      outfile.close();
      RCLCPP_INFO(this->get_logger(), "Data saved to joint_states_recorded.csv");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not open file for writing.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Time start_time_;
  std::vector<sensor_msgs::msg::JointState> recorded_data_;
  double recording_duration_; // Duration to record data in seconds
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateListener>());
  rclcpp::shutdown();
  return 0;
}
 
