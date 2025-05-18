/**
* Program to print end-effector pose
*/

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>

int main(int argc, char* argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Declare Node
  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("get_eef_pose",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
        true));

std::ofstream csv_file("end_effector_pose.csv");
  if (!csv_file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open CSV file for writing.");
    rclcpp::shutdown();
    return 1;
  }

  csv_file << "Timestamp,x,y,z\n";

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "panda_arm");
    
  // Loop to print the pose at regular intervals
  rclcpp::Rate loop_rate(20.0);  // 1 Hz frequency (adjust as needed)
  
  while (rclcpp::ok()){

  // Get the current timestamp
  rclcpp::Time now = node->get_clock()->now();
  double timestamp = now.nanoseconds()/1e9;	
  
  // print current pose
  geometry_msgs::msg::Pose current_pose =
    move_group_interface.getCurrentPose().pose;

  // Print the current pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w
    );
  csv_file << std::fixed << std::setprecision(3) << timestamp << ","<< current_pose.position.x << ","<< current_pose.position.y << ","<< current_pose.position.z << "," <<
  current_pose.orientation.x << ","<< current_pose.orientation.y << ","<< current_pose.orientation.z << ","<< current_pose.orientation.w << '\n' ;
    
  loop_rate.sleep();}
  csv_file.close();
  
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
