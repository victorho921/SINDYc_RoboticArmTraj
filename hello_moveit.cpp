#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    	rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    	
   //auto const eef_node = std::make_shared<rclcpp::Node>(
    //"get_eef_pose",
    	//rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    	
    

// We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  //rclcpp::executors::SingleThreadedExecutor executor;
  //executor.add_node(eef_node);
  //std::thread spinner = std::thread([&executor]() { executor.spin(); });
  //ros::AsyncSpinner spinner(1); spinner.start();


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "panda_arm");

// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  
  msg.orientation.x = 0.926;
  msg.orientation.y = -0.378;
  msg.orientation.z = 0.002;
  msg.orientation.w = 0.005;
    //msg.position.x = 0.28;
    //msg.position.y = -0.2;
    //msg.position.z = 0.5;
  // Starting pose  
  msg.position.x = 0.407902;
  msg.position.y = 0.6545998;
  msg.position.z = 0.367972; 
  // Ending pose
  //msg.position.x = 0.488;
  //msg.position.y = -0.366801;
  //msg.position.z = 0.633409;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
  // Shutdown ROS
  rclcpp::shutdown();
  //spinner.join();
  return 0;
}
