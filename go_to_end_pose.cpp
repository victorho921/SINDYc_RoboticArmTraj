
#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>

// Add this function to create and publish a marker
void publishMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& marker_pub, 
                   const geometry_msgs::msg::Pose& pose, int id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "eef_markers";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker_pub->publish(marker);
}

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
  
  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "panda_arm");

// Construct and initialize MoveItVisualTools
//auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    //node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    //move_group_interface.getRobotModel()};
//moveit_visual_tools.deleteAllMarkers();
//moveit_visual_tools.loadRemoteControl();

//auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    //node, "base_link", "visualization_marker", move_group_interface.getRobotModel()};


// Create closures for visualization
// auto const draw_title = [&moveit_visual_tools](auto text) {
//   auto const text_pose = [] {
//     auto msg = Eigen::Isometry3d::Identity();
//     msg.translation().z() = 1.0;  // Place text 1m above the base link
//     return msg;
//   }();
//   moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
//                                   rviz_visual_tools::XLARGE);
// };
// auto const prompt = [&moveit_visual_tools](auto text) {
//   moveit_visual_tools.prompt(text);
// };
// auto const draw_trajectory_tool_path =
//     [&moveit_visual_tools,
//      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
//          "panda_arm")](auto const trajectory) {
//       moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
//     };

// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = 0.864;
  msg.orientation.y = -0.495;
  msg.orientation.z = 0.02;
  msg.orientation.w = 0.084;
  // Starting pose  
  //msg.position.x = 0.407902;
  //msg.position.y = 0.6545998;
  //msg.position.z = 0.367972; 
  // Ending pose
  msg.position.x = 0.479;
  msg.position.y = -0.374;
  msg.position.z = 0.637;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
// moveit_visual_tools.trigger();
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  // draw_trajectory_tool_path(plan.trajectory_);
  // moveit_visual_tools.trigger();
  
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
  move_group_interface.asyncExecute(plan);
  //move_group_interface.execute(plan);
  
  rclcpp::Rate loop_rate(1);  // 1 Hz
  int marker_id = 0;

  while (rclcpp::ok()) {
    // Get current pose of the end effector
    auto current_pose = move_group_interface.getCurrentPose().pose;
    
    // Publish a marker
    publishMarker(marker_pub, current_pose, marker_id++);
    
    loop_rate.sleep();
  }
  
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
