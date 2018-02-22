#include "robotiq_action_server/robotiq_s_model_action_server.h"

int main(int argc, char** argv)
{
  // Can be renamed with standard ROS-node launch interface
  ros::init(argc, argv, "gripper_action_server");
  
  // Private Note Handle for retrieving parameter arguments to the server
  ros::NodeHandle private_nh("~");

  std::string gripper_name;
  private_nh.param<std::string>("gripper_name", gripper_name, "gripper");

  ROS_INFO("Initializing Robotiq action server for gripper: %s", gripper_name.c_str());

  // The name of the gripper -> this server communicates over name/inputs and name/outputs
  robotiq_action_server::SModelGripperActionServer gripper (gripper_name);

  ROS_INFO("Robotiq action-server spinning for gripper: %s", gripper_name.c_str());
  ros::spin();
}
