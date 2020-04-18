/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq S-Model (3 finger) device
 */

#ifndef ROBOTIQ_S_MODEL_ACTION_SERVER_H
#define ROBOTIQ_S_MODEL_ACTION_SERVER_H

// STL
#include <string>
// ROS standard
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
// Repo specific includes

#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>

namespace robotiq_action_server
{

typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput GripperOutput;
typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput GripperInput;

typedef control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryGoal;
typedef control_msgs::FollowJointTrajectoryFeedback FollowJointTrajectoryFeedback;
typedef control_msgs::FollowJointTrajectoryResult FollowJointTrajectoryResult;

/**
 * @brief The SModelGripperActionServer class. Takes as argument the name of the gripper it is to command.
 *        
 *        Listens for messages on input and publishes on output. Remap these.
 */
class SModelGripperActionServer
{
public:
  SModelGripperActionServer(const std::string& name);

  // These functions are meant to be called by simple action server
  void goalCB();
  void preemptCB();
  void analysisCB(const GripperInput::ConstPtr& msg);

private:
  void issueActivation();

  // Maps from joint state to the corresponding GripperOutput
  std::map<std::vector<double>, GripperOutput>::iterator get_goal_reg_state(std::vector<double>& goal_joint_state);

  // Compares joint states using the euclidean distance and an epsilon
  bool compare_joint_states(const std::vector<double> &js1, const std::vector<double> &js2, double eps);

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

  ros::Subscriber state_sub_; // Subs to grippers "input" topic
  ros::Publisher goal_pub_; // Pubs to grippers "output" topic

  GripperOutput goal_reg_state_; // Goal information in gripper-register form
  GripperInput current_reg_state_; // State info in gripper-register form

  std::string action_name_;
  std::string gripper_start_mode_;
  int gripper_speed_;

  std::map<std::vector<double>, GripperOutput> joint_state_to_reg_state_;
};

}
#endif
