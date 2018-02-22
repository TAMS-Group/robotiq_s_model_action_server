/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq S-Model device
 */

#include "robotiq_action_server/robotiq_s_model_action_server.h"

namespace robotiq_action_server
{

SModelGripperActionServer::SModelGripperActionServer(const std::string& name)
  : nh_(),
    as_(nh_, name, false),
    action_name_(name)
{
  ros::NodeHandle pn("~");

  std::vector<double> basic_open_joint_states = {0.107314, 0.0, -0.113446, 0.107314, 0.0, -0.113446, 
                                                 0.107314, 0.0, -0.113446, -0.016212, 0.016212};
  std::vector<double> basic_closed_joint_states = {1.22173, 1.5708, -0.959931, 1.22173, 1.5708, -0.959931, 
                                                   1.22173, 1.5708, -0.959931, -0.016212, 0.016212};
  std::vector<double> pinch_open_joint_states = {0.0495296, 0.0, -0.0523599, 0.0495296, 0.0, -0.0523599,
                                                 0.0495296, 0.0, -0.0523599, -0.1561488, 0.1561488};
  std::vector<double> pinch_closed_joint_states = {0.9328077, 0.0, -0.9861110, 0.9328077, 0.0, -0.9861110,
                                                   0.9328077, 0.0, -0.9861110, -0.1561488, 0.1561488};
  std::vector<double> wide_open_joint_states = {0.0495296, 0.0, -0.0523599, 0.0495296, 0.0, -0.0523599,
                                                0.0495296, 0.0, -0.0523599, 0.1766273, -0.1766273};
  std::vector<double> wide_closed_joint_states = {1.2217304, 1.5707963, -0.9599310, 1.2217304, 1.5707963, -0.9599310,
                                                  1.2217304, 1.5707963, -0.9599310, 0.1766273, -0.1766273};
  std::vector<double> scissor_closed_joint_states = {0.0495296, 0.0, -0.0523599, 0.0495296, 0.0, -0.0523599,
                                                     0.0495296, 0.0, -0.0523599, -0.1783338, 0.1783338};
  std::vector<double> wide_pinch_open_joint_states = {0.0495296, 0.0, -0.0523599, 0.0495296, 0.0, -0.0523599, 
                                                      0.0495296, 0.0, -0.0523599, 0.191986, -0.191986};
  std::vector<double> wide_pinch_closed_joint_states = {0.924553, 0.0, -0.977384, 0.924553, 0.0, -0.977384,
                                                        0.924553, 0.0, -0.977384, -0.14079, 0.14079};

  pn.param("gripper_start_mode", gripper_start_mode_, std::string("basic"));
  pn.param("gripper_speed", gripper_speed_, 128);

  if (gripper_speed_ < 0 || gripper_speed_ > 255)
  {
    ROS_WARN("Gripper speed was not set between 0 and 255. Defaulting to 128");
    gripper_speed_ = 128;
  }

  // Setting default values that are shared for all grasps
  GripperOutput go;
  go.rACT = 1;
  go.rGTO = 1;
  go.rSPA = gripper_speed_;
  go.rFRA = 150;

  // basic grasps
  go.rMOD = 0;
  go.rPRA = 0;
  if (!basic_open_joint_states.empty())
    joint_state_to_reg_state_[basic_open_joint_states] = go;
  go.rPRA = 255;
  if (!basic_closed_joint_states.empty())
    joint_state_to_reg_state_[basic_closed_joint_states] = go;

  // pinch grasps
  go.rMOD = 1;
  go.rPRA = 0;
  if (!pinch_open_joint_states.empty())
    joint_state_to_reg_state_[pinch_open_joint_states] = go;
  go.rPRA = 255;
  if (!pinch_closed_joint_states.empty())
    joint_state_to_reg_state_[pinch_closed_joint_states] = go;

  // wide grasps
  go.rMOD = 2;
  go.rPRA = 0;
  if (!wide_open_joint_states.empty())
    joint_state_to_reg_state_[wide_open_joint_states] = go;
  go.rPRA = 255;
  if (!wide_closed_joint_states.empty())
    joint_state_to_reg_state_[wide_closed_joint_states] = go;

  // scissor grasps
  // wide pinch open is identical to scissor open, so it is used instead
  go.rMOD = 3;
  go.rPRA = 0;
  go.rPRA = 255;
  if (!scissor_closed_joint_states.empty())
    joint_state_to_reg_state_[scissor_closed_joint_states] = go;

  // wide pinch grasps
  go.rMOD = 0;
  go.rICS = 1;
  go.rPRA = 6;
  go.rSPA = 80; // Hardcoded for now. Different values could lead to unexpected behavior
  go.rFRA = 127; // See comment above
  go.rPRS = 11;
  go.rSPS = 255;
  if (!wide_pinch_open_joint_states.empty())
    joint_state_to_reg_state_[wide_pinch_open_joint_states] = go;
  go.rPRA = 111;
  go.rPRS = 208;
  if (!wide_pinch_closed_joint_states.empty())
    joint_state_to_reg_state_[wide_pinch_closed_joint_states] = go;

  as_.registerGoalCallback(boost::bind(&SModelGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&SModelGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe("input", 1, &SModelGripperActionServer::analysisCB, this);
  goal_pub_ = nh_.advertise<GripperOutput>("output", 1);
  as_.start();
}

void SModelGripperActionServer::goalCB()
{
  // Check to see if the gripper is in an active state where it can take goals
  if (current_reg_state_.gIMC != 0x3)
  {
    ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
    return;
  }

  FollowJointTrajectoryGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  std::vector<double> current_goal_state = current_goal.trajectory.points.back().positions;

  // find equal joint state
  auto it = find_if(joint_state_to_reg_state_.begin(), joint_state_to_reg_state_.end(), [this, current_goal_state] (const std::pair<std::vector<double>, GripperOutput>& s) { return compare_joint_states(s.first, current_goal_state, 0.001); } );
  if(it != joint_state_to_reg_state_.end())
  {
     goal_reg_state_ = it->second;
     goal_pub_.publish(goal_reg_state_);
  }
  else
  {
    ROS_WARN("Provided grasp is not known.");
  }
}

void SModelGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void SModelGripperActionServer::analysisCB(const GripperInput::ConstPtr& msg)
{
  current_reg_state_ = *msg;
  // Check to see if the gripper is in its activated state
  if (current_reg_state_.gIMC != 0x3)
  {
    // Check to see if the gripper is active or if it has been asked to be active
    if (current_reg_state_.gIMC == 0x0 && goal_reg_state_.rACT != 0x1)
    {
      // If it hasn't been asked, active it
      issueActivation();
    }

    // Otherwise wait for the gripper to activate
    // TODO: If message delivery isn't guaranteed, then we may want to resend activate
    return;
  }
  if (!as_.isActive()) return;

  // Check for errors
  if (current_reg_state_.gFLT)
  {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
    FollowJointTrajectoryResult action_res;
    action_res.error_code = -5;
    as_.setAborted(action_res);
  }
  else if (current_reg_state_.gGTO && current_reg_state_.gIMC == 0x03 && current_reg_state_.gPRA == goal_reg_state_.rPRA && current_reg_state_.gSTA)
  {
    // If commanded to move and if at a goal state and if the position request matches the echo'd PR, we're
    // done with a moveregisterStateToR
    ROS_INFO("%s succeeded", action_name_.c_str());
    FollowJointTrajectoryResult action_res;
    action_res.error_code = 0;
    as_.setSucceeded(action_res);
  }
  // We could publish feedback here using the current_reg_state_.
}

void SModelGripperActionServer::issueActivation()
{
  ROS_INFO("Activating gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x1;
  out.rSPA = gripper_speed_;

  if (gripper_start_mode_ == "pinch")
    out.rMOD = 0x1;
  else if (gripper_start_mode_ == "wide")
    out.rMOD = 0x2;
  else if (gripper_start_mode_ == "scissor")
    out.rMOD = 0x3;
  else
  {
    if (gripper_start_mode_ != "basic")
    {
      ROS_WARN("No valid gripper start mode was given. Setting to basic mode.");
    }
    out.rMOD = 0x0;
  }

  // other params should be zero
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}

bool SModelGripperActionServer::compare_joint_states(const std::vector<double> &js1, const std::vector<double> &js2, double eps)
{
  if(js1.size() != js2.size())
  {
    ROS_WARN("The number of joints received deviates from the number of joints in the robotiq_s_model_action_server.");
    return false;
  }

  // Using euclidean distance for comparison
  double distance = 0.0;
  for(int i=0; i < js1.size(); i++)
  {
    distance += std::pow(js1[i] - js2[i], 2.0);
  }
  distance = std::sqrt(distance);

  if (distance <= eps)
    return true;

  return false;
}
} // end robotiq_action_server namespace
