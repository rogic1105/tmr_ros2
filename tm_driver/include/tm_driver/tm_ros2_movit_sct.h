#include "tm_ros2_sct.h"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class TmRos2SctMoveit : public TmSctRos2{
  private:
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr as_;
    std::mutex goal_mtx_;
    bool goal_thread_open_;
    bool goal_reserved_ = false;
    std::thread goal_thread_;
    std::deque< std::pair<rclcpp_action::GoalUUID, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal>> > deferred_goals_;
    std::deque< std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> > goals_queue_;
    TmSctCommunication &sct_;
    TmSvrCommunication &svr_;
    TmRobotState &state_;

    void intial_action();
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void handle_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void execute_goal_traj();
    void reorder_traj_joints(
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
      const trajectory_msgs::msg::JointTrajectory& traj);
    bool has_points(const trajectory_msgs::msg::JointTrajectory &traj);
    bool is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj);
    bool is_positions_match(
      const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps);
  public:
    TmRos2SctMoveit(rclcpp::Node::SharedPtr node, TmDriver &iface, bool is_fake) 
     : TmSctRos2(node, iface, is_fake)
     , sct_(iface.sct)
     , svr_(iface.svr)
     , state_(iface.state){
        intial_action();
    }
    ~TmRos2SctMoveit()
    {
      {
        std::lock_guard<std::mutex> lock(goal_mtx_);
        goals_queue_.clear();
      }
      if (goal_thread_.joinable())
      {
        goal_thread_.join();
      }
    }
};