#include "tm_driver/tm_ros2_movit_sct.h"
#include "tm_driver/tm_pvt_conversion.h"
void TmRos2SctMoveit::intial_action(){
    as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
     node->get_node_base_interface(),
     node->get_node_clock_interface(),
     node->get_node_logging_interface(),
     node->get_node_waitables_interface(),
     "tmr_arm_controller/follow_joint_trajectory",
     std::bind(&TmRos2SctMoveit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
     std::bind(&TmRos2SctMoveit::handle_cancel, this, std::placeholders::_1),
     std::bind(&TmRos2SctMoveit::handle_accepted, this, std::placeholders::_1)
    );

    goal_thread_open_ = false;
}

rclcpp_action::GoalResponse TmRos2SctMoveit::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal>)
{
  std::unique_lock<std::mutex> lck(goal_mtx_);
  
  auto goal_id = rclcpp_action::to_string(uuid);
  print_info("[ACTION] Received new action goal %s", goal_id.c_str());

  if (!is_fake_) {
    if (!svr_.is_connected()) {
      print_info("[ACTION] -----GOAL REJECT-----");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sct_.is_connected()) {
      print_info("[ACTION] -----GOAL REJECT-----");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (state_.has_error()) {
      print_info("[ACTION] -----GOAL REJECT-----");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (iface_.pvt_stop_unconfirmed()) {
    print_error("[ACTION] STOP ACK unresolved: driver restart required");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goals_queue_.empty() || goal_thread_open_ || goal_reserved_) {
    print_info("[ACTION] -----GOAL REJECT BUSY: no deferred PVT queue-----");
    return rclcpp_action::GoalResponse::REJECT;
  }

  goal_reserved_ = true;
  print_info("[ACTION] -----GOAL ACCEPT-----");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void TmRos2SctMoveit::handle_accepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  std::unique_lock<std::mutex> lock(goal_mtx_);
  goal_reserved_ = false;
  // Invalid goals receive an action result before any controller dispatch or
  // cancellation-gate reset. In particular, a short goal must never issue Stop.
  TmPvtTraj validated;
  std::string conversion_error;
  const auto &trajectory = goal_handle->get_goal()->trajectory;
  const bool joint_count_valid = trajectory.joint_names.size() == 6;
  if (!joint_count_valid) conversion_error = "PVT trajectory requires six joint names";
  if (!joint_count_valid ||
      !tm_convert_joint_pvt(validated, trajectory.points, 0.025, conversion_error)) {
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = result->INVALID_GOAL;
    result->error_string = conversion_error;
    goal_handle->abort(result);
    return;
  }
  if (!iface_.prepare_pvt_traj()) {
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = result->PATH_TOLERANCE_VIOLATED;
    result->error_string = "STOP ACK unresolved: driver restart required";
    goal_handle->abort(result);
    return;
  }
  goals_queue_.push_back(goal_handle);
  print_info("[ACTION] goals_queue_ size: %d", goals_queue_.size());
  // The previous worker sets open=false as its last locked operation. Once
  // admission succeeds it cannot need this mutex again, so joining is safe.
  if (goal_thread_.joinable()) goal_thread_.join();
  goal_thread_open_ = true;
  goal_thread_ = std::thread(&TmRos2SctMoveit::execute_goal_traj, this);
}

rclcpp_action::CancelResponse TmRos2SctMoveit::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  print_info("[ACTION CANCEL] Cancel request received for goal %s", goal_id.c_str());

  // if the goal is current
  if(goal_handle->is_executing()) {
    if (!iface_.stop_pvt_traj(!is_fake_)) {
      print_error("[ACTION CANCEL] controller stop ACK unresolved");
      return rclcpp_action::CancelResponse::REJECT;
    }
    print_info("[ACTION CANCEL] stop_pvt_traj()");
    print_info("[ACTION CANCEL] Goal %s execution canceled", goal_id.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // remove the goal from queue
  std::unique_lock<std::mutex> lock(goal_mtx_);
  auto it = std::find(goals_queue_.begin(), goals_queue_.end(), goal_handle);
  if(it != goals_queue_.end()) {
    goals_queue_.erase(it);
    print_info("[ACTION CANCEL] Deferd Goal canceled %s", goal_id.c_str());
    print_info("deferred_goal_handles queue size: %d", goals_queue_.size());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // the goal not exist
  print_warn("[ACTION CANCEL] Cancel request received for goal %s, but it is neither active nor deferred.", goal_id.c_str());
  return rclcpp_action::CancelResponse::REJECT;
}

void TmRos2SctMoveit::execute_goal_traj()
{
  print_info("[ACTION] -----goal execution thread begin-----");
  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle;
  {
    std::unique_lock<std::mutex> lock(goal_mtx_);
    if (goals_queue_.empty()) { goal_thread_open_ = false; return; }
    goal_handle = goals_queue_.front();
  }

    try {
      auto &traj_points = goal_handle->get_goal()->trajectory.points;
      auto pvts = std::make_shared<TmPvtTraj>();
      std::string conversion_error;
      if (!tm_convert_joint_pvt(*pvts, traj_points, 0.025, conversion_error)) {
        result->error_code = result->INVALID_GOAL;
        result->error_string = conversion_error;
        goal_handle->abort(result);
        std::unique_lock<std::mutex> lock(goal_mtx_);
        goals_queue_.clear();
        goal_thread_open_ = false;
        return;
      }
      bool is_match_start_pose = is_positions_match(traj_points.front(), 0.01);
      if (!is_match_start_pose) {
        print_warn("[ACTION] Start point doesn't match current pose");
      }

      print_info("[ACTION] traj. total time:= %d", (int)pvts->total_time);
      if (!goal_handle->is_executing()) {
        goal_handle->execute();
        print_info("[ACTION] traj executed");
      }

      if (!is_fake_) {
        const bool completed = iface_.run_pvt_traj(*pvts);
        if (!completed) {
          // The cancel callback waits for controller STOP ACK before returning
          // ACCEPT. Let rclcpp commit CANCELING before publishing terminal state.
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
          while (!goal_handle->is_canceling() && goal_handle->is_active() &&
                 std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (!completed && !goal_handle->is_canceling()) {
          result->error_code = result->PATH_TOLERANCE_VIOLATED;
          result->error_string = "PVT execution/cancellation acknowledgement unresolved";
          goal_handle->abort(result);
          std::unique_lock<std::mutex> lock(goal_mtx_);
          goals_queue_.clear();
          goal_thread_open_ = false;
          return;
        }
      }
      else {
        iface_.fake_run_pvt_traj(*pvts);
      }
      // ERROR, and clear goal queue
      if (iface_.is_sct_error) {
        result->error_code = result->INVALID_JOINTS;
        result->error_string = "TM_SCT get Error";
        print_error(result->error_string.c_str());
        iface_.is_sct_error = false;
        goal_handle->abort(result);
        {
        std::unique_lock<std::mutex> lck(goal_mtx_);
        goals_queue_.clear();
        goal_thread_open_ = false;
        }
        print_info("[ACTION] ERROR, clean all goals, goal execution thread end");
        return;
      }
      // CANCEL, and start next goal
      if (goal_handle->is_canceling()) {
        result->error_code = result->INVALID_GOAL;
        result->error_string = "Goal canceled";
        print_info(result->error_string.c_str());
        goal_handle->canceled(result);
        print_info("[ACTION] Current traj execution thread end");
      }
      // NORMAL, and start next goal
      else if (goal_handle->is_active()) {
        bool is_match_final_pose = is_positions_match(traj_points.back(), 0.01);
        if (!is_match_final_pose) {
          result->error_code = result->GOAL_TOLERANCE_VIOLATED;
          result->error_string = "Current pose doesn't match Goal point";
          print_warn(result->error_string.c_str());
          goal_handle->abort(result);
        }
        else {
          result->error_code = result->SUCCESSFUL;
          result->error_string = "Goal reached, success!";
          print_info(result->error_string.c_str());
          goal_handle->succeed(result);
        }
      }
    }
    catch (...) {
      result->error_code = result->INVALID_GOAL;
      result->error_string = "Goal fail !";
      goal_handle->abort(result);
    }
  {
    std::unique_lock<std::mutex> lck(goal_mtx_);
    goals_queue_.clear();
    goal_thread_open_ = false;
  }
  print_info("[ACTION] -----goal execution thread end-----");
}

void TmRos2SctMoveit::reorder_traj_joints(
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
  const trajectory_msgs::msg::JointTrajectory& traj)
{
  /* Reorders trajectory - destructive */
  std::vector<size_t> mapping;
  mapping.resize(jns_.size(), jns_.size());
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    for (size_t j = 0; j < jns_.size(); ++j) {
      if (traj.joint_names[i] == jns_[j])
        mapping[j] = i;
    }
  }
  new_traj_points.clear();
  for (unsigned int i = 0; i < traj.points.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
      new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj_points.push_back(new_point);
  }
}
bool TmRos2SctMoveit::has_points(const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (traj.points.size() == 0) return false;
  for (auto &point : traj.points) {
    if (point.positions.size() != traj.joint_names.size() ||
      point.velocities.size() != traj.joint_names.size()) return false;
  }
  return true;
}
bool TmRos2SctMoveit::is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj)
{
  for (auto &point : traj.points) {
    for (auto &p : point.positions) {
      if (!std::isfinite(p)) return false;
    }
    for (auto &v : point.velocities) {
      if (!std::isfinite(v)) return false;
    }
  }
  return true;
}
bool TmRos2SctMoveit::is_positions_match(
  const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps)
{
  auto q_act = state_.joint_angle();
  for (size_t i = 0; i < point.positions.size(); ++i) {
    if (fabs(point.positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}
