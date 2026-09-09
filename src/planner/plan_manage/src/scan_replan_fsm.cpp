
#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace
{
  std::string shellQuote(const std::string &value)
  {
    std::string quoted = "'";
    for (const char c : value)
    {
      if (c == '\'')
        quoted += "'\\''";
      else
        quoted += c;
    }
    quoted += "'";
    return quoted;
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    local_detour_limit_exceeded_ = false;
    local_detour_rejection_message_.clear();
    last_freeze_update_time_ = ros::Time::now();
    action_goal_active_ = false;
    action_goal_adjusted_ = false;
    action_saw_trajectory_active_ = false;
    controller_trajectory_active_ = false;
    action_terminal_pending_ = false;
    action_terminal_code_ = scan_planner::NavigateToPoseResult::FAULT;
    odom_pos_.setZero();
    odom_vel_.setZero();
    odom_acc_.setZero();
    odom_orient_ = Eigen::Quaterniond::Identity();

    /*  fsm param  */
    nh.param("fsm/navi_mode", navi_mode_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizon_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    nh.param("fsm/terminal_clearance", terminal_clearance_, 0.25);
    terminal_clearance_ = std::max(0.0, terminal_clearance_);
    nh.param("fsm/max_local_detour_ratio", max_local_detour_ratio_, 0.0);
    nh.param("fsm/max_local_detour_m", max_local_detour_m_, 0.0);
    max_local_detour_ratio_ = std::max(0.0, max_local_detour_ratio_);
    max_local_detour_m_ = std::max(0.0, max_local_detour_m_);
    nh.param("grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    nh.param("grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    nh.param("grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    nh.param("grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    nh.param("grid_map/body_height", body_height_, 0.0);
    nh.param("grid_map/frame_id", self_inflation_frame_id_, std::string("world"));

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const std::string keypoints_yaml = "\"$(rospack find scan_planner)/../../../tools/keypoint.yaml\"";
      const std::string load_keypoints_cmd =
          "rosparam load " + keypoints_yaml + " " + shellQuote(nh.getNamespace());
      if (std::system(load_keypoints_cmd.c_str()) != 0)
      {
        ROS_ERROR("[SCANReplanFSM] Failed to load keypoints_yaml: tools/keypoint.yaml");
        ros::shutdown();
        return;
      }

      nh.param("fsm/waypoint_num", waypoint_num_, -1);

      if (waypoint_num_ <= 0)
      {
        ROS_ERROR("[SCANReplanFSM] navi_mode=2 requires keypoints_yaml with fsm/waypoint_num and fsm/waypoint{i}_{x,y,z}.");
        ros::shutdown();
        return;
      }
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        nh.param("fsm/waypoint" + to_string(i) + "_x", preset_waypoints_[i](0), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_y", preset_waypoints_[i](1), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_z", preset_waypoints_[i](2), -1.0);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &SCANReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &SCANReplanFSM::checkCollisionCallback, this);

    std::string body_pose_topic;
    ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    odom_sub_ = nh.subscribe(body_pose_topic, 1, &SCANReplanFSM::odometryCallback, this);
    go2_execution_frozen_sub_ = nh.subscribe("/planning/go2_execution_frozen", 10, &SCANReplanFSM::go2ExecutionFrozenCallback, this);
    trajectory_active_sub_ = nh.subscribe("/planning/trajectory_active", 10, &SCANReplanFSM::trajectoryActiveCallback, this);

    bspline_pub_ = nh.advertise<scan_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<scan_planner::DataDisp>("/planning/data_display", 100);
    self_inflation_pub_ = nh.advertise<visualization_msgs::Marker>("self_inflation", 10, true);
    navigation_active_pub_ = nh.advertise<std_msgs::Bool>("/planning/navigation_active", 1, true);
    goal_check_srv_ = nh.advertiseService("check_goal", &SCANReplanFSM::checkGoalCallback, this);
    setNavigationActive(false);

    navigate_action_server_.reset(new NavigateActionServer(nh, "navigate", false));
    navigate_action_server_->registerGoalCallback(boost::bind(&SCANReplanFSM::navigationGoalCallback, this));
    navigate_action_server_->registerPreemptCallback(boost::bind(&SCANReplanFSM::navigationPreemptCallback, this));
    navigate_action_server_->start();

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &SCANReplanFSM::rvizGoalCallback, this);
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = nh.subscribe("/initial_path", 1, &SCANReplanFSM::pathCallback, this);
    else
      cout << "Wrong navi_mode_ value! navi_mode_=" << navi_mode_ << endl;
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory to first preset waypoint!");
    }
  }

  void SCANReplanFSM::setNavigationActive(bool active)
  {
    std_msgs::Bool msg;
    msg.data = active;
    navigation_active_pub_.publish(msg);
  }

  void SCANReplanFSM::updateActionExecutedGoal()
  {
    if (!action_goal_active_)
      return;

    action_executed_goal_ = action_requested_goal_;
    action_executed_goal_.header.stamp = ros::Time::now();
    action_executed_goal_.pose.position.x = end_pt_(0);
    action_executed_goal_.pose.position.y = end_pt_(1);
    action_executed_goal_.pose.position.z = end_pt_(2);
    const double dx = end_pt_(0) - action_requested_goal_.pose.position.x;
    const double dy = end_pt_(1) - action_requested_goal_.pose.position.y;
    const double dz = end_pt_(2) - action_requested_goal_.pose.position.z;
    action_goal_adjusted_ = std::sqrt(dx * dx + dy * dy + dz * dz) > 1e-3;
  }

  void SCANReplanFSM::publishActionFeedback(uint8_t phase, const std::string &message)
  {
    if (!action_goal_active_ || !navigate_action_server_ || !navigate_action_server_->isActive())
      return;

    scan_planner::NavigateToPoseFeedback feedback;
    feedback.phase = phase;
    feedback.message = message;
    feedback.executed_goal = action_executed_goal_;
    feedback.replan_count = static_cast<uint32_t>(std::max(0, replan_fail_count_));
    navigate_action_server_->publishFeedback(feedback);
  }

  void SCANReplanFSM::queueActionTerminal(uint8_t completion_code, const std::string &message)
  {
    if (!action_goal_active_ || action_terminal_pending_)
      return;
    action_terminal_pending_ = true;
    action_terminal_code_ = completion_code;
    action_terminal_message_ = message;
    publishActionFeedback(scan_planner::NavigateToPoseFeedback::STOPPING, message);
    if (!controller_trajectory_active_)
      finishActionTerminal();
  }

  void SCANReplanFSM::finishActionTerminal()
  {
    if (!action_goal_active_ || !action_terminal_pending_ || controller_trajectory_active_ ||
        !navigate_action_server_ || !navigate_action_server_->isActive())
      return;

    scan_planner::NavigateToPoseResult result;
    result.completion_code = action_terminal_code_;
    result.reached_requested_goal =
        action_terminal_code_ == scan_planner::NavigateToPoseResult::REACHED_REQUESTED_GOAL;
    result.goal_adjusted = action_goal_adjusted_;
    result.message = action_terminal_message_;
    result.requested_goal = action_requested_goal_;
    result.executed_goal = action_executed_goal_;
    result.final_pose.header.stamp = ros::Time::now();
    result.final_pose.header.frame_id = action_requested_goal_.header.frame_id;
    result.final_pose.pose.position.x = odom_pos_(0);
    result.final_pose.pose.position.y = odom_pos_(1);
    result.final_pose.pose.position.z = odom_pos_(2);
    result.final_pose.pose.orientation.w = odom_orient_.w();
    result.final_pose.pose.orientation.x = odom_orient_.x();
    result.final_pose.pose.orientation.y = odom_orient_.y();
    result.final_pose.pose.orientation.z = odom_orient_.z();

    const double adjust_x = action_executed_goal_.pose.position.x - action_requested_goal_.pose.position.x;
    const double adjust_y = action_executed_goal_.pose.position.y - action_requested_goal_.pose.position.y;
    const double adjust_z = action_executed_goal_.pose.position.z - action_requested_goal_.pose.position.z;
    result.adjustment_distance = std::sqrt(adjust_x * adjust_x + adjust_y * adjust_y + adjust_z * adjust_z);
    const double final_x = odom_pos_(0) - action_requested_goal_.pose.position.x;
    const double final_y = odom_pos_(1) - action_requested_goal_.pose.position.y;
    const double final_z = odom_pos_(2) - action_requested_goal_.pose.position.z;
    result.final_distance_to_requested = std::sqrt(final_x * final_x + final_y * final_y + final_z * final_z);

    if (action_terminal_code_ == scan_planner::NavigateToPoseResult::REACHED_REQUESTED_GOAL ||
        action_terminal_code_ == scan_planner::NavigateToPoseResult::REACHED_ADJUSTED_GOAL)
      navigate_action_server_->setSucceeded(result, result.message);
    else if (action_terminal_code_ == scan_planner::NavigateToPoseResult::CANCELED)
      navigate_action_server_->setPreempted(result, result.message);
    else
      navigate_action_server_->setAborted(result, result.message);

    ROS_INFO("[SCANReplanFSM] navigation action finished code=%u adjusted=%s distance_to_requested=%.3f: %s",
             static_cast<unsigned int>(result.completion_code), result.goal_adjusted ? "true" : "false",
             result.final_distance_to_requested, result.message.c_str());
    action_goal_active_ = false;
    action_terminal_pending_ = false;
    action_saw_trajectory_active_ = false;
    setNavigationActive(false);
  }

  void SCANReplanFSM::trajectoryActiveCallback(const std_msgs::BoolConstPtr &msg)
  {
    controller_trajectory_active_ = msg->data;
    if (action_goal_active_ && msg->data)
    {
      action_saw_trajectory_active_ = true;
      publishActionFeedback(scan_planner::NavigateToPoseFeedback::EXECUTING, "SCAN controller is executing a trajectory");
    }
    if (!msg->data)
    {
      finishActionTerminal();
      if (!action_goal_active_ && exec_state_ == WAIT_TARGET)
        setNavigationActive(false);
    }
  }

  void SCANReplanFSM::navigationGoalCallback()
  {
    if (!navigate_action_server_ || !navigate_action_server_->isNewGoalAvailable())
      return;

    const scan_planner::NavigateToPoseGoalConstPtr goal = navigate_action_server_->acceptNewGoal();
    action_goal_active_ = true;
    action_goal_adjusted_ = false;
    action_saw_trajectory_active_ = false;
    action_terminal_pending_ = false;
    action_terminal_message_.clear();
    action_requested_goal_ = goal->target;
    action_requested_goal_.header.stamp = ros::Time::now();
    action_executed_goal_ = action_requested_goal_;
    setNavigationActive(true);
    publishActionFeedback(scan_planner::NavigateToPoseFeedback::PLANNING, "SCAN accepted the navigation goal");

    if (!startManualGoal(action_requested_goal_, true))
    {
      queueActionTerminal(scan_planner::NavigateToPoseResult::REJECTED_NO_GLOBAL_PATH,
                          have_odom_ ? "SCAN could not generate a global path" : "SCAN has no odometry");
    }
    else
    {
      updateActionExecutedGoal();
      publishActionFeedback(scan_planner::NavigateToPoseFeedback::PLANNING,
                            action_goal_adjusted_ ? "SCAN adjusted the occupied goal and is planning" :
                                                    "SCAN found a global path and is planning");
    }
  }

  void SCANReplanFSM::navigationPreemptCallback()
  {
    if (!action_goal_active_ || action_terminal_pending_)
      return;

    queueActionTerminal(scan_planner::NavigateToPoseResult::CANCELED,
                        "Navigation canceled by AgenticNav or the safety supervisor");
    if (controller_trajectory_active_ && have_odom_)
    {
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "ACTION_CANCEL");
    }
    else
    {
      have_target_ = false;
      trigger_ = false;
      changeFSMExecState(WAIT_TARGET, "ACTION_CANCEL");
      finishActionTerminal();
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (!msg)
      return;

    if (action_goal_active_)
    {
      ROS_WARN("[SCANReplanFSM] Ignore RViz goal while an AgenticNav action is active.");
      return;
    }

    startManualGoal(*msg, false);
  }

  bool SCANReplanFSM::startManualGoal(const geometry_msgs::PoseStamped &goal, bool from_action)
  {
    if (navi_mode_ != NAVI_MODE::MANUAL_TARGET)
    {
      ROS_WARN("[SCANReplanFSM] Reject manual navigation goal in navi_mode=%d.", navi_mode_);
      return false;
    }

    if (!rviz_height_ready_)
    {
      ROS_WARN("[SCANReplanFSM] Ignore RViz goal before receiving initial body pose.");
      Eigen::Vector3d goal_point(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      visualization_->displayPlanningStatus(goal_point, "PLAN FAILED: NO ODOM", Eigen::Vector4d(1.0, 0.1, 0.1, 1.0));
      return false;
    }

    nav_msgs::PathPtr path(new nav_msgs::Path);
    path->header = goal.header;
    path->poses.push_back(goal);
    setNavigationActive(true);
    const bool started = waypointCallback(path);
    if (!started && !from_action)
      setNavigationActive(false);
    return started;
  }

  bool SCANReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[waypointCallback] Empty waypoint message, ignore.");
      return false;
    }

    if (msg->poses[0].pose.position.z < -1.0)
      return false;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    visualization_->clearCurrentPlan();
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1.0, 0.8, 0.0, 1.0), 0.3, 0);
    visualization_->displayPlanningStatus(end_pt_, "GOAL RECEIVED - PLANNING", Eigen::Vector4d(1.0, 0.8, 0.0, 1.0));
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0.0, 0.8, 1.0, 1.0), 0.3, 0);
      visualization_->displayPlanningStatus(end_pt_, "GLOBAL PATH OK - OPTIMIZING", Eigen::Vector4d(1.0, 0.8, 0.0, 1.0));
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1.0, 0.1, 0.1, 1.0), 0.3, 0);
      visualization_->displayPlanningStatus(end_pt_, "PLAN FAILED: NO GLOBAL PATH", Eigen::Vector4d(1.0, 0.1, 0.1, 1.0));
      ROS_ERROR("Unable to generate global trajectory!");
    }
    return success;
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2)
    {
      ROS_WARN("[planGlobalTrajByWaypoints] Reference path requires at least two points.");
      return false;
    }

    end_pt_ = waypoints.back();
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("Unable to generate global trajectory from waypoints!");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      ROS_WARN("[navi_mode=%d] No active waypoint to plan.", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("[navi_mode=%d] Unable to generate trajectory to waypoint %d.", navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    ROS_INFO("[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f].",
             navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    int first_free_idx = -1;
    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        first_free_idx = i;
        break;
      }
    }

    if (first_free_idx < 0)
    {
      ROS_ERROR("[global target] Target is occupied, and no collision-free point was found along the global trajectory.");
      return false;
    }

    int target_idx = terminal_clearance_ <= 1e-6 ? first_free_idx : -1;
    double backed_distance = 0.0;
    Eigen::Vector3d previous_pt = global_data.global_traj_.evaluate(duration * first_free_idx / sample_num);
    for (int i = first_free_idx - 1; i >= 0 && target_idx < 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);
      backed_distance += (previous_pt - pt).norm();
      previous_pt = pt;
      if (backed_distance + 1e-6 >= terminal_clearance_ &&
          map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
        target_idx = i;
    }

    if (target_idx < 0)
    {
      ROS_ERROR("[global target] Found a collision-free boundary point, but the route cannot provide %.2f m terminal clearance.",
                terminal_clearance_);
      return false;
    }

    const Eigen::Vector3d raw_end = end_pt_;
    const Eigen::Vector3d first_free = global_data.global_traj_.evaluate(duration * first_free_idx / sample_num);
    const double target_t = duration * target_idx / sample_num;
    end_pt_ = global_data.global_traj_.evaluate(target_t);
    global_data.global_duration_ = target_t;
    global_data.last_progress_time_ = std::min(global_data.last_progress_time_, target_t);
    updateActionExecutedGoal();
    ROS_WARN("[global target] Target [%.2f, %.2f, %.2f] is occupied; first free [%.2f, %.2f, %.2f], use %.2f m-clearance target [%.2f, %.2f, %.2f].",
             raw_end(0), raw_end(1), raw_end(2), first_free(0), first_free(1), first_free(2),
             terminal_clearance_, end_pt_(0), end_pt_(1), end_pt_(2));
    return true;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[pathCallback] Received empty /initial_path, ignore.");
      return;
    }

    if (!have_odom_)
    {
      ROS_WARN_THROTTLE(1.0, "[pathCallback] No odometry yet, cannot plan global trajectory.");
      return;
    }

    trigger_ = true;
    end_pt_ << msg->poses.back().pose.position.x,
        msg->poses.back().pose.position.y,
        msg->poses.back().pose.position.z + body_height_;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());
    constexpr double min_dist = 0.5;
    Eigen::Vector3d last_wp;
    bool first = true;

    for (const auto &pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_;

      if (first || (wp - last_wp).norm() >= min_dist)
      {
        waypoints.push_back(wp);
        last_wp = wp;
        first = false;
      }
    }

    if ((waypoints.back() - end_pt_).norm() > 1e-6)
      waypoints.push_back(end_pt_);

    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      ROS_INFO("==========================================\n");
    }
    else
    {
      ROS_ERROR("❌ Unable to generate global trajectory!");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
    {
      // A legged robot may start the sensor stack while crouched and stand up
      // before receiving its first goal. Keep the manual planning plane on
      // the current body height instead of freezing the first odometry z.
      rviz_goal_height_ = odom_pos_(2);
      if (!rviz_height_ready_)
        ROS_INFO("[SCANReplanFSM] Tracking manual goal height from body_pose z: %.3f", rviz_goal_height_);
      rviz_height_ready_ = true;
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  bool SCANReplanFSM::checkGoalCallback(scan_planner::CheckGoal::Request &request,
                                        scan_planner::CheckGoal::Response &response)
  {
    response.planned_goal = request.goal;
    response.planned_goal.header.stamp = ros::Time::now();
    response.planned_path.header = response.planned_goal.header;

    if (navi_mode_ != NAVI_MODE::MANUAL_TARGET)
    {
      response.success = false;
      response.message = "SCAN-Planner goal checking requires manual-target mode";
      return true;
    }
    if (!have_odom_ || !rviz_height_ready_)
    {
      response.success = false;
      response.message = "SCAN-Planner is waiting for body odometry";
      return true;
    }

    // A VLM candidate must be checked by the same global and local planning
    // algorithms used for execution, but a check must never publish a
    // trajectory or alter the currently executing FSM. Save the planner/FSM
    // trajectory state, run a deterministic attempt plus one randomized
    // retry, then restore everything before returning the result.
    const GlobalTrajData global_backup = planner_manager_->global_data_;
    const LocalTrajData local_backup = planner_manager_->local_data_;
    const int failure_count_backup = planner_manager_->continuousFailuresCount();
    const Eigen::Vector3d start_pt_backup = start_pt_;
    const Eigen::Vector3d start_vel_backup = start_vel_;
    const Eigen::Vector3d start_acc_backup = start_acc_;
    const Eigen::Vector3d end_pt_backup = end_pt_;
    const Eigen::Vector3d end_vel_backup = end_vel_;
    const Eigen::Vector3d local_target_pt_backup = local_target_pt_;
    const Eigen::Vector3d local_target_vel_backup = local_target_vel_;

    end_pt_ << request.goal.pose.position.x,
        request.goal.pose.position.y,
        rviz_goal_height_;
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    end_vel_.setZero();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_, start_vel_, start_acc_, end_pt_,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    std::string message;
    if (!success)
    {
      message = "SCAN-Planner failed to generate a global trajectory";
    }
    else if (!adjustGlobalTargetIfOccupied())
    {
      success = false;
      message = "SCAN-Planner found no collision-free target along the route";
    }
    else
    {
      getLocalTarget();
      success = planner_manager_->reboundReplan(
          start_pt_, start_vel_, start_acc_, local_target_pt_,
          local_target_vel_, true, false);
      if (!success)
      {
        success = planner_manager_->reboundReplan(
            start_pt_, start_vel_, start_acc_, local_target_pt_,
            local_target_vel_, true, true);
      }
      message = success
                    ? "SCAN-Planner found a collision-free trajectory"
                    : "SCAN-Planner failed to generate a collision-free local trajectory";
    }

    const Eigen::Vector3d planned_end = end_pt_;
    std::vector<Eigen::Vector3d> checked_local_path;
    if (success)
    {
      checked_local_path = sampleLocalTrajectory();
      if (checked_local_path.size() < 2)
      {
        success = false;
        message = "SCAN-Planner generated an empty local trajectory";
      }
      else
      {
        const double endpoint_error =
            (checked_local_path.back().head<2>() - planned_end.head<2>()).norm();
        if (endpoint_error > 0.15)
        {
          success = false;
          std::ostringstream stream;
          stream << std::fixed << std::setprecision(2)
                 << "SCAN-Planner local trajectory stops " << endpoint_error
                 << "m before the requested current-step goal";
          message = stream.str();
        }
        else
        {
          double path_distance = 0.0;
          double direct_distance = 0.0;
          double detour_ratio = 0.0;
          double detour_distance = 0.0;
          if (!localTrajectoryDetourAcceptable(
                  message, &path_distance, &direct_distance,
                  &detour_ratio, &detour_distance))
          {
            success = false;
          }
          else
          {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(2)
                   << "SCAN-Planner found a collision-free local trajectory: path="
                   << path_distance << "m, direct=" << direct_distance
                   << "m, ratio=" << detour_ratio << ", extra="
                   << detour_distance << "m";
            message = stream.str();
          }
        }
      }
    }
    if (success)
    {
      response.planned_path.poses.reserve(checked_local_path.size());
      for (const Eigen::Vector3d &point : checked_local_path)
      {
        geometry_msgs::PoseStamped pose;
        pose.header = response.planned_path.header;
        pose.pose.position.x = point(0);
        pose.pose.position.y = point(1);
        pose.pose.position.z = point(2);
        pose.pose.orientation = request.goal.pose.orientation;
        response.planned_path.poses.push_back(pose);
      }
      response.planned_path.poses.front().pose.position.x = odom_pos_(0);
      response.planned_path.poses.front().pose.position.y = odom_pos_(1);
      response.planned_path.poses.front().pose.position.z = odom_pos_(2);
      response.planned_path.poses.back().pose.position.x = planned_end(0);
      response.planned_path.poses.back().pose.position.y = planned_end(1);
      response.planned_path.poses.back().pose.position.z = planned_end(2);
    }
    planner_manager_->global_data_ = global_backup;
    planner_manager_->local_data_ = local_backup;
    planner_manager_->restoreContinuousFailuresCount(failure_count_backup);
    start_pt_ = start_pt_backup;
    start_vel_ = start_vel_backup;
    start_acc_ = start_acc_backup;
    end_pt_ = end_pt_backup;
    end_vel_ = end_vel_backup;
    local_target_pt_ = local_target_pt_backup;
    local_target_vel_ = local_target_vel_backup;

    response.success = success;
    response.message = message;
    response.planned_goal.pose.position.x = planned_end(0);
    response.planned_goal.pose.position.y = planned_end(1);
    response.planned_goal.pose.position.z = planned_end(2);
    ROS_INFO("[goal check] candidate [%.2f, %.2f] -> %s: %s; path_points=%zu",
             request.goal.pose.position.x, request.goal.pose.position.y,
             success ? "accepted" : "rejected", message.c_str(),
             response.planned_path.poses.size());
    return true;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const ros::Time now = ros::Time::now();
    double dt = (now - last_freeze_update_time_).toSec();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.toSec() > 1e-5)
      info->start_time_ += ros::Duration(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = ros::Duration(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_.publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_.publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;

    if (action_goal_active_ && !action_terminal_pending_)
    {
      if (new_state == GEN_NEW_TRAJ)
        publishActionFeedback(scan_planner::NavigateToPoseFeedback::PLANNING, "SCAN is generating a local trajectory");
      else if (new_state == REPLAN_TRAJ)
        publishActionFeedback(scan_planner::NavigateToPoseFeedback::REPLANNING, "SCAN is replanning around updated obstacles");
      else if (new_state == EXEC_TRAJ)
        publishActionFeedback(scan_planner::NavigateToPoseFeedback::EXECUTING, "SCAN is executing the navigation goal");
      else if (new_state == EMERGENCY_STOP)
        publishActionFeedback(scan_planner::NavigateToPoseFeedback::STOPPING, "SCAN is performing an emergency stop");
    }
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        visualization_->displayPlanningStatus(end_pt_, "PLAN OK", Eigen::Vector4d(0.1, 1.0, 0.2, 1.0));
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        if (local_detour_limit_exceeded_)
        {
          abortForExcessiveDetour("INITIAL_PLAN_DETOUR");
          break;
        }
        replan_fail_count_++;
        if (replan_fail_count_ == 1 || replan_fail_count_ % 50 == 0)
        {
          visualization_->displayPlanningStatus(
              end_pt_, "SEARCHING LOCAL PATH - RETRY " + std::to_string(replan_fail_count_),
              Eigen::Vector4d(1.0, 0.5, 0.0, 1.0));
        }
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        visualization_->displayPlanningStatus(end_pt_, "REPLAN OK", Eigen::Vector4d(0.1, 1.0, 0.2, 1.0));
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        if (local_detour_limit_exceeded_)
        {
          abortForExcessiveDetour("RUNTIME_REPLAN_DETOUR");
          break;
        }
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        visualization_->displayPlanningStatus(end_pt_, "GOAL REACHED", Eigen::Vector4d(0.1, 1.0, 0.2, 1.0));

        if (action_goal_active_)
        {
          updateActionExecutedGoal();
          queueActionTerminal(
              action_goal_adjusted_ ? scan_planner::NavigateToPoseResult::REACHED_ADJUSTED_GOAL
                                    : scan_planner::NavigateToPoseResult::REACHED_REQUESTED_GOAL,
              action_goal_adjusted_ ? "SCAN reached the adjusted collision-free goal"
                                    : "SCAN reached the requested goal");
        }

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
          finishActionTerminal();
          if (!action_goal_active_)
            setNavigationActive(false);
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      ROS_WARN("Replan failed %d times. Emergency stop and wait for a new target.", replan_fail_count_);
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1.0, 0.1, 0.1, 1.0), 0.3, 0);
      visualization_->displayPlanningStatus(end_pt_, "PLAN FAILED: NO LOCAL PATH", Eigen::Vector4d(1.0, 0.1, 0.1, 1.0));
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      queueActionTerminal(scan_planner::NavigateToPoseResult::ABORTED_NO_LOCAL_PATH,
                          "SCAN exhausted local replanning and stopped safely");
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  std::vector<Eigen::Vector3d> SCANReplanFSM::sampleLocalTrajectory(double sample_period_s)
  {
    std::vector<Eigen::Vector3d> points;
    LocalTrajData &info = planner_manager_->local_data_;
    if (info.duration_ <= 1e-6)
      return points;

    const double period = std::max(0.01, sample_period_s);
    const int sample_count = std::max(
        2, static_cast<int>(std::ceil(info.duration_ / period)) + 1);
    points.reserve(sample_count);
    for (int index = 0; index < sample_count; ++index)
    {
      const double ratio = static_cast<double>(index) /
                           static_cast<double>(sample_count - 1);
      points.push_back(info.position_traj_.evaluateDeBoorT(info.duration_ * ratio));
    }
    return points;
  }

  bool SCANReplanFSM::localTrajectoryDetourAcceptable(
      std::string &message,
      double *path_distance,
      double *direct_distance,
      double *detour_ratio,
      double *detour_distance)
  {
    const std::vector<Eigen::Vector3d> points = sampleLocalTrajectory();
    if (points.size() < 2)
    {
      message = "SCAN-Planner generated an empty local trajectory";
      return false;
    }

    double path = 0.0;
    for (size_t index = 1; index < points.size(); ++index)
      path += (points[index].head<2>() - points[index - 1].head<2>()).norm();
    const double direct = (points.back().head<2>() - points.front().head<2>()).norm();
    const double ratio = path / std::max(direct, 1e-3);
    const double extra = path - direct;
    if (path_distance)
      *path_distance = path;
    if (direct_distance)
      *direct_distance = direct;
    if (detour_ratio)
      *detour_ratio = ratio;
    if (detour_distance)
      *detour_distance = extra;

    bool safe_mode = false;
    ros::param::param<bool>("/agenticnav/safe_mode", safe_mode, false);
    if (!safe_mode)
      return true;

    const bool ratio_exceeded =
        max_local_detour_ratio_ > 0.0 && ratio > max_local_detour_ratio_;
    const bool distance_exceeded =
        max_local_detour_m_ > 0.0 && extra > max_local_detour_m_;
    if (!ratio_exceeded && !distance_exceeded)
      return true;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "SCAN rejected excessive local detour before execution: path="
           << path << "m, direct=" << direct << "m, ratio=" << ratio
           << " (max " << max_local_detour_ratio_ << "), extra=" << extra
           << "m (max " << max_local_detour_m_ << "m)";
    message = stream.str();
    return false;
  }

  void SCANReplanFSM::abortForExcessiveDetour(const std::string &source)
  {
    const std::string message = local_detour_rejection_message_.empty()
                                    ? "SCAN rejected an excessive local detour before execution"
                                    : local_detour_rejection_message_;
    ROS_WARN("[%s] %s", source.c_str(), message.c_str());
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1.0, 0.1, 0.1, 1.0), 0.3, 0);
    visualization_->displayPlanningStatus(
        end_pt_, "PLAN REJECTED: EXCESSIVE DETOUR",
        Eigen::Vector4d(1.0, 0.1, 0.1, 1.0));
    replan_fail_count_ = 0;
    need_hover_stop_ = true;
    flag_escape_emergency_ = true;
    queueActionTerminal(scan_planner::NavigateToPoseResult::ABORTED_NO_LOCAL_PATH,
                        message);
    changeFSMExecState(EMERGENCY_STOP, source);
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      bool success = callReboundReplan(false, false);
      if (!success)
      {
        if (local_detour_limit_exceeded_)
          return false;
        success = callReboundReplan(true, false);
        if (!success)
        {
          if (local_detour_limit_exceeded_)
            return false;
          success = callReboundReplan(true, true);
          if (!success)
            return false;
        }
      }

      return true;
    }

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      if (local_detour_limit_exceeded_)
        return false;
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.toSec() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (ros::Time::now() - info->start_time_).toSec();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (local_detour_limit_exceeded_)
          {
            abortForExcessiveDetour("SAFETY_DETOUR");
            return;
          }
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    local_detour_limit_exceeded_ = false;
    local_detour_rejection_message_.clear();

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      if (!localTrajectoryDetourAcceptable(local_detour_rejection_message_))
      {
        local_detour_limit_exceeded_ = true;
        return false;
      }

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist_to_start = (pos_t - start_pt_).norm();
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }

    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj);
    local_target_pt_ = end_pt_;

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      total_dist += (pos_t - prev_pos).norm();
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        ROS_WARN_THROTTLE(1.0, "Local target in collision, adjusted to a nearby collision-free point.");
        target_t = adjusted_t;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Local target in collision and no nearby collision-free target was found.");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (max_vel * max_vel) / (2 * max_acc))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
