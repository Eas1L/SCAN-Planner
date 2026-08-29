#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

constexpr double kMaxVYawLimit = 1.0;
constexpr int kRealFollowPath = 1;
constexpr int kRosPlanAction = 2;
constexpr int kRosPubAction = 4;

ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Subscriber real_action_sub;
ros::Subscriber ros_state_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
bool have_real_action = false;
bool motion_enabled = false;
int real_action = 0;
int ros_state = -1;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
ros::Time last_update_time;

double time_forward;
double heading_error_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
std::string body_pose_topic;

bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[closed_loop_controller] missing required private parameter ~" << name);
  return false;
}

bool loadParams(const ros::NodeHandle &nh)
{
  bool ok = true;
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ok &= loadRequiredParam(nh, "time_forward", time_forward);
  ok &= loadRequiredParam(nh, "heading_error_threshold", heading_error_threshold);
  ok &= loadRequiredParam(nh, "kp_pos", kp_pos);
  ok &= loadRequiredParam(nh, "kp_yaw", kp_yaw);
  ok &= loadRequiredParam(nh, "max_vx", max_vx);
  ok &= loadRequiredParam(nh, "max_vy", max_vy);
  ok &= loadRequiredParam(nh, "max_vyaw", max_vyaw);
  ok &= loadRequiredParam(nh, "finish_dist", finish_dist);
  if (ok && max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  return ok;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  const double t_look = std::min(traj_duration, t_cur + time_forward);
  Eigen::Vector3d dir = traj[0].evaluateDeBoorT(t_look) - pos_des;

  if (dir.head<2>().squaredNorm() < 1e-4)
  {
    Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_cur);
    dir = vel;
  }

  if (dir.head<2>().squaredNorm() < 1e-4)
    return odom_yaw;

  return std::atan2(dir(1), dir(0));
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void clearActiveTrajectory()
{
  receive_traj = false;
  traj.clear();
  traj_duration = 0.0;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
}

void updateMotionGate(const char *source)
{
  const bool state_allows_motion =
      ros_state >= kRosPlanAction && ros_state <= kRosPubAction;
  const bool next_enabled =
      have_real_action && real_action == kRealFollowPath && state_allows_motion;

  if (next_enabled == motion_enabled)
    return;

  motion_enabled = next_enabled;
  if (!motion_enabled)
  {
    clearActiveTrajectory();
    publishExecutionFrozen(false);
    publishStop();
    ROS_WARN("[closed_loop_controller] motion disabled by %s "
             "(real_action=%d ros_state=%d); trajectory cleared.",
             source, real_action, ros_state);
  }
  else
  {
    last_update_time = ros::Time::now();
    ROS_WARN("[closed_loop_controller] motion enabled by %s "
             "(real_action=%d ros_state=%d).",
             source, real_action, ros_state);
  }
}

void realActionCallback(const std_msgs::Int32ConstPtr &msg)
{
  if (!msg)
    return;
  real_action = msg->data;
  have_real_action = true;
  updateMotionGate("/ros/real_action");
}

void rosStateCallback(const std_msgs::Int32ConstPtr &msg)
{
  if (!msg)
    return;
  ros_state = msg->data;
  updateMotionGate("/ros/state");
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  receive_traj = true;

  ROS_WARN("[closed_loop_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
}

void cmdCallback(const ros::TimerEvent &)
{
  if (!motion_enabled || !receive_traj || !have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  if (std::abs(yaw_err) > heading_error_threshold)
  {
    publishExecutionFrozen(true);
    publishStop(vyaw_cmd);
    last_update_time = now; // freeze exec_time while rotating in place
    return;
  }

  publishExecutionFrozen(false);
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  Eigen::Vector2d pos_err(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));
  Eigen::Vector2d vel_ff(vel_des(0), vel_des(1));
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, std::max(max_vx, max_vy));

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
    cmd = geometry_msgs::Twist();

  cmd_vel_pub.publish(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "closed_loop_controller");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  if (!loadParams(nh))
    return 1;

  bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  real_action_sub = node.subscribe("/ros/real_action", 10, realActionCallback);
  ros_state_sub = node.subscribe("/ros/state", 10, rosStateCallback);
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller] ready.");

  ros::spin();
  return 0;
}
