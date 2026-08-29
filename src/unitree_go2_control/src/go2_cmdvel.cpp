#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <unitree/robot/client/client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include <atomic>
#include <functional>
#include <string>

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

struct CmdState {
  geometry_msgs::Twist last_cmd;
  ros::Time last_cmd_time;
  bool have_cmd = false;
  std::atomic<bool> is_wireless_controller{false};
  bool last_up_pressed = false;
};

void CmdVelCb(const geometry_msgs::Twist::ConstPtr& msg, CmdState* state)
{
  state->last_cmd = *msg;
  state->last_cmd_time = ros::Time::now();
  state->have_cmd = true;

  ROS_INFO("recv cmd_vel: x=%.3f y=%.3f yaw=%.3f",
           msg->linear.x, msg->linear.y, msg->angular.z);
}

void JoystickHandler(const void* message, CmdState* state)
{
  unitree_go::msg::dds_::WirelessController_ joystick =
      *(unitree_go::msg::dds_::WirelessController_*)message;
  xKeySwitchUnion key;
  key.value = joystick.keys();

  bool up_pressed = (int)key.components.up == 1;
  if (up_pressed && !state->last_up_pressed) {
    bool next = !state->is_wireless_controller.load();
    state->is_wireless_controller.store(next);
    ROS_INFO("Wireless controller mode: %s", next ? "true" : "false");
  }
  state->last_up_pressed = up_pressed;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go2_cmdvel_bridge");
  ros::NodeHandle nh("~");
  ROS_INFO("go2_cmdvel_bridge node started!"); 

  std::string iface;
  nh.param<std::string>("network_interface", iface, "enp4s0");
  bool use_wireless_gate = false;
  nh.param<bool>("use_wireless_gate", use_wireless_gate, false);
  bool standup_on_start = true;
  nh.param<bool>("standup_on_start", standup_on_start, true);
  double cmd_timeout = 0.5;
  nh.param<double>("cmd_timeout", cmd_timeout, 0.5);

  unitree::robot::ChannelFactory::Instance()->Init(0, iface);

  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);
  sport_client.Init();
  sport_client.WaitLeaseApplied();

  CmdState state;
  state.is_wireless_controller.store(use_wireless_gate);

  if (standup_on_start) {
    int ret = sport_client.StandUp();
    ROS_INFO("StandUp ret=%d", ret);
    ros::Duration(1.0).sleep();
    ret = sport_client.BalanceStand();
    ROS_INFO("BalanceStand ret=%d", ret);
    ros::Duration(0.5).sleep();
  }

  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_suber;
  joystick_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
  joystick_suber->InitChannel(std::bind(&JoystickHandler, std::placeholders::_1, &state), 1);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 10, boost::bind(&CmdVelCb, _1, &state));

  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();

    if (state.is_wireless_controller.load()) {
    ROS_INFO_THROTTLE(1.0, "Controller is not loaded");
      rate.sleep();
      continue;
    }

    bool timed_out = false;
    if (state.have_cmd) {
      double dt = (ros::Time::now() - state.last_cmd_time).toSec();
      timed_out = (dt > cmd_timeout);
    }

    int ret = 0;
    if (!state.have_cmd || timed_out) {
      ret = sport_client.Move(0.0f, 0.0f, 0.0f);
    } else {
      ret = sport_client.Move(state.last_cmd.linear.x,
                              state.last_cmd.linear.y,
                              state.last_cmd.angular.z);
    }

    ROS_INFO_THROTTLE(1.0, "Move ret=%d (%s)", ret, (ret == 0 ? "OK" : "FAIL"));

    rate.sleep();
  }

  sport_client.Move(0.0f, 0.0f, 0.0f);
  return 0;
}
