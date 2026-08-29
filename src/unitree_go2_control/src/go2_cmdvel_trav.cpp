#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include <unitree/robot/client/client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include <atomic>
#include <cmath>
#include <functional>

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
  int real_action = 0;
  bool have_real_action = false;
  int last_real_action = -1;
  ros::Time real_action_start_time;
  std::atomic<bool> is_wireless_controller{true};
  bool last_up_pressed = false;
};

enum REAL_ACTION {
  REAL_STOP,
  REAL_FOLLOW_PATH,
  REAL_TURN,
  REAL_TURN_DOWN,
  REAL_TURN_UP
};

void CmdVelCb(const geometry_msgs::Twist::ConstPtr& msg, CmdState* state)
{
  state->last_cmd = *msg;
  state->last_cmd_time = ros::Time::now();
  state->have_cmd = true;

  ROS_INFO("recv cmd_vel: x=%.3f y=%.3f yaw=%.3f",
           msg->linear.x, msg->linear.y, msg->angular.z);
}

void RealActionCb(const std_msgs::Int32::ConstPtr& msg,
                  CmdState* state,
                  ros::Publisher* head_rotate_pub)
{
  state->real_action = msg->data;
  state->have_real_action = true;

  if (msg->data == REAL_TURN_DOWN) {
    std_msgs::Int32 rotate_msg;
    rotate_msg.data = 90;
    head_rotate_pub->publish(rotate_msg);
    ROS_INFO("publish /head/rotate: %d", rotate_msg.data);
  } else if (msg->data == REAL_TURN_UP) {
    std_msgs::Int32 rotate_msg;
    rotate_msg.data = 135;
    head_rotate_pub->publish(rotate_msg);
    ROS_INFO("publish /head/rotate: %d", rotate_msg.data);
  }

  ROS_INFO("recv real_action: %d", msg->data);
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

  std::string iface;
  nh.param<std::string>("network_interface", iface, "enp4s0");
  bool start_locked = true;
  nh.param("start_locked", start_locked, true);
  double max_linear_speed = 0.35;
  nh.param("max_linear_speed", max_linear_speed, 0.35);
  if (!std::isfinite(max_linear_speed) || max_linear_speed <= 0.0) {
    ROS_ERROR("Invalid max_linear_speed %.3f; expected a positive finite value.",
              max_linear_speed);
    return 1;
  }
  ROS_INFO("Max linear speed: %.3f m/s", max_linear_speed);

  unitree::robot::ChannelFactory::Instance()->Init(0, iface);

  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);
  sport_client.Init();
  sport_client.WaitLeaseApplied();

  CmdState state;
  state.is_wireless_controller.store(start_locked);
  ROS_INFO("Wireless controller mode: %s",
           start_locked ? "true" : "false");
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_suber;
  joystick_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
  joystick_suber->InitChannel(std::bind(&JoystickHandler, std::placeholders::_1, &state), 1);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 10, boost::bind(&CmdVelCb, _1, &state));
  ros::Publisher head_rotate_pub = nh.advertise<std_msgs::Int32>("/head/rotate", 10);
  ros::Subscriber real_action_sub = nh.subscribe<std_msgs::Int32>(
      "/ros/real_action", 10, boost::bind(&RealActionCb, _1, &state, &head_rotate_pub));

  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();

    if (state.is_wireless_controller.load()) {
      rate.sleep();
      continue;
    }

    bool timed_out = false;
    if (state.have_cmd) {
      double dt = (ros::Time::now() - state.last_cmd_time).toSec();
      timed_out = (dt > 10.0);
    }

    if (state.real_action != state.last_real_action) {
      state.real_action_start_time = ros::Time::now();
      state.last_real_action = state.real_action;
    }

    float send_x = 0.0f;
    float send_y = 0.0f;
    float send_yaw = 0.0f;
    if (!state.have_real_action) {
    } else if (state.real_action == REAL_FOLLOW_PATH) {
      if (state.have_cmd && !timed_out) {
        send_x = static_cast<float>(state.last_cmd.linear.x);
        send_y = static_cast<float>(state.last_cmd.linear.y);
        send_yaw = static_cast<float>(state.last_cmd.angular.z);
      }
    } else if (state.real_action == REAL_TURN) {
      double elapsed = (ros::Time::now() - state.real_action_start_time).toSec();
      double phase = std::fmod(elapsed, 20.0);
      if (phase < 6.0) {
        send_yaw = 1.5f;
      } else {
        send_x = -0.3f;
      }
    }

    const float linear_speed = std::hypot(send_x, send_y);
    if (linear_speed > max_linear_speed) {
      const float scale = static_cast<float>(max_linear_speed) / linear_speed;
      send_x *= scale;
      send_y *= scale;
      ROS_WARN_THROTTLE(1.0,
          "clamp linear speed to %.3f m/s", max_linear_speed);
    }

    const int ret = sport_client.Move(send_x, send_y, send_yaw);
    ROS_INFO_THROTTLE(1.0,
        "send Move: action=%d x=%.3f y=%.3f yaw=%.3f ret=%d (%s)",
        state.have_real_action ? state.real_action : -1,
        send_x, send_y, send_yaw, ret, (ret == 0 ? "OK" : "FAIL"));

    rate.sleep();
  }

  sport_client.Move(0.0f, 0.0f, 0.0f);
  return 0;
}
