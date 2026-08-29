#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

class TerminalRawMode {
public:
  TerminalRawMode()
  {
    fd_ = STDIN_FILENO;
    enabled_ = isatty(fd_) && tcgetattr(fd_, &old_termios_) == 0;
    if (!enabled_) {
      return;
    }

    termios raw = old_termios_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &raw);

    old_flags_ = fcntl(fd_, F_GETFL, 0);
    if (old_flags_ >= 0) {
      fcntl(fd_, F_SETFL, old_flags_ | O_NONBLOCK);
    }
  }

  ~TerminalRawMode()
  {
    if (enabled_) {
      tcsetattr(fd_, TCSANOW, &old_termios_);
      if (old_flags_ >= 0) {
        fcntl(fd_, F_SETFL, old_flags_);
      }
    }
  }

  bool enabled() const
  {
    return enabled_;
  }

private:
  int fd_ = -1;
  int old_flags_ = -1;
  bool enabled_ = false;
  termios old_termios_{};
};

enum class KeyCommand {
  NONE,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  STOP,
  QUIT
};

KeyCommand ReadKey()
{
  char c = 0;
  ssize_t n = read(STDIN_FILENO, &c, 1);
  if (n <= 0) {
    return KeyCommand::NONE;
  }

  if (c == 'q' || c == 'Q') {
    return KeyCommand::QUIT;
  }
  if (c == ' ' || c == 's' || c == 'S') {
    return KeyCommand::STOP;
  }

  if (c != '\033') {
    return KeyCommand::NONE;
  }

  char seq[2] = {0, 0};
  if (read(STDIN_FILENO, &seq[0], 1) <= 0 ||
      read(STDIN_FILENO, &seq[1], 1) <= 0) {
    return KeyCommand::NONE;
  }

  if (seq[0] != '[') {
    return KeyCommand::NONE;
  }

  switch (seq[1]) {
    case 'A':
      return KeyCommand::FORWARD;
    case 'B':
      return KeyCommand::BACKWARD;
    case 'C':
      return KeyCommand::TURN_RIGHT;
    case 'D':
      return KeyCommand::TURN_LEFT;
    default:
      return KeyCommand::NONE;
  }
}

geometry_msgs::Twist MakeTwist(KeyCommand cmd,
                               double linear_speed,
                               double angular_speed)
{
  geometry_msgs::Twist twist;
  switch (cmd) {
    case KeyCommand::FORWARD:
      twist.linear.x = linear_speed;
      break;
    case KeyCommand::BACKWARD:
      twist.linear.x = -linear_speed;
      break;
    case KeyCommand::TURN_LEFT:
      twist.angular.z = angular_speed;
      break;
    case KeyCommand::TURN_RIGHT:
      twist.angular.z = -angular_speed;
      break;
    default:
      break;
  }
  return twist;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_cmdvel");
  ros::NodeHandle nh("~");

  std::string topic;
  double linear_speed = 0.3;
  double angular_speed = 0.8;
  double publish_rate = 20.0;
  double key_timeout = 0.3;

  nh.param<std::string>("topic", topic, "/cmd_vel");
  nh.param<double>("linear_speed", linear_speed, 0.3);
  nh.param<double>("angular_speed", angular_speed, 0.8);
  nh.param<double>("publish_rate", publish_rate, 20.0);
  nh.param<double>("key_timeout", key_timeout, 0.3);

  TerminalRawMode terminal;
  if (!terminal.enabled()) {
    ROS_ERROR("keyboard_cmdvel must run in an interactive terminal.");
    return 1;
  }

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic, 10);
  ros::Rate rate(publish_rate);
  geometry_msgs::Twist cmd;
  ros::Time last_key_time = ros::Time::now();

  ROS_INFO("keyboard_cmdvel started. Arrow keys control /cmd_vel, SPACE/s stops, q quits.");

  while (ros::ok()) {
    KeyCommand key = ReadKey();
    if (key == KeyCommand::QUIT) {
      break;
    }

    if (key == KeyCommand::STOP) {
      cmd = geometry_msgs::Twist();
      last_key_time = ros::Time::now();
    } else if (key != KeyCommand::NONE) {
      cmd = MakeTwist(key, linear_speed, angular_speed);
      last_key_time = ros::Time::now();
    }

    if ((ros::Time::now() - last_key_time).toSec() > key_timeout) {
      cmd = geometry_msgs::Twist();
    }

    pub.publish(cmd);
    ros::spinOnce();
    rate.sleep();
  }

  pub.publish(geometry_msgs::Twist());
  ros::Duration(0.1).sleep();
  return 0;
}
