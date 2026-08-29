/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowState.h>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include <geometry_msgs/Twist.h>

#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWSTATE "rt/lowstate"

using namespace unitree::common;

namespace unitree_go2_controller
{

    class UnitreeGo2Controller
    {
    public:
        UnitreeGo2Controller(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
                : nh_(nh)
                , nh_private_(nh_private)
                , px0(0.0)
                , py0(0.0)
                , yaw0(0.0)
                , control_timer_count_(0.0)
                , flag_(0)
                , control_frequency_(200.0)  // 200Hz control frequency
        {
            // Initialize parameters
            nh_private_.param("control_frequency", control_frequency_, 200.0);
            dt_ = 1.0 / control_frequency_;

            // Initialize robot interface
            sport_client_.SetTimeout(10.0f);
            sport_client_.Init();

            // Initialize publishers
            high_state_pub_ = nh_.advertise<unitree_legged_msgs::HighState>("/go2/high_state", 10);

            // Initialize services (you can add services for different commands)
            // Example: stand_up_srv_ = nh_.advertiseService("stand_up", &UnitreeGo2Controller::standUpCallback, this);

            // Initialize subscribers for robot control commands
            cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &UnitreeGo2Controller::cmdVelCallback, this);

            // Initialize Unitree DDS subscribers
            highstate_subscriber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
            highstate_subscriber_->InitChannel(std::bind(&UnitreeGo2Controller::highStateHandler, this, std::placeholders::_1), 1);

            lowstate_subscriber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
            lowstate_subscriber_->InitChannel(std::bind(&UnitreeGo2Controller::lowStateHandler, this, std::placeholders::_1), 1);

            // Start control timer
            control_timer_ = nh_.createTimer(ros::Duration(dt_), &UnitreeGo2Controller::controlTimerCallback, this);

            ROS_INFO("Unitree Go2 Controller initialized");
        }

        ~UnitreeGo2Controller()
        {
            // Cleanup
            sport_client_.StopMove();
        }

    private:
        // DDS message handlers
        void lowStateHandler(const void* message)
        {
            low_state_ = *(unitree_go::msg::dds_::LowState_*)message;

            // Convert to ROS message and publish
            unitree_legged_msgs::HighState high_state_msg;
            high_state_msg.stamp = ros::Time::now();

//            std::cout<<"low_state_.imu_state().quaternion()[0]; "<<low_state_.imu_state().quaternion()[0]<<std::endl;
            // Fill IMU data
            high_state_msg.imu.quaternion[0] = low_state_.imu_state().quaternion()[0];
            high_state_msg.imu.quaternion[1] = low_state_.imu_state().quaternion()[1];
            high_state_msg.imu.quaternion[2] = low_state_.imu_state().quaternion()[2];
            high_state_msg.imu.quaternion[3] = low_state_.imu_state().quaternion()[3];

            high_state_msg.imu.gyroscope[0] = low_state_.imu_state().gyroscope()[0];
            high_state_msg.imu.gyroscope[1] = low_state_.imu_state().gyroscope()[1];
            high_state_msg.imu.gyroscope[2] = low_state_.imu_state().gyroscope()[2];

            high_state_msg.imu.accelerometer[0] = low_state_.imu_state().accelerometer()[0];
            high_state_msg.imu.accelerometer[1] = low_state_.imu_state().accelerometer()[1];
            high_state_msg.imu.accelerometer[2] = low_state_.imu_state().accelerometer()[2];

            high_state_msg.imu.rpy[0] = low_state_.imu_state().rpy()[0];
            high_state_msg.imu.rpy[1] = low_state_.imu_state().rpy()[1];
            high_state_msg.imu.rpy[2] = low_state_.imu_state().rpy()[2];

//            // Fill motor states
//            low_state_msg.motor_state.resize(low_state_.motor_state().size());
            for (size_t i = 0; i < low_state_.motor_state().size(); ++i)
            {
                high_state_msg.motorState[i].q = low_state_.motor_state()[i].q();
                high_state_msg.motorState[i].dq = low_state_.motor_state()[i].dq();
                high_state_msg.motorState[i].tauEst = low_state_.motor_state()[i].tau_est();
                high_state_msg.motorState[i].mode = low_state_.motor_state()[i].mode();
            }
//
//            // Fill foot force
            for (size_t i = 0; i < low_state_.foot_force().size(); ++i)
            {
                high_state_msg.footForce[i] = low_state_.foot_force()[i];
            }

            high_state_pub_.publish(high_state_msg);
        }

        void highStateHandler(const void* message)
        {
            return;
        }

        // Control timer callback
        void controlTimerCallback(const ros::TimerEvent& event)
        {
            control_timer_count_ += dt_;

            // Execute control logic
            executeControl();

            // Log at lower frequency
            static int log_counter = 0;
            if (log_counter++ >= 50)  // ~10Hz logging
            {
                log_counter = 0;
                ROS_INFO_THROTTLE(1.0, "Control running at %.0fHz, time: %.1fs",
                                  control_frequency_, control_timer_count_);
            }
        }

        // Command velocity callback
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            // Convert ROS twist to Unitree movement command
            command_vel_x_ = msg->linear.x;
            command_vel_y_ = msg->linear.y;
            command_vel_z_ = msg->angular.z;

            ROS_DEBUG("Received velocity command: vx=%.2f, vy=%.2f, wz=%.2f",
                      command_vel_x_, command_vel_y_, command_vel_z_);
        }

        // Main control execution
        void executeControl()
        {
            // This is where you implement your control logic
            // Example: basic idle stand
            //sport_client_.Move(command_vel_x_, command_vel_y_, command_vel_z_);
            sport_client_.StandUp();

        }

    private:
        // ROS
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher high_state_pub_;
        ros::Publisher low_state_pub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Timer control_timer_;

        // ROS Services (example - uncomment and implement as needed)
        // ros::ServiceServer stand_up_srv_;
        // ros::ServiceServer sit_down_srv_;
        // ros::ServiceServer stop_srv_;

        // Unitree
        unitree::robot::go2::SportClient sport_client_;
        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> highstate_subscriber_;
        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
        unitree_go::msg::dds_::SportModeState_ high_state_;
        unitree_go::msg::dds_::LowState_ low_state_;

        // Control parameters
        double px0, py0, yaw0;
        double control_timer_count_;
        double dt_;
        int flag_;
        double control_frequency_;
        double command_vel_x_;
        double command_vel_y_;
        double command_vel_z_;
    };

} // namespace unitree_go2_controller

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " networkInterface [ROS parameters...]" << std::endl;
        std::cerr << "Example: " << argv[0] << " eth0 __ns:=/go2 _control_frequency:=200" << std::endl;
        return -1;
    }

    // Initialize Unitree channel
    std::string network_interface = argv[1];
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    // Initialize ROS
    ros::init(argc, argv, "unitree_go2_controller");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create controller
    unitree_go2_controller::UnitreeGo2Controller controller(nh, nh_private);

    // Give time for initialization
    ros::Duration(1.0).sleep();

    ROS_INFO("Unitree Go2 Controller node started");

    // Spin
    ros::spin();

    return 0;
}
