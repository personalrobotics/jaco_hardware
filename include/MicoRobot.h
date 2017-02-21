#ifndef MICO_ROBOT_H
#define MICO_ROBOT_H

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pr_ros_controllers/joint_mode_interface.h>
//#include <hardware_interface/controller_info_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

// ros
#include <ros/ros.h>
#include <ros/console.h>

// kinova api
#include <kinova/KinovaTypes.h>
#include <kinova/Kinova.API.UsbCommandLayerUbuntu.h>

// c++
#include <stdexcept>
#include <limits>
#include <iostream>

static const double HARDCODED_POS_MIDPOINTS[6] = { 0.0, -0.5 * M_PI, 0.5 * M_PI, 0.0, 0.0, 0.0 };
static const int NUM_FULL_DOF = 8;
static const int NUM_ARM_DOF = 6;

class MicoRobot final: public hardware_interface::RobotHW  
{
    public:
        MicoRobot(ros::NodeHandle nh);

        virtual ~MicoRobot();
        
        void initializeOffsets();
        
        ros::Time getTime();

        ros::Duration getPeriod();

        inline double degreesToRadians(double degrees);
        inline double radiansToDegrees(double radians);
        inline double radiansToFingerTicks(double radians);
        inline double fingerTicksToRadians(double ticks);

        void sendPositionCommand(const std::vector<double>& command);
        void sendVelocityCommand(const std::vector<double>& command);
        void sendTorqueCommand(const std::vector<double>& command);

        void write(const ros::Time&, const ros::Duration&);
        void read();

        void checkForStall();

        bool eff_stall;

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::JointModeInterface jm_interface;

        //JacoArm *arm;
        std::vector<double> cmd_pos;
        std::vector<double>  cmd_vel;
        std::vector<double>  pos;
        std::vector<double>  vel;
        std::vector<double>  eff;
        std::vector<double>  pos_offsets;
        std::vector<double>  soft_limits;
        std::vector<double> zero_velocity_command;
        int joint_mode; // this tells whether we're in position or velocity control mode
        int last_mode;
};

#endif

