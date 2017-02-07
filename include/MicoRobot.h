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

using namespace std; // TODO: remove this and propogate change

// TODO: the static consts should be in all caps
static const double hardcoded_pos_midpoints[6] = { 0.0, -0.5 * M_PI, 0.5 * M_PI, 0.0, 0.0, 0.0 };
static const int num_full_dof = 8;
static const int num_arm_dof = 6;

// TODO: add explicit override keyword when appropriate (probably write and read)
// TODO: change get_time to getTime and get_period to getPeriod, eff_stall to effStall 
//      (i.e., make everything camel case except ROS package commands write and read)
// TODO: decide if private variables should be camel case
// TODO: get rid of "void" arguments (it's a holdover from C)
// TODO: change write and read arguments to match override
class MicoRobot: public hardware_interface::RobotHW  
{
    public:
        MicoRobot(ros::NodeHandle nh);

        virtual ~MicoRobot();
        
        void initializeOffsets();
        
        ros::Time get_time(void);

        ros::Duration get_period(void);

        inline double degreesToRadians(double degrees);
        inline double radiansToDegrees(double radians);
        inline double radiansToFingerTicks(double radians);
        inline double fingerTicksToRadians(double ticks);

        void sendPositionCommand(const std::vector<double>& command);
        void sendVelocityCommand(const std::vector<double>& command);
        void sendTorqueCommand(const std::vector<double>& command);

        void write(void);
        void read(void);

        void checkForStall(void);

        bool eff_stall;

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::JointModeInterface jm_interface;

        //JacoArm *arm;
        vector<double> cmd_pos;
        vector<double>  cmd_vel;
        vector<double>  pos;
        vector<double>  vel;
        vector<double>  eff;
        vector<double>  pos_offsets;
        vector<double>  soft_limits;
        vector<double> zero_velocity_command;
        int joint_mode; // this tells whether we're in position or velocity control mode
        int last_mode;
};

#endif

