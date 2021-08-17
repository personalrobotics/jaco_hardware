#include <JacoRobot.h>
#include <iostream>
#include "std_msgs/String.h"
#include <ros/rate.h>
#include <ros/console.h>
#include <sstream>

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("JACO HARDWARE starting");
    ros::init(argc, argv, "jaco_hardware");
    ros::NodeHandle nh("~");

    JacoRobot robot(nh);
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Run Grav Comp Calibration, writes to "ParametersOptimal_Z.txt"
    bool runGravCalib = false;
    nh.getParam("run_grav_calib", runGravCalib);
    if(runGravCalib) {
        nh.setParam("run_grav_calib", false);
        ROS_INFO_STREAM("Running Gravcomp Calibration...");
        auto params = robot.calcGravcompParams();
        if (!params.empty()) {
            std::ostringstream oss;
            std::copy(params.begin(), params.end()-1,
                std::ostream_iterator<float>(oss, ","));
            oss << params.back();
            ROS_INFO_STREAM("Gravcomp Params: " << oss.str());
            return 0;
        }
        return -1;
    }

    // Zero Torque Sensors
    bool zeroTorque = false;
    nh.getParam("zero_torque", zeroTorque);
    if(zeroTorque) {
        nh.setParam("zero_torque", false);
        ROS_INFO_STREAM("Moving Robot To Candlestick [ENTER when ready]...");
        std::cin.get();
        bool ret = robot.zeroTorqueSensors();
        if(!ret) {
            ROS_ERROR("Could not zero torque sensors");
            return -1;
        }
        return 0;
    }

    bool enableGravComp = false;
    nh.getParam("grav_comp", enableGravComp);
    ROS_INFO_STREAM("Enable gravcomp? " << enableGravComp);
    if(enableGravComp) {
      nh.setParam("grav_comp", false);

      // Activate default grav comp parameters
      std::string gravCompFile = "calib/GravComParams_Empty.txt";
      nh.getParam("grav_comp_file", gravCompFile);
      ROS_INFO_STREAM("Enabling GravComp with Params: " << gravCompFile);
      if(robot.useGravcompForEStop(true, gravCompFile)) {
        ROS_INFO_STREAM("Gravcomp Enabled with Params: " << gravCompFile);

        // Whether to enter grav comp on start-up
        bool enterGravComp = false;
        nh.getParam("start_grav_comp", enterGravComp);
        if(enterGravComp) {
            ROS_INFO_STREAM("Entering Grav Comp...");
            nh.setParam("start_grav_comp", false);
            robot.enterGravComp();
        }
      } // end if(Gravcomp successfuly enabled)
    } // end if(enableGravComp)
    nh.setParam("grav_comp_file", "calib/GravComParams_Empty.txt");

      

    ROS_INFO_STREAM("JACO Hardware Setup Complete!");

    // Ros control rate of 100Hz
    ros::Rate controlRate(100.0);
    while (ros::ok())
    {
        robot.read();
        if (robot.eff_stall == true)
        {
            cm.update(robot.get_time(), robot.get_period(), true);
        }
        else
        {
            cm.update(robot.get_time(), robot.get_period());
        }

        robot.write();
        controlRate.sleep();
    }

    // Ensure we go rigid
    robot.setTorqueMode(false);

    return 0;
}
