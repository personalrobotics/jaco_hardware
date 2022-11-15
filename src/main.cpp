#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <JacoRobot.h>
#include <cerrno>
#include <cstdio>
#include <iostream>
#include <ros/console.h>
#include <ros/rate.h>
#include <sstream>

void watchdog(const std_msgs::BoolConstPtr &msg, JacoRobot &robot) {
  if (msg->data)
    robot.feedWatchdog();
  else
    robot.EStop();
}

int main(int argc, char *argv[]) {
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
  if (runGravCalib) {
    nh.setParam("run_grav_calib", false);
    ROS_INFO_STREAM("Running Gravcomp Calibration...");
    auto params = robot.calcGravcompParams();
    if (!params.empty()) {
      std::ostringstream oss;
      std::copy(params.begin(), params.end() - 1,
                std::ostream_iterator<float>(oss, ","));
      oss << params.back();
      ROS_INFO_STREAM("Gravcomp Params: " << oss.str());

      std::string gravCompFile = "ParametersOptimal_Z.txt";
      if (nh.getParam("grav_comp_file", gravCompFile)) {
        std::string path =
            ros::package::getPath("jaco_hardware") + "/" + gravCompFile;
        if (std::rename("ParametersOptimal_Z.txt", path.c_str()) < 0) {
          std::perror("Error moving grav comp file to destionation.");
          gravCompFile = "ParametersOptimal_Z.txt";
        }
      }
      ROS_INFO_STREAM("Params written to " << gravCompFile);
      return 0;
    }
    return -1;
  }

  // Zero Torque Sensors
  bool zeroTorque = false;
  nh.getParam("run_zero_torque", zeroTorque);
  if (zeroTorque) {
    nh.setParam("run_zero_torque", false);
    ROS_INFO_STREAM("Moving Robot To Candlestick [ENTER when ready]...");
    std::cin.get();
    bool ret = robot.zeroTorqueSensors();
    if (!ret) {
      ROS_ERROR("Could not zero torque sensors");
      return -1;
    }
    return 0;
  }

  bool enableGravComp = false;
  nh.getParam("grav_comp", enableGravComp);
  if (enableGravComp) {

    // Activate default grav comp parameters
    std::string gravCompFile = "calib/GravComParams_Empty.txt";
    nh.getParam("grav_comp_file", gravCompFile);
    ROS_INFO_STREAM("Enabling GravComp with Params: " << gravCompFile);
    if (robot.useGravcompForEStop(true, gravCompFile)) {
      ROS_INFO_STREAM("Gravcomp Enabled!");

      // Whether to enter grav comp on start-up
      bool enterGravComp = false;
      nh.getParam("run_grav_comp", enterGravComp);
      if (enterGravComp) {
        ROS_INFO_STREAM("Entering Grav Comp...");
        nh.setParam("run_grav_comp", false);
        robot.enterGravComp();
      }
    } // end if(Gravcomp successfuly enabled)
  }   // end if(enableGravComp)

  // Start Watchdog
  ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(
      "watchdog", 100,
      std::bind(watchdog, std::placeholders::_1, std::ref(robot)));

  // Blocks until watchdog is active
  ROS_INFO_STREAM("Waiting for watchdog...");
  robot.checkWatchdog();

  ROS_INFO_STREAM("JACO Hardware Setup Complete!");

  // Ros control rate: default 100Hz
  double dControlRate = 100.0;
  nh.getParam("control_rate", dControlRate);
  ros::Rate controlRate(dControlRate);
  while (ros::ok()) {
    robot.read();
    if (robot.eff_stall == true) {
      cm.update(robot.get_time(), robot.get_period(), true);
    } else {
      cm.update(robot.get_time(), robot.get_period());
    }

    robot.write();

    robot.checkWatchdog();
    controlRate.sleep();
  }

  // Ensure we go rigid
  robot.setTorqueMode(false);

  return 0;
}
