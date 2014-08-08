#include <MicoRobot.h>
#include <iostream>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mico_hardware");
  ros::NodeHandle nh;

  MicoRobot robot(nh);
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
     robot.read();
     cm.update(robot.get_time(), robot.get_period());
     robot.write();
     sleep(0.01);
  }

  return 0;
}
