#include "MicoRobot.h"

using namespace std;


MicoRobot::MicoRobot(ros::NodeHandle nh)
{
    int i;
    cmd_pos.resize(num_full_dof);
    cmd_vel.resize(num_full_dof);
    zero_velocity_command.resize(num_full_dof, 0.0);
    pos.resize(num_full_dof);
    vel.resize(num_full_dof);
    eff.resize(num_full_dof);
    pos_offsets.resize(num_arm_dof);
    soft_limits.resize(num_full_dof);

    // connect and register the joint state interface.
    // this gives joint states (pos, vel, eff) back as an output.
    hardware_interface::JointStateHandle state_handle_base("j2n6a300_joint_1", &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_handle_shoulder("j2n6a300_joint_2", &pos[1], &vel[1], &eff[1]);
    hardware_interface::JointStateHandle state_handle_elbow("j2n6a300_joint_3", &pos[2], &vel[2], &eff[2]);
    hardware_interface::JointStateHandle state_handle_wrist0("j2n6a300_joint_4", &pos[3], &vel[3], &eff[3]);
    hardware_interface::JointStateHandle state_handle_wrist1("j2n6a300_joint_5", &pos[4], &vel[4], &eff[4]);
    hardware_interface::JointStateHandle state_handle_wrist2("j2n6a300_joint_6", &pos[5], &vel[5], &eff[5]);
    hardware_interface::JointStateHandle state_handle_finger0("j2n6a300_joint_finger_1", &pos[6], &vel[6], &eff[6]);
    hardware_interface::JointStateHandle state_handle_finger1("j2n6a300_joint_finger_2", &pos[7], &vel[7], &eff[7]);
    hardware_interface::JointStateHandle state_handle_finger2("j2n6a300_joint_finger_3", &pos[8], &vel[8], &eff[8]);

    jnt_state_interface.registerHandle(state_handle_base);
    jnt_state_interface.registerHandle(state_handle_shoulder);
    jnt_state_interface.registerHandle(state_handle_elbow);
    jnt_state_interface.registerHandle(state_handle_wrist0);
    jnt_state_interface.registerHandle(state_handle_wrist1);
    jnt_state_interface.registerHandle(state_handle_wrist2);
    jnt_state_interface.registerHandle(state_handle_finger0);
    jnt_state_interface.registerHandle(state_handle_finger1);
    jnt_state_interface.registerHandle(state_handle_finger2);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    // this takes joint velocities in as a command.
    hardware_interface::JointHandle vel_handle_base(jnt_state_interface.getHandle("j2n6a300_joint_1"), &cmd_vel[0]);
    hardware_interface::JointHandle vel_handle_shoulder(jnt_state_interface.getHandle("j2n6a300_joint_2"), &cmd_vel[1]);
    hardware_interface::JointHandle vel_handle_elbow(jnt_state_interface.getHandle("j2n6a300_joint_3"), &cmd_vel[2]);
    hardware_interface::JointHandle vel_handle_wrist0(jnt_state_interface.getHandle("j2n6a300_joint_4"), &cmd_vel[3]);
    hardware_interface::JointHandle vel_handle_wrist1(jnt_state_interface.getHandle("j2n6a300_joint_5"), &cmd_vel[4]);
    hardware_interface::JointHandle vel_handle_wrist2(jnt_state_interface.getHandle("j2n6a300_joint_6"), &cmd_vel[5]);
    hardware_interface::JointHandle vel_handle_finger0(jnt_state_interface.getHandle("j2n6a300_joint_finger_1"), &cmd_vel[6]);
    hardware_interface::JointHandle vel_handle_finger1(jnt_state_interface.getHandle("j2n6a300_joint_finger_2"), &cmd_vel[7]);
    hardware_interface::JointHandle vel_handle_finger2(jnt_state_interface.getHandle("j2n6a300_joint_finger_3"), &cmd_vel[8]);

    jnt_vel_interface.registerHandle(vel_handle_base);
    jnt_vel_interface.registerHandle(vel_handle_shoulder);
    jnt_vel_interface.registerHandle(vel_handle_elbow);
    jnt_vel_interface.registerHandle(vel_handle_wrist0);
    jnt_vel_interface.registerHandle(vel_handle_wrist1);
    jnt_vel_interface.registerHandle(vel_handle_wrist2);
    jnt_vel_interface.registerHandle(vel_handle_finger0);
    jnt_vel_interface.registerHandle(vel_handle_finger1);
    jnt_vel_interface.registerHandle(vel_handle_finger2);

    registerInterface(&jnt_vel_interface);

    // connect and register the joint position interface
    // this takes joint positions in as a command.
    hardware_interface::JointHandle pos_handle_base(jnt_state_interface.getHandle("j2n6a300_joint_1"), &cmd_pos[0]);
    hardware_interface::JointHandle pos_handle_shoulder(jnt_state_interface.getHandle("j2n6a300_joint_2"), &cmd_pos[1]);
    hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("j2n6a300_joint_3"), &cmd_pos[2]);
    hardware_interface::JointHandle pos_handle_wrist0(jnt_state_interface.getHandle("j2n6a300_joint_4"), &cmd_pos[3]);
    hardware_interface::JointHandle pos_handle_wrist1(jnt_state_interface.getHandle("j2n6a300_joint_5"), &cmd_pos[4]);
    hardware_interface::JointHandle pos_handle_wrist2(jnt_state_interface.getHandle("j2n6a300_joint_6"), &cmd_pos[5]);
    hardware_interface::JointHandle pos_handle_finger0(jnt_state_interface.getHandle("j2n6a300_joint_finger_1"), &cmd_pos[6]);
    hardware_interface::JointHandle pos_handle_finger1(jnt_state_interface.getHandle("j2n6a300_joint_finger_2"), &cmd_pos[7]);
    hardware_interface::JointHandle pos_handle_finger2(jnt_state_interface.getHandle("j2n6a300_joint_finger_3"), &cmd_pos[8]);

    jnt_pos_interface.registerHandle(pos_handle_base);
    jnt_pos_interface.registerHandle(pos_handle_shoulder);
    jnt_pos_interface.registerHandle(pos_handle_elbow);
    jnt_pos_interface.registerHandle(pos_handle_wrist0);
    jnt_pos_interface.registerHandle(pos_handle_wrist1);
    jnt_pos_interface.registerHandle(pos_handle_wrist2);
    jnt_pos_interface.registerHandle(pos_handle_finger0);
    jnt_pos_interface.registerHandle(pos_handle_finger1);
    jnt_pos_interface.registerHandle(pos_handle_finger2);

    registerInterface(&jnt_pos_interface);

    // connect and register the joint mode interface
    // this is needed to determine if velocity or position control is needed.
    hardware_interface::JointModeHandle mode_handle("joint_mode", &joint_mode);
    jm_interface.registerHandle(mode_handle);

    registerInterface(&jm_interface);


    // Start Up Kinova API
    int r = NO_ERROR_KINOVA;
    
    r = InitAPI();
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not initialize API: Error code %d",r);
    }
    
    r = StartControlAPI();
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not start API Control: Error code %d",r);
    }
    
    r = SetAngularControl();
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not set angular control: Error code %d",r);
    }
    
    r = StartForceControl();
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not start force control: Error code %d",r);
    }
    
    // get soft limits from rosparams
    if (nh.hasParam("soft_limits/eff"))
    {
        nh.getParam("soft_limits/eff", soft_limits);
        ROS_INFO("Set soft_limits for eff to: [%f,%f,%f,%f,%f,%f,%f,%f,%f]", soft_limits[0], soft_limits[1], soft_limits[2], soft_limits[3], soft_limits[4], soft_limits[5], soft_limits[6], soft_limits[7], soft_limits[8]);
    }
    else
    {
        ROS_ERROR("No soft limits set for the MICO!");
        throw std::runtime_error("no soft limits set for the MICO!");
    }

    // set stall
    eff_stall = false;

    // initialize default positions
    initializeOffsets();

    last_mode = hardware_interface::MODE_VELOCITY;
}

MicoRobot::~MicoRobot()
{
    int r = NO_ERROR_KINOVA;
    r = StopControlAPI();
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not stop API Control: Error code %d",r);
    }
    r = CloseAPI();
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not close API Control: Error code %d",r);
    }
}

void MicoRobot::initializeOffsets()
{
    this->read();

    // Next, we wrap the positions so they are within -pi to pi of
    // the hardcoded midpoints, and add that to the offset. TODO(mklingen):
    // figure out if this makes sense.
    for (int i = 0; i < num_arm_dof; i++)
    {
        while (this->pos[i] < hardcoded_pos_midpoints[i] - M_PI)
        {
            this->pos[i] += 2.0 * M_PI;
            this->pos_offsets[i] += 2.0 * M_PI;
        }
        while (this->pos[i] > hardcoded_pos_midpoints[i] + M_PI)
        {
            this->pos[i] -= 2.0 * M_PI;
            this->pos_offsets[i] -= 2.0 * M_PI;
        }
    }
}

ros::Time MicoRobot::get_time(void)
{
    return ros::Time::now();
}

ros::Duration MicoRobot::get_period(void)
{
    // TODO(benwr): What is a reasonable period?
    // Here I've assumed  10ms
    return ros::Duration(0.01);
}

inline double MicoRobot::degreesToRadians(double degrees)
{
    return (M_PI / 180.0) * degrees;
}

inline double MicoRobot::radiansToDegrees(double radians)
{
    return (180.0 / M_PI) * radians;
}

inline double MicoRobot::radiansToFingerTicks(double radians)
{
    return (6400.0/1.4) * radians; //this magic number was found in the kinova-ros code, jaco_driver/src/jaco_arm.cpp
}

inline double MicoRobot::fingerTicksToRadians(double ticks)
{
    return ticks * (1.4/6400.0);  //this magic number was found in the kinova-ros code, jaco_driver/src/jaco_arm.cpp
}

void MicoRobot::sendPositionCommand(const std::vector<double>& command)
{
    // Need to send an "advance trajectory" with a single point and the correct settings
    // Angular position
    
    AngularInfo joint_pos;
    joint_pos.InitStruct();
    
    // The crazy weird position changes are how Kinova does it in their driver node!! ><
    joint_pos.Actuator1 = float(radiansToDegrees(command.at(0)));
    joint_pos.Actuator2 = float(radiansToDegrees(command.at(1)));
    joint_pos.Actuator3 = float(radiansToDegrees(command.at(2)));
    joint_pos.Actuator4 = float(radiansToDegrees(command.at(3)));
    joint_pos.Actuator5 = float(radiansToDegrees(command.at(4)));
    joint_pos.Actuator6 = float(radiansToDegrees(command.at(5)) );
    
    TrajectoryPoint trajectory;
    trajectory.InitStruct(); // initialize structure
    memset(&trajectory, 0, sizeof(trajectory));  // zero out the structure
    trajectory.Position.Type = ANGULAR_POSITION; // set to angular position 
    trajectory.Position.Actuators = joint_pos; // position is passed in the position struct
    
    //trajectory.Position.FingersPosition

    int r = NO_ERROR_KINOVA;
    r = SendAdvanceTrajectory(trajectory);
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not send : Error code %d",r);
    }

}

void MicoRobot::sendVelocityCommand(const std::vector<double>& command)
{
    // Need to send an "advance trajectory" with a single point and the correct settings
    // Angular velocity
    
    AngularInfo joint_vel;
    joint_vel.InitStruct();
    // Don't ask me why all the joint velocities are negative, I have no idea.
    joint_vel.Actuator1 = float(radiansToDegrees(command.at(0)));
    joint_vel.Actuator2 = float(radiansToDegrees(command.at(1)));
    joint_vel.Actuator3 = float(radiansToDegrees(command.at(2)));
    joint_vel.Actuator4 = float(radiansToDegrees(command.at(3)));
    joint_vel.Actuator5 = float(radiansToDegrees(command.at(4)));
    joint_vel.Actuator6 = float(radiansToDegrees(command.at(5)));
    
    TrajectoryPoint trajectory;
    trajectory.InitStruct(); // initialize structure
    memset(&trajectory, 0, sizeof(trajectory));  // zero out the structure
    trajectory.Position.Type = ANGULAR_VELOCITY; // set to angular velocity 
    trajectory.Position.Actuators = joint_vel; // confusingly, velocity is passed in the position struct

    trajectory.Position.HandMode = VELOCITY_MODE;
    trajectory.Position.Type = ANGULAR_VELOCITY;
    trajectory.Position.Fingers.Finger1 = float(radiansToDegrees(command.at(6)));
    trajectory.Position.Fingers.Finger2 = float(radiansToDegrees(command.at(7)));
    trajectory.Position.Fingers.Finger3 = float(radiansToDegrees(command.at(8)));
    //trajectory.Position.Delay = 0.0;

    
    int r = NO_ERROR_KINOVA;
    r = SendAdvanceTrajectory(trajectory);
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not send : Error code %d",r);
    }
}

void MicoRobot::sendTorqueCommand(const std::vector<double>& command)
{
    // TODO
    //SendAngularTorqueCommand()
}

void MicoRobot::write(void)
{
    if (last_mode != joint_mode)
    {
        EraseAllTrajectories();
    }

    if (eff_stall)
        return;

    // have to check the mode type and then choose what commands to send
    switch (joint_mode)
    {
        // send joint position commands
        case hardware_interface::MODE_POSITION:
        {
            sendPositionCommand(cmd_pos);
            break;
        }

        // send joint velocity commands.
        // To send joint velocities, we have to send it a trajectory point in angular mode.
        case hardware_interface::MODE_VELOCITY:
        {
            sendVelocityCommand(cmd_vel);
            break;
        }

        case hardware_interface::MODE_EFFORT:
        {
            ROS_WARN_THROTTLE(1.0, "Mico hardware does not support effort control.");
            break;
        }
    }

    last_mode = joint_mode;
}

void MicoRobot::checkForStall(void)
{
    // check soft limits. If outside of limits, set to force control mode
    // this way the arm can move easily. (if we sent it zero commands, it
    // would still be hitting whatever it was.

    bool all_in_limits = true;
    for (int i = 0; i < num_full_dof; i++)
    {
        if (eff[i] < -soft_limits[i] || eff[i] > soft_limits[i])
        {
            all_in_limits = false;
            ROS_WARN("Exceeded soft effort limits on joint %d. Limit=%f, Measured=%f", i, soft_limits[i], eff[i]);
            /*
            if (!eff_stall)
            {
                ROS_INFO("Erasing all trajectory points")
                EraseAllTrajectories();
                ROS_INFO("Sending zero velocities");
                sendVelocityCommand(zero_velocity_command);
            }*/
        }
    }

    if (all_in_limits && eff_stall)
    {
        eff_stall = false;
        ROS_INFO("Exiting force_control mode.");
        //arm->stop_force_ctrl();
        //arm->set_control_ang();
        sendVelocityCommand(zero_velocity_command);
    }
}

void MicoRobot::read(void)
{
    // make sure that pos, vel, and eff are up to date.
    // TODO: If there is too much lag between calling read()
    // and getting the actual values back, we'll need to be
    // reading values constantly and storing them locally, so
    // at least there is a recent value available for the controller.
    
    AngularPosition arm_pos;
    AngularPosition arm_vel;
    ForcesInfo arm_torq;
    
    // Requires 3 seperate calls to the USB
    GetAngularPosition(arm_pos);
    GetAngularVelocity(arm_vel);
    GetForcesInfo(arm_torq);
    
    // The crazy weird position changes are how Kinova does it in their driver node!! ><
    pos[0] = degreesToRadians(double(arm_pos.Actuators.Actuator1));
    pos[1] = degreesToRadians(double(arm_pos.Actuators.Actuator2));    //kinova-ros/jaco_driver/src/jaco_arm used 260 instead of 270, so switched
    pos[2] = degreesToRadians(double(arm_pos.Actuators.Actuator3));
    pos[3] = degreesToRadians(double(arm_pos.Actuators.Actuator4));
    pos[4] = degreesToRadians(double(arm_pos.Actuators.Actuator5)); 
    pos[5] = degreesToRadians(double(arm_pos.Actuators.Actuator6));
    pos[6] = degreesToRadians(double(arm_pos.Fingers.Finger1));
    pos[7] = degreesToRadians(double(arm_pos.Fingers.Finger2));
    pos[8] = degreesToRadians(double(arm_pos.Fingers.Finger3));
    
    vel[0] = degreesToRadians(double(arm_vel.Actuators.Actuator1));
    vel[1] = degreesToRadians(double(arm_vel.Actuators.Actuator2));
    vel[2] = degreesToRadians(double(arm_vel.Actuators.Actuator3));
    vel[3] = degreesToRadians(double(arm_vel.Actuators.Actuator4));
    vel[4] = degreesToRadians(double(arm_vel.Actuators.Actuator5));
    vel[5] = degreesToRadians(double(arm_vel.Actuators.Actuator6));
    vel[6] = degreesToRadians(double(arm_vel.Fingers.Finger1)); //note: these are set to zero in the kinova ros code
    vel[7] = degreesToRadians(double(arm_vel.Fingers.Finger2));
    vel[8] = degreesToRadians(double(arm_vel.Fingers.Finger3));

    eff[0] = degreesToRadians(double(arm_torq.Actuator1));
    eff[1] = degreesToRadians(double(arm_torq.Actuator2));
    eff[2] = degreesToRadians(double(arm_torq.Actuator3));
    eff[3] = degreesToRadians(double(arm_torq.Actuator4));
    eff[4] = degreesToRadians(double(arm_torq.Actuator5));
    eff[5] = degreesToRadians(double(arm_torq.Actuator6));
    eff[6] = 0;
    eff[7] = 0;
    eff[8] = 0;
    
    checkForStall();
    
}


