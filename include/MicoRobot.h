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

// libkindrv
#include <libkindrv/types.h>
#include <libkindrv/kindrv.h>

// c++
#include <stdexcept>
#include <limits>
#include <iostream>

using namespace std;
using namespace KinDrv;

static const double hardcoded_pos_offsets[6] = { M_PI, 0.5 * M_PI, -0.5 * M_PI, M_PI, M_PI, 0.0 };
static const double hardcoded_pos_midpoints[6] = { 0.0, -0.5 * M_PI, 0.5 * M_PI, 0.0, 0.0, 0.0 };
static const int num_full_dof = 8;
static const int num_arm_dof = 6;


class MicoRobot: public hardware_interface::RobotHW
{
    public:
        MicoRobot(ros::NodeHandle nh)
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
            hardware_interface::JointStateHandle state_handle_base("j1", &pos[0], &vel[0], &eff[0]);
            hardware_interface::JointStateHandle state_handle_shoulder("j2", &pos[1], &vel[1], &eff[1]);
            hardware_interface::JointStateHandle state_handle_elbow("j3", &pos[2], &vel[2], &eff[2]);
            hardware_interface::JointStateHandle state_handle_wrist0("j4", &pos[3], &vel[3], &eff[3]);
            hardware_interface::JointStateHandle state_handle_wrist1("j5", &pos[4], &vel[4], &eff[4]);
            hardware_interface::JointStateHandle state_handle_wrist2("j6", &pos[5], &vel[5], &eff[5]);
            hardware_interface::JointStateHandle state_handle_finger0("f1", &pos[6], &vel[6], &eff[6]);
            hardware_interface::JointStateHandle state_handle_finger1("f2", &pos[7], &vel[7], &eff[7]);

            jnt_state_interface.registerHandle(state_handle_base);
            jnt_state_interface.registerHandle(state_handle_shoulder);
            jnt_state_interface.registerHandle(state_handle_elbow);
            jnt_state_interface.registerHandle(state_handle_wrist0);
            jnt_state_interface.registerHandle(state_handle_wrist1);
            jnt_state_interface.registerHandle(state_handle_wrist2);
            jnt_state_interface.registerHandle(state_handle_finger0);
            jnt_state_interface.registerHandle(state_handle_finger1);

            registerInterface(&jnt_state_interface);

            // connect and register the joint position interface
            // this takes joint velocities in as a command.
            hardware_interface::JointHandle vel_handle_base(jnt_state_interface.getHandle("j1"), &cmd_vel[0]);
            hardware_interface::JointHandle vel_handle_shoulder(jnt_state_interface.getHandle("j2"), &cmd_vel[1]);
            hardware_interface::JointHandle vel_handle_elbow(jnt_state_interface.getHandle("j3"), &cmd_vel[2]);
            hardware_interface::JointHandle vel_handle_wrist0(jnt_state_interface.getHandle("j4"), &cmd_vel[3]);
            hardware_interface::JointHandle vel_handle_wrist1(jnt_state_interface.getHandle("j5"), &cmd_vel[4]);
            hardware_interface::JointHandle vel_handle_wrist2(jnt_state_interface.getHandle("j6"), &cmd_vel[5]);
            hardware_interface::JointHandle vel_handle_finger0(jnt_state_interface.getHandle("f1"), &cmd_vel[6]);
            hardware_interface::JointHandle vel_handle_finger1(jnt_state_interface.getHandle("f2"), &cmd_vel[7]);

            jnt_vel_interface.registerHandle(vel_handle_base);
            jnt_vel_interface.registerHandle(vel_handle_shoulder);
            jnt_vel_interface.registerHandle(vel_handle_elbow);
            jnt_vel_interface.registerHandle(vel_handle_wrist0);
            jnt_vel_interface.registerHandle(vel_handle_wrist1);
            jnt_vel_interface.registerHandle(vel_handle_wrist2);
            jnt_vel_interface.registerHandle(vel_handle_finger0);
            jnt_vel_interface.registerHandle(vel_handle_finger1);

            registerInterface(&jnt_vel_interface);

            // connect and register the joint position interface
            // this takes joint positions in as a command.
            hardware_interface::JointHandle pos_handle_base(jnt_state_interface.getHandle("j1"), &cmd_pos[0]);
            hardware_interface::JointHandle pos_handle_shoulder(jnt_state_interface.getHandle("j2"), &cmd_pos[1]);
            hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("j3"), &cmd_pos[2]);
            hardware_interface::JointHandle pos_handle_wrist0(jnt_state_interface.getHandle("j4"), &cmd_pos[3]);
            hardware_interface::JointHandle pos_handle_wrist1(jnt_state_interface.getHandle("j5"), &cmd_pos[4]);
            hardware_interface::JointHandle pos_handle_wrist2(jnt_state_interface.getHandle("j6"), &cmd_pos[5]);
            hardware_interface::JointHandle pos_handle_finger0(jnt_state_interface.getHandle("f1"), &cmd_pos[6]);
            hardware_interface::JointHandle pos_handle_finger1(jnt_state_interface.getHandle("f2"), &cmd_pos[7]);

            jnt_pos_interface.registerHandle(pos_handle_base);
            jnt_pos_interface.registerHandle(pos_handle_shoulder);
            jnt_pos_interface.registerHandle(pos_handle_elbow);
            jnt_pos_interface.registerHandle(pos_handle_wrist0);
            jnt_pos_interface.registerHandle(pos_handle_wrist1);
            jnt_pos_interface.registerHandle(pos_handle_wrist2);
            jnt_pos_interface.registerHandle(pos_handle_finger0);
            jnt_pos_interface.registerHandle(pos_handle_finger1);

            registerInterface(&jnt_pos_interface);

            // connect and register the joint mode interface
            // this is needed to determine if velocity or position control is needed.
            hardware_interface::JointModeHandle mode_handle("joint_mode", &joint_mode);
            jm_interface.registerHandle(mode_handle);

            registerInterface(&jm_interface);

            // set up libkindrv
            arm = new JacoArm();
            arm->start_api_ctrl();
            arm->set_control_ang();

            // get soft limits from rosparams
            if (nh.hasParam("soft_limits/eff"))
            {
                nh.getParam("soft_limits/eff", soft_limits);
                ROS_INFO("Set soft_limits for eff to: [%f,%f,%f,%f,%f,%f,%f,%f]", soft_limits[0], soft_limits[1], soft_limits[2], soft_limits[3], soft_limits[4], soft_limits[5], soft_limits[6], soft_limits[7]);
            }
            else
            {
                ROS_ERROR("No soft limits set for the MICO!");
                throw std::runtime_error("no soft limits set for the MICO!");
            }

            // set stall
            eff_stall = false;
            arm->stop_force_ctrl();

            /* initialize default positions */
            initializeOffsets();

            last_mode = hardware_interface::MODE_VELOCITY;
        }

        void initializeOffsets()
        {
            // Initially, the offsets are the hard coded positions
            for (i = 0; i < num_arm_dof; i++)
                this->pos_offsets[i] = hardcoded_pos_offsets[i];

            this->read();

            // Next, we wrap the positions so they are within -pi to pi of
            // the hardcoded midpoints, and add that to the offset. TODO(mklingen):
            // figure out if this makes sense.
            for (i = 0; i < num_arm_dof; i++)
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

        ros::Time get_time(void)
        {
            return ros::Time::now();
        }

        ros::Duration get_period(void)
        {
            // TODO(benwr): What is a reasonable period?
            // Here I've assumed  10ms
            return ros::Duration(0.01);
        }

        inline double degreesToRadians(double degrees)
        {
            return (180.0 / M_PI) * degrees;
        }

        inline double radiansToDegrees(double radians)
        {
            return (M_PI / 180.0) * radians;
        }

        inline double radiansToFingerTicks(double radians)
        {
            return 5400.0 * radians;
        }

        inline double fingerTicksToRadians(double ticks)
        {
            return ticks / 5400.0;
        }

        void sendPositionCommand(const std::vector<double>& command)
        {
            arm->set_target_ang(radiansToDegrees(command.at(0) - pos_offsets[0]),
                                radiansToDegrees(command.at(1) - pos_offsets[1]),
                                radiansToDegrees(command.at(2) - pos_offsets[2]),
                                radiansToDegrees(command.at(3) - pos_offsets[3]),
                                radiansToDegrees(command.at(4) - pos_offsets[4]),
                                radiansToDegrees(command.at(5) - pos_offsets[5]),
                                radiansToFingerTicks(command.at(6)),
                                radiansToFingerTicks(command.at(7)),
                                0);
        }

        void sendVelocityCommand(const std::vector<double>& command)
        {
            jaco_basic_traj_point_t robot_cmd;
            robot_cmd.pos_type = SPEED_ANGULAR;
            robot_cmd.hand_mode = MODE_SPEED;
            robot_cmd.time_delay = 0.0;

            jaco_position_t target;

            for (int i = 0; i < num_arm_dof; i++)
            {
                target.joints[i] = radiansToDegrees(command.at(i));
            }

            target.finger_position[0] = radiansToFingerTicks(command.at(6));
            target.finger_position[1] = radiansToFingerTicks(command.at(7));
            robot_cmd.target = target;

            arm->set_target(robot_cmd);
        }

        void write(void)
        {
            if (last_mode != joint_mode)
            {
                arm->erase_trajectories();
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

        jaco_position_t returnPos(void)
        {
            jaco_position_t arm_pos = arm->get_ang_pos();
            return arm_pos;
        }

        void checkForStall()
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
                    ROS_ERROR("Exceeded soft effort limits on joint %d. Limit=%f, Measured=%f", i, soft_limits[i], eff[i]);
                    if (!eff_stall)
                    {
                        ROS_INFO("Sending zero velocities");
                        sendVelocityCommand(zero_velocity_command);
                        ROS_INFO("Entering force_control mode.");
                        eff_stall = true;
                        arm->start_force_ctrl();
                    }
                }
            }

            if (all_in_limits && eff_stall)
            {
                eff_stall = false;
                ROS_INFO("Exiting force_control mode.");
                arm->stop_force_ctrl();
                arm->set_control_ang();
                sendVelocityCommand(zero_velocity_command);
            }
        }

        void read(void)
        {
            // make sure that pos, vel, and eff are up to date.
            // TODO: If there is too much lag between calling read()
            // and getting the actual values back, we'll need to be
            // reading values constantly and storing them locally, so
            // at least there is a recent value available for the controller.

            jaco_position_t arm_pos = arm->get_ang_pos();
            jaco_position_t arm_vel = arm->get_ang_vel();
            jaco_position_t arm_eff = arm->get_ang_force();
            for (int i = 0; i < num_arm_dof; i++)
            {
                /* note: here we convert to radians, and add our offsets;
                 * at some point we might reverse direction as well
                 * (when urdf/orxml models have been similarly updated) */
                pos[i] = degreesToRadians(arm_pos.joints[i]) + pos_offsets[i];
                vel[i] = degreesToRadians(arm_vel.joints[i]);
                eff[i] = arm_eff.joints[i];
            }

            for (int i = 0; i < 2; i++)
            {
                int j = i + num_arm_dof; // joint corresponding to finger i

                pos[j] = fingerTicksToRadians(arm_pos.finger_position[i]);
                vel[j] = fingerTicksToRadians(arm_vel.finger_position[i]);
                eff[j] = fingerTicksToRadians(arm_eff.finger_position[i]); //this is likely to be meaningless
            }

            checkForStall();
        }

        bool eff_stall;

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::JointModeInterface jm_interface;

        JacoArm *arm;
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
