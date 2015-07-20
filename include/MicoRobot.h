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

using namespace std;

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
                ROS_INFO("Set soft_limits for eff to: [%f,%f,%f,%f,%f,%f,%f,%f]", soft_limits[0], soft_limits[1], soft_limits[2], soft_limits[3], soft_limits[4], soft_limits[5], soft_limits[6], soft_limits[7]);
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

        virtual ~MicoRobot()
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

        void initializeOffsets()
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
            return (M_PI / 180.0) * degrees;
        }

        inline double radiansToDegrees(double radians)
        {
            return (180.0 / M_PI) * radians;
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
            // Need to send an "advance trajectory" with a single point and the correct settings
            // Angular position
            
            // Here the pos_offsets are just to avoid the arm "unwinding" back to zero when we 
            // first use position control since that might cause unintended effects.
            
            AngularInfo joint_pos;
            joint_pos.InitStruct();
            joint_pos.Actuator1 = float(radiansToDegrees(command.at(0) - pos_offsets[0]));
            joint_pos.Actuator2 = float(radiansToDegrees(command.at(1) - pos_offsets[1]));
            joint_pos.Actuator3 = float(radiansToDegrees(command.at(2) - pos_offsets[2]));
            joint_pos.Actuator4 = float(radiansToDegrees(command.at(3) - pos_offsets[3]));
            joint_pos.Actuator5 = float(radiansToDegrees(command.at(4) - pos_offsets[4]));
            joint_pos.Actuator6 = float(radiansToDegrees(command.at(5) - pos_offsets[5]));
            
            TrajectoryPoint trajectory;
            trajectory.InitStruct(); // initialize structure
            memset(&trajectory, 0, sizeof(trajectory));  // zero out the structure
            trajectory.Position.Type = ANGULAR_POSITION; // set to angular position 
            trajectory.Position.Actuators = joint_pos; // position is passed in the position struct
            
            int r = NO_ERROR_KINOVA;
            r = SendAdvanceTrajectory(trajectory);
            if (r != NO_ERROR_KINOVA) {
                ROS_ERROR("Could not send : Error code %d",r);
            }

        }

        void sendVelocityCommand(const std::vector<double>& command)
        {
            // Need to send an "advance trajectory" with a single point and the correct settings
            // Angular velocity
            
            AngularInfo joint_vel;
            joint_vel.InitStruct();
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
            
            int r = NO_ERROR_KINOVA;
            r = SendAdvanceTrajectory(trajectory);
            if (r != NO_ERROR_KINOVA) {
                ROS_ERROR("Could not send : Error code %d",r);
            }
        }
        
        void sendTorqueCommand(const std::vector<double>& command)
        {
            //SendAngularTorqueCommand()
        }

        void write(void)
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
                        //arm->start_force_ctrl();
                    }
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

        void read(void)
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
            
            pos[0] = double(arm_pos.Actuators.Actuator1);
            pos[1] = double(arm_pos.Actuators.Actuator2);
            pos[2] = double(arm_pos.Actuators.Actuator3);
            pos[3] = double(arm_pos.Actuators.Actuator4);
            pos[4] = double(arm_pos.Actuators.Actuator5);
            pos[5] = double(arm_pos.Actuators.Actuator6);
            pos[6] = double(arm_pos.Fingers.Finger1);
            pos[7] = double(arm_pos.Fingers.Finger2);
            
            vel[0] = double(arm_vel.Actuators.Actuator1);
            vel[1] = double(arm_vel.Actuators.Actuator2);
            vel[2] = double(arm_vel.Actuators.Actuator3);
            vel[3] = double(arm_vel.Actuators.Actuator4);
            vel[4] = double(arm_vel.Actuators.Actuator5);
            vel[5] = double(arm_vel.Actuators.Actuator6);
            vel[6] = double(arm_vel.Fingers.Finger1);
            vel[7] = double(arm_vel.Fingers.Finger2);
            
            eff[0] = double(arm_torq.Actuator1);
            eff[1] = double(arm_torq.Actuator2);
            eff[2] = double(arm_torq.Actuator3);
            eff[3] = double(arm_torq.Actuator4);
            eff[4] = double(arm_torq.Actuator5);
            eff[5] = double(arm_torq.Actuator6);
            eff[6] = 0;
            eff[7] = 0;
            
            checkForStall();
            
        }

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
