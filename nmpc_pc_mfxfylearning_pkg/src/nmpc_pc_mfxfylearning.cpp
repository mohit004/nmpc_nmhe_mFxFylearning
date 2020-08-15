/**
 * @file   nmpc_pc_mFxFylearning.cpp
 * @author Mohit Mehndiratta
 * @date   September 2017
 *
 * @copyright
 * Copyright (C) 2017.
 */

#include <nmpc_pc_mfxfylearning.h>
#include <boost/shared_ptr.hpp>

using namespace Eigen;
using namespace ros;

double sampleTime = 0.02;

NMPCworkspace nmpcWorkspace;
NMPCvariables nmpcVariables;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
}

geometry_msgs::TwistStamped current_vel_rates;
void local_vel_rates_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rates = *msg;
}

Vector3d nmhe_mFxFy;
void nmhe_mFxFy_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmhe_mFxFy(i) = *itr;
        i++;
    }
}

void local_traj_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_trajectory << msg->x, msg->y, msg->z;
}


NMPC_PC::NMPC_PC()
{
    /* ************************************** */
    /* ROS communication
    /* ************************************** */

    is_control_init = false;

//    m = 1.412;
    m = 1.443;
//    m = 1.9;
    g = 9.81;
    min_Fz_scale = 0* m*g;
    max_Fz_scale = 1.8* m*g;

    VectorXd Uref_dummy(NMPC_NU);
    VectorXd W_dummy(NMPC_NY);
    VectorXd WN_dummy(NMPC_NYN);

    Uref_dummy << -0.0414, 0.0, 0.0, m*g;

    W_dummy << 30, 32, 32, 1.3, 1.3, 1.3, 22, 22, 80, 1.8e-2;           // New White tricopter FINAL weights
    WN_dummy << 50, 50, 50, 1, 1, 1;

//    W_dummy << 25, 26, 32, 1, 1, 1.1, 22, 25, 80, 2.4e-2;           // White tricopter FINAL weights
//    WN_dummy << 40, 40, 40, 1, 1, 1;

//    W_dummy << 35, 36, 25, 1, 1, 1.5, 22, 25, 80, 2.6e-2;           // Black Talon tricopter FINAL weights
//    WN_dummy << 50, 50, 50, 1, 1, 1;

    U_ref = Uref_dummy;
    W = W_dummy;
    WN = WN_dummy;

    // ----------
    // Subscribers
    // ----------
    local_pos_sub = private_nh.subscribe<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1, local_pos_cb);
    local_vel_rates_sub = private_nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity", 1, local_vel_rates_cb);
    nmhe_mFxFy_sub = private_nh.subscribe<std_msgs::Float64MultiArray>("nmhe_learning/mFxFy", 1, nmhe_mFxFy_cb);
    local_traj_sub = private_nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/pose", 1, local_traj_cb);

    // ----------
    // Publishers
    // ----------
    att_throttle_pub = private_nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 1, true);
    attitude_pub = private_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 1, true);
    nmpc_cmd_rpy_pub = private_nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/rpy", 1, true);
    nmpc_cmd_Fz_pub = private_nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/Fz_FzScaled", 1, true);
    nmpc_cmd_exeTime_pub = private_nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/exeTime", 1, true);
    nmpc_cmd_kkt_pub = private_nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/kkt", 1, true);

    // --------------------
    // ACADO NMPC CONTROLLER
    // --------------------

    nmpc_struct.initializeSolver = boost::bind(nmpc_initializeSolver);
    nmpc_struct.preparationStep = boost::bind(nmpc_preparationStep);
    nmpc_struct.feedbackStep = boost::bind(nmpc_feedbackStep);
    nmpc_struct.getKKT = boost::bind(nmpc_getKKT);
    nmpc_struct.printDifferentialVariables = boost::bind(nmpc_printDifferentialVariables);
    nmpc_struct.printControlVariables = boost::bind(nmpc_printControlVariables);

    nmpc_struct.acado_N = NMPC_N;
    nmpc_struct.acado_NX = NMPC_NX;
    nmpc_struct.acado_NY = NMPC_NY;
    nmpc_struct.acado_NYN = NMPC_NYN;
    nmpc_struct.acado_NU = NMPC_NU;
    nmpc_struct.acado_NOD = NMPC_NOD;

    nmpc_struct.x0 = &nmpcVariables.x0[0];
    nmpc_struct.x = &nmpcVariables.x[0];
    nmpc_struct.od = &nmpcVariables.od[0];
    nmpc_struct.y = &nmpcVariables.y[0];
    nmpc_struct.yN = &nmpcVariables.yN[0];
    nmpc_struct.u = &nmpcVariables.u[0];
    nmpc_struct.W = &nmpcVariables.W[0];
    nmpc_struct.WN = &nmpcVariables.WN[0];

    nmpc_cmd_struct.roll_ang = 0.0;
    nmpc_cmd_struct.pitch_ang = 0.0;
    nmpc_cmd_struct.yaw_ang = 0.0;
    nmpc_cmd_struct.Fz = m*g;
//    nmpc_cmd_struct.Fz_scaled = 0.0;
    nmpc_cmd_struct.Fz_scaled = ( (1 - 0)/(max_Fz_scale - min_Fz_scale) ) * (nmpc_cmd_struct.Fz - min_Fz_scale);
    nmpc_cmd_struct.exe_time = 0.0;
    nmpc_cmd_struct.kkt_tol = 0.0;

    ROS_INFO_STREAM("Constructor of the class NMPC_PC is created");

}

NMPC_PC::~NMPC_PC()
{
    ROS_INFO_STREAM("Destructor of the class NMPC_PC");
}

bool NMPC_PC::return_control_init_value()
{
    return NMPC_PC::is_control_init;
}

void NMPC_PC::nmpc_init(const geometry_msgs::PoseStamped& posref, struct acado_struct& acadostruct)
{

    ROS_INFO_STREAM("outer_nmpc_initController - start");

    // Initialize the solver
    // ---------------------
    acadostruct.initializeSolver();

    // NMPC: initialize/set the states
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NX * (acadostruct.acado_N + 1); ++i)
    {
        acadostruct.x[i] = 0.0;
    }

    // NMPC: initialize/set the controls
    // ---------------------
//    for (int i = 0; i < acadostruct.acado_NU * acadostruct.acado_N; ++i)
//    {
//        acadostruct.u[i] = 0.0;
//    }
    for (int i = 0; i < acadostruct.acado_N; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NU; ++j)
            acadostruct.u[(i * acadostruct.acado_NU) + j] = U_ref[j];
    }

    // NMPC: initialize/set the online data
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NOD * (acadostruct.acado_N + 1); ++i)
    {
        if(i%acadostruct.acado_NOD == 0)
        {
            acadostruct.od[i] = m;
        }
        else
            acadostruct.od[i] = 0.0;
//        std::cout<<"acadostruct.od["<<i<<"] ="<<acadostruct.od[i]<<"\n";
    }

    // NMPC: initialize the measurements/reference
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NY * acadostruct.acado_N; ++i)
    {
        acadostruct.y[i] = 0.0;
    }
    for (int i = 0; i < acadostruct.acado_NYN; ++i)
    {
        acadostruct.yN[i] = 0.0;
    }

    // NMPC: initialize the current state feedback
    // ---------------------
#if ACADO_INITIAL_STATE_FIXED
    for (int i = 2; i < acadostruct.acado_NX; ++i)
    {
        if (i < 3)
        {
            acadostruct.x0[0] = posref.pose.position.x;
            acadostruct.x0[1] = posref.pose.position.y;
            acadostruct.x0[2] = posref.pose.position.z;
        }
        else
            acadostruct.x0[i] = 0;
    }
#endif

    // NMPC: initialize the weight matrices
    // ---------------------
    for(int i = 0; i < acadostruct.acado_NY; ++i )
    {
        for(int j = 0; j < acadostruct.acado_NY; ++j )
        {
            if(i==j)
                acadostruct.W[(i * acadostruct.acado_NY) + j] = W[i];
            else
                acadostruct.W[(i * acadostruct.acado_NY) + j] = 0.0;
        }
    }
    for(int i = 0; i < acadostruct.acado_NYN; ++i )
    {
        for(int j = 0; j < acadostruct.acado_NYN; ++j )
        {
            if(i==j)
                acadostruct.WN[(i * acadostruct.acado_NYN) + j] = WN[i];
            else
                acadostruct.WN[(i * acadostruct.acado_NYN) + j] = 0.0;
        }
    }

    // Prepare first step
    // ------------------
    acadostruct.preparationStep();

    ROS_INFO("NMPC_mFxFylearning: initialized correctly");
    is_control_init = true;

}

void NMPC_PC::nmpc_core(struct acado_struct &acadostruct, struct command_struct &commandstruct,
                        Vector3d reftrajectory, Vector3d &nmhemFxFy, geometry_msgs::PoseStamped &currentpos,
                        geometry_msgs::TwistStamped &currentvelrates)
{

    // set the current state feedback
    set_measurements(acadostruct, nmhemFxFy, currentpos, currentvelrates);

    U_ref << -0.0414, 0.0, 0.0, nmhemFxFy(0)*g;

    // set the reference path
    set_reftrajectory(acadostruct, reftrajectory);

    // NMPC: calc and apply control and prepare optimization for the next step
    // ----------------------------------------------------------------------

    // Execute Calculation (Optimization)
    ros::Time stopwatch = ros::Time::now();
    int acado_feedbackStep_fb = acadostruct.feedbackStep();

    if (acado_feedbackStep_fb != 0)
    {
        ROS_ERROR_STREAM("ACADO ERROR: " << acado_feedbackStep_fb);
        ROS_ERROR_STREAM(
                "acado outer nmpc controller states: x, y, z, u, v, w" << acadostruct.x0[0] << " "
                << acadostruct.x0[1] << " " << acadostruct.x0[2] << " " << acadostruct.x0[3] << " "
                << acadostruct.x0[4] << " " << acadostruct.x0[5]);
    }

    // Apply the new control immediately to the process, first NU components.
    commandstruct.roll_ang = acadostruct.u[0];
    commandstruct.pitch_ang = acadostruct.u[1];
    commandstruct.yaw_ang = acadostruct.u[2];
    commandstruct.Fz = acadostruct.u[3];
    commandstruct.Fz_scaled = ( (1 - 0)/(max_Fz_scale - min_Fz_scale) ) * (commandstruct.Fz - min_Fz_scale);
    commandstruct.exe_time = ros::Time::now().toSec() - stopwatch.toSec();
    commandstruct.kkt_tol = acadostruct.getKKT();

    // Publish NMPC output
    publish_rpyFz(commandstruct);

    // Settings for the next iteration
    acadostruct.preparationStep();

    ROS_INFO_STREAM("Stoptime outer NMPC: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");

    /* ------ NMPC_DEBUG ------*/
//    acadostruct.printDifferentialVariables();
//    acadostruct.printControlVariables();
}

void NMPC_PC::set_measurements(struct acado_struct &acadostruct, Vector3d &nmhemFxFy,
                               geometry_msgs::PoseStamped &currentpos,  geometry_msgs::TwistStamped &currentvelrates)
{
    acadostruct.x0[0] = currentpos.pose.position.x;
    acadostruct.x0[1] = currentpos.pose.position.y;
    acadostruct.x0[2] = currentpos.pose.position.z;
    acadostruct.x0[3] = currentvelrates.twist.linear.x;
    acadostruct.x0[4] = currentvelrates.twist.linear.y;
    acadostruct.x0[5] = currentvelrates.twist.linear.z;

    for (int i = 0; i < acadostruct.acado_N + 1; ++i)
    {
//        std::cout<<"m = "<<nmhemass.data<<"\n";
        acadostruct.od[(i * acadostruct.acado_NOD)]     = nmhemFxFy(0);
        acadostruct.od[(i * acadostruct.acado_NOD) + 1] = nmhemFxFy(1);
        acadostruct.od[(i * acadostruct.acado_NOD) + 2] = nmhemFxFy(2);
        acadostruct.od[(i * acadostruct.acado_NOD) + 3] = currentvelrates.twist.angular.x;
        acadostruct.od[(i * acadostruct.acado_NOD) + 4] = currentvelrates.twist.angular.y;
        acadostruct.od[(i * acadostruct.acado_NOD) + 5] = currentvelrates.twist.angular.z;
    }
}

void NMPC_PC::set_reftrajectory(struct acado_struct &acadostruct, Vector3d reftrajectory)
{
    acadostruct.yN[0] = reftrajectory(0);
    acadostruct.yN[1] = reftrajectory(1);
    acadostruct.yN[2] = reftrajectory(2);

    for(int i = 0; i < acadostruct.acado_N; ++i )
    {
        for(int j = 0; j < acadostruct.acado_NY; ++j )
        {
            if (j < acadostruct.acado_NX)
                acadostruct.y[(i * acadostruct.acado_NY) + j] = acadostruct.yN[j];
            else
                acadostruct.y[(i * acadostruct.acado_NY) + j] = U_ref[j - acadostruct.acado_NX];
        }
    }
}

void NMPC_PC::publish_rpyFz(struct command_struct &commandstruct)
{
//    std_msgs::Float64 att_thro_cmd;
//    att_thro_cmd.data = commandstruct.Fz_scaled;
    mavros_msgs::Thrust att_thro_cmd;
    att_thro_cmd.header.frame_id = "";
    att_thro_cmd.header.stamp = ros::Time::now();
    att_thro_cmd.thrust = commandstruct.Fz_scaled;
    att_throttle_pub.publish(att_thro_cmd);

    tf::Quaternion q(current_pos.pose.orientation.x, current_pos.pose.orientation.y, current_pos.pose.orientation.z, current_pos.pose.orientation.w);
    q.setRPY(commandstruct.roll_ang, commandstruct.pitch_ang, commandstruct.yaw_ang);

    geometry_msgs::PoseStamped attitude_cmd;
    attitude_cmd.header.frame_id = "";
    attitude_cmd.header.stamp = ros::Time::now();
    attitude_cmd.pose.orientation.x = q.getX();
    attitude_cmd.pose.orientation.y = q.getY();
    attitude_cmd.pose.orientation.z = q.getZ();
    attitude_cmd.pose.orientation.w = q.getW();
    attitude_pub.publish(attitude_cmd);

    std::vector<double> rpy_vec = {commandstruct.roll_ang, commandstruct.pitch_ang, commandstruct.yaw_ang};
    std_msgs::Float64MultiArray rpy_cmd;
    rpy_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rpy_cmd.layout.dim[0].size = rpy_vec.size();
    rpy_cmd.layout.dim[0].stride = 1;
    rpy_cmd.layout.dim[0].label = "Roll, Pitch, Yaw (rad)";
    rpy_cmd.data.clear();
    rpy_cmd.data.insert(rpy_cmd.data.end(), rpy_vec.begin(), rpy_vec.end());
    nmpc_cmd_rpy_pub.publish(rpy_cmd);

    std::vector<double> Fz_vec = {commandstruct.Fz, commandstruct.Fz_scaled};
    std_msgs::Float64MultiArray Fz_cmd;
    Fz_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    Fz_cmd.layout.dim[0].size = Fz_vec.size();
    Fz_cmd.layout.dim[0].stride = 1;
    Fz_cmd.layout.dim[0].label = "Fz (N), Fz_scaled";
    Fz_cmd.data.clear();
    Fz_cmd.data.insert(Fz_cmd.data.end(), Fz_vec.begin(), Fz_vec.end());
    nmpc_cmd_Fz_pub.publish(Fz_cmd);

    std_msgs::Float64 exe_time_cmd;
    exe_time_cmd.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub.publish(exe_time_cmd);

    std_msgs::Float64 kkt_tol_cmd;
    kkt_tol_cmd.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub.publish(kkt_tol_cmd);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_pc_mfxfylearning");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);

    NMPC_PC *nmpc_pc = new NMPC_PC();

    ros::Rate rate(1/sampleTime);

    ref_traj_type = 0;
    ref_trajectory << 0, 0, 0;

    int print_flag_offboard = 1;
    int print_flag_arm = 1;
    int print_flag_altctl = 1;
    geometry_msgs::PoseStamped pos_ref;

    bool control_stop = false;

    while(ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec();

        if( current_state.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if( !current_state.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if(current_state.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if( current_state.mode == "ALTCTL")
        {
            pos_ref = current_pos;
            if(print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        while(ros::ok() && current_state.mode == "OFFBOARD" && !control_stop)
        {
            if(!nmpc_pc->return_control_init_value())
                nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);

            t_pc_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_pc_loop - (int)(t_pc_loop)), (double)(sampleTime)) == 0)
                std::cout<<"loop time for outer NMPC: " << t_pc_loop << " (sec)"<<"\n";

            nmpc_pc->nmpc_core(nmpc_pc->nmpc_struct, nmpc_pc->nmpc_cmd_struct, ref_trajectory, nmhe_mFxFy, current_pos, current_vel_rates);
            if(std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
               std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
            {
                ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t <<" (sec)" );
                control_stop = true;
                exit;
            }

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            ros::spinOnce();
            rate.sleep();
        }

        nmpc_pc->publish_rpyFz(nmpc_pc->nmpc_cmd_struct);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
