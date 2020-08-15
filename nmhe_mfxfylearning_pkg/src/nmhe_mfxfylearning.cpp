/**
 * @file   nmhe_mFxFylearning.cpp
 * @author Mohit Mehndiratta
 * @date   September 2017
 *
 * @copyright
 * Copyright (C) 2017.
 */

#include <nmhe_mfxfylearning.h>
#include <boost/shared_ptr.hpp>

using namespace Eigen;
using namespace ros;

double sampleTime = 0.03;

NMHEworkspace nmheWorkspace;
NMHEvariables nmheVariables;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
VectorXd current_vel_rates(6);
void local_vel_rates_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rates << msg->twist.linear.x, msg->twist.linear.y,
                         msg->twist.linear.z, msg->twist.angular.x,
                         msg->twist.angular.y, msg->twist.angular.z;
}

Vector3d nmpc_cmd_ryp;
void nmpc_cmd_rpy_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_cmd_ryp(i) = *itr;
        i++;
    }
}

Vector2d nmpc_cmd_Fz;
void nmpc_cmd_Fz_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_cmd_Fz(i) = *itr;
        i++;
    }
}

NMHE::NMHE()
{
    /* ************************************** */
    /* ROS communication
    /* ************************************** */

    is_estimator_init = false;

    VectorXd X0_dummy(NMHE_NX);
    VectorXd W_dummy(NMHE_NY);
    VectorXd WN_dummy(NMHE_NYN);

    X0_dummy << 0.0, 0.0, 0.0, 1.35, 0.1, 0.1;
//    W_dummy << 0.002, 0.002, 0.002, 0.01, 0.01, 0.05;
//    WN_dummy << 0.002, 0.002, 0.002;
    W_dummy << 0.0002, 0.0002, 0.0002, 0.0001, 0.0001, 0.0005;
    WN_dummy << 0.0002, 0.0002, 0.0002;

    X0 = X0_dummy;
    W = W_dummy;
    WN = WN_dummy;

    VectorXd SAC_dummy(NMHE_NX);
    VectorXd process_noise_cov(NMHE_NX);
    MatrixXd WL_mat_dummy(NMHE_NX,NMHE_NX);

    SAC_dummy << 1e-2, 1e-2, 1e-2, 1e-3, 1e-2, 1e-2;
    process_noise_cov << 30, 30, 30, 8, 8, 8;

    SAC = SAC_dummy;
    xAC = X0_dummy;

    for(int i = 0; i < NMHE_NX; ++i )
    {
        for(int j = 0; j < NMHE_NX; ++j )
        {
            if(i==j)
                WL_mat_dummy((i * NMHE_NX) + j) = 1/process_noise_cov(i);        // Check for lower triangle Cholesky decomposition!
            else
                WL_mat_dummy((i * NMHE_NX) + j) = 0.0001;
        }
    }

    Eigen::LLT<MatrixXd> lltofWL_mat_dummy(WL_mat_dummy);
    WL_mat = lltofWL_mat_dummy.matrixL();


//    WL_mat = WL_mat_dummy.inverse();
//    WL_mat = WL_mat_dummy;

    run_cnt = 1;

    // ----------
    // Subscribers
    // ----------
    local_vel_rates_sub = private_nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity", 1, local_vel_rates_cb);
    nmpc_cmd_rpy_sub = private_nh.subscribe<std_msgs::Float64MultiArray>("outer_nmpc_cmd/rpy", 1, nmpc_cmd_rpy_cb);
    nmpc_cmd_Fz_sub = private_nh.subscribe<std_msgs::Float64MultiArray>("outer_nmpc_cmd/Fz_FzScaled", 1, nmpc_cmd_Fz_cb);


    // ----------
    // Publishers
    // ----------
    nmhe_vel_pub = private_nh.advertise<std_msgs::Float64MultiArray>("nmhe_learning/uvw", 1, true);
    nmhe_mFxFy_pub = private_nh.advertise<std_msgs::Float64MultiArray>("nmhe_learning/mFxFy", 1, true);
    nmhe_exeTime_pub = private_nh.advertise<std_msgs::Float64>("nmhe_learning/exeTime", 1, true);
    nmhe_kkt_pub = private_nh.advertise<std_msgs::Float64>("nmhe_learning/kkt", 1, true);

    // --------------------
    // ACADO NMHE
    // --------------------

    nmhe_struct.initializeSolver = boost::bind(nmhe_initializeSolver);
    nmhe_struct.preparationStep = boost::bind(nmhe_preparationStep);
    nmhe_struct.feedbackStep = boost::bind(nmhe_feedbackStep);
    nmhe_struct.getKKT = boost::bind(nmhe_getKKT);
    nmhe_struct.printDifferentialVariables = boost::bind(nmhe_printDifferentialVariables);
    nmhe_struct.printControlVariables = boost::bind(nmhe_printControlVariables);
    nmhe_struct.updateArrivalCost = boost::bind(nmhe_updateArrivalCost, 0);


    nmhe_struct.acado_N = NMHE_N;
    nmhe_struct.acado_NX = NMHE_NX;
    nmhe_struct.acado_NY = NMHE_NY;
    nmhe_struct.acado_NYN = NMHE_NYN;
    nmhe_struct.acado_NU = NMHE_NU;
    nmhe_struct.acado_NOD = NMHE_NOD;

    nmhe_struct.x = &nmheVariables.x[0];
    nmhe_struct.u = &nmheVariables.u[0];
    nmhe_struct.od = &nmheVariables.od[0];
    nmhe_struct.y = &nmheVariables.y[0];
    nmhe_struct.yN = &nmheVariables.yN[0];
    nmhe_struct.W = &nmheVariables.W[0];
    nmhe_struct.WN = &nmheVariables.WN[0];
    nmhe_struct.SAC = &nmheVariables.SAC[0];
    nmhe_struct.xAC = &nmheVariables.xAC[0];
    nmhe_struct.WL = &nmheVariables.WL[0];

    nmhe_est_struct.u_est = X0(0);
    nmhe_est_struct.v_est = X0(1);
    nmhe_est_struct.w_est = X0(2);
    nmhe_est_struct.m_est = X0(3);
    nmhe_est_struct.Fx_dist_est = X0(4);
    nmhe_est_struct.Fy_dist_est = X0(5);
    nmhe_est_struct.exe_time = 0.0;
    nmhe_est_struct.kkt_tol = 0.0;

    ROS_INFO_STREAM("Constructor of the class NMHE_mFxFylearning is created");

}

NMHE::~NMHE()
{
    ROS_INFO_STREAM("Destructor of the class NMHE_mFxFylearning");
}

bool NMHE::return_estimator_init_value()
{
    return NMHE::is_estimator_init;
}

void NMHE::nmhe_init( struct acado_struct &acadostruct)
{

    ROS_INFO_STREAM("NMHE_mFxFylearning_initEstimator - start");

    // Initialize the solver
    // ---------------------
    acadostruct.initializeSolver();

    // NMHE: initialize/set the states
    // ---------------------
    for (int i = 0; i < acadostruct.acado_N + 1; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NX; ++j)
            acadostruct.x[(i * acadostruct.acado_NX) + j] = X0[j];
    }

    // NMHE: initialize/set the controls
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NU * acadostruct.acado_N; ++i)
    {
        acadostruct.u[i] = 0.0;
    }

    // NMHE: initialize/set the online data
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NOD * (acadostruct.acado_N + 1); ++i)
    {
        acadostruct.od[i] = 0.0;
    }

    // NMHE: initialize the measurements/reference
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NY * acadostruct.acado_N; ++i)
    {
        acadostruct.y[i] = 0.0;
    }
    for (int i = 0; i < acadostruct.acado_NYN; ++i)
    {
        acadostruct.yN[i] = 0.0;
    }

    // NMHE: initialize the current state feedback
    // ---------------------
/*
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
*/
    // NMHE: initialize the weight matrices
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

    // NMHE: initialize the arrival cost
    // ---------------------
    for(int i = 0; i < acadostruct.acado_NX; ++i )
    {
        for(int j = 0; j < acadostruct.acado_NX; ++j )
        {
            if(i==j)
                acadostruct.SAC[(i * acadostruct.acado_NX) + j] = 1/SAC[i];
            else
                acadostruct.SAC[(i * acadostruct.acado_NX) + j] = 0.0001;
        }
    }
    for(int i = 0; i < acadostruct.acado_NX; ++i )
    {
        for(int j = 0; j < acadostruct.acado_NX; ++j )
        {
            if(i>=j)
                acadostruct.WL[(i * acadostruct.acado_NX) + j] = WL_mat(i);
            else
                acadostruct.WL[(i * acadostruct.acado_NX) + j] = 0.0001;
        }
    }

    // Prepare first step
    // ------------------
    acadostruct.preparationStep();

    acadostruct.updateArrivalCost(1);               // pass 1 to init the SAC matrix
/*
    std::cout<<"SAC = ";
    for(int i = 0; i < acadostruct.acado_NX; ++i )
    {
        for(int j = 0; j < acadostruct.acado_NX; ++j )
        {
            std::cout<<nmheVariables.SAC[(i * acadostruct.acado_NX) + j]<<", ";
        }
        std::cout<<"\n";
    }
*/
    MatrixXd SAC_dummy(acadostruct.acado_NX,acadostruct.acado_NX);
    int is_nan = 0;
    for(int i = 0; i < acadostruct.acado_NX; ++i )
    {
        for(int j = 0; j < acadostruct.acado_NX; ++j )
        {
            SAC_dummy(i,j) = nmheVariables.SAC[(i * acadostruct.acado_NX) + j];
            if(std::isnan(SAC_dummy(i,j)))
                is_nan++;
        }
    }
    if(is_nan != 0)
    {
        ROS_ERROR_STREAM("nmheVariables.SAC has " << is_nan
                         <<" NANs! \n reinitialize SAC matrix!" );
        exit;
    }

    ROS_INFO("NMHE_mFxFylearning: initialized correctly");
    is_estimator_init = true;
}

void NMHE::nmhe_core(struct acado_struct &acadostruct, struct estimation_struct &estimationstruct,
                     VectorXd &currentvelrates, Vector3d &nmpccmdryp, Vector2d &nmpccmdFz)
{

    Vector3d nmpc_cmd;
    nmpc_cmd << nmpccmdryp(0), nmpccmdryp(1), nmpccmdFz(0);

    // set the measurement feedback
    set_measurements(acadostruct, currentvelrates, nmpc_cmd);

    // NMHE: calc and give estimation and prepare optimization for the next step
    // ----------------------------------------------------------------------

    // Execute Calculation (Optimization)
    ros::Time stopwatch = ros::Time::now();
    int acado_feedbackStep_fb = acadostruct.feedbackStep();

    if (acado_feedbackStep_fb != 0)
    {
        ROS_ERROR_STREAM("ACADO ERROR: " << acado_feedbackStep_fb);
        ROS_ERROR_STREAM(
                "acado NMHE_mLearning states: u, v, w, m, Fx_dist, Fy_dist  = " << acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX)] << ", "
                << acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 1] << ", "
                << acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 2] << ", "
                << acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 3] << ", "
                << acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 4] << ", "
                << acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 5]);
    }

    // Feedback the new estimation immediately to the process, first NU components.
    estimationstruct.u_est = acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX)];
    estimationstruct.v_est = acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 1];
    estimationstruct.w_est = acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 2];
    estimationstruct.m_est = acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 3];
    estimationstruct.Fx_dist_est = acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 4];
    estimationstruct.Fy_dist_est = acadostruct.x[(acadostruct.acado_N*acadostruct.acado_NX) + 5];
    estimationstruct.exe_time = ros::Time::now().toSec() - stopwatch.toSec();
    estimationstruct.kkt_tol = acadostruct.getKKT();

    // Publish NMHE output
    publish_uvw_mFxFy(estimationstruct);

    acadostruct.updateArrivalCost(0);                       // pass 0 to just update the arrival cost

    // Settings for the next iteration
    acadostruct.preparationStep();

    ROS_INFO_STREAM("Stoptime NMHE_mFxFylearning: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");

    /* ------ NMHE_DEBUG ------*/
//    acadostruct.printDifferentialVariables();
//    acadostruct.printControlVariables();

}

void NMHE::set_measurements(struct acado_struct &acadostruct, VectorXd &currentvelrates,
                            Vector3d &nmpccmd)
{
    // Fill in the measurement buffer, entries 1: N
    if (run_cnt < (acadostruct.acado_N + 1))
    {
        for(int i = 0; i < acadostruct.acado_NYN; ++i )
        {
            acadostruct.y[(run_cnt-1)*acadostruct.acado_NY + i] = currentvelrates(i);
        }

        if (run_cnt > 1)
        {
            for(int i = acadostruct.acado_NYN; i < acadostruct.acado_NY; ++i )
            {
                acadostruct.y[(run_cnt-2)*acadostruct.acado_NY + i] = nmpccmd(i - acadostruct.acado_NYN);
            }
        }

        // Initialize solver, measured states, measured control and online data on shooting nodes 1: N
        for(int i = 0; i < acadostruct.acado_NX-3; ++i )
        {
            acadostruct.x[(run_cnt-1)*acadostruct.acado_NX + i] = currentvelrates(i);
        }
        for(int i = 0; i < acadostruct.acado_NU; ++i )
        {
            if (run_cnt > 1)
                acadostruct.u[(run_cnt-2)*acadostruct.acado_NU + i] = nmpccmd(i);
        }
        for(int i = 0; i < acadostruct.acado_NOD; ++i )
        {
            acadostruct.od[(run_cnt-1)*acadostruct.acado_NOD + i] = currentvelrates(i + 3);
        }

        std::cout<<"run_cnt = "<<run_cnt<<"\n";

        // Increment counter
        run_cnt++;
    }

    // Initialize measurements on node N + 1,
    else if(run_cnt == (acadostruct.acado_N + 1))
    {
        for(int i = 0; i < acadostruct.acado_NYN; ++i )
            acadostruct.yN[i] = currentvelrates(i);

        // Initialize measured controls on previous node, node N
        for(int i = acadostruct.acado_NYN; i < acadostruct.acado_NY; ++i )
        {
            acadostruct.y[(acadostruct.acado_N-1)*acadostruct.acado_NY + i] = nmpccmd(i - acadostruct.acado_NYN);
        }

        // Initialize measured states, measured control and online data on node N + 1
        for(int i = 0; i < acadostruct.acado_NYN; ++i )
        {
            acadostruct.x[(run_cnt-1)*acadostruct.acado_NX + i] = currentvelrates(i);
        }
        for(int i = 0; i < acadostruct.acado_NU; ++i )
        {
            acadostruct.u[(acadostruct.acado_N-1)*acadostruct.acado_NU + i] = nmpccmd(i);
        }
        for(int i = 0; i < acadostruct.acado_NOD; ++i )
        {
            acadostruct.od[(run_cnt-1)*acadostruct.acado_NOD + i] = currentvelrates(i + 3);
        }

        std::cout<<"run_cnt = "<<run_cnt<<"\n";

        // Increment counter
        run_cnt++;
    }

    // Shift measurements
    else
    {
        for(int i = 0; i < acadostruct.acado_N-1; ++i )
        {
            for(int j = 0; j < acadostruct.acado_NY; ++j )
                acadostruct.y[i*acadostruct.acado_NY + j] = acadostruct.y[(i+1)*acadostruct.acado_NY + j];
        }

        for(int i = 0; i < acadostruct.acado_NYN; ++i )
        {
            acadostruct.yN[i] = currentvelrates(i);
            acadostruct.y[(acadostruct.acado_N-1)*acadostruct.acado_NY + i] = acadostruct.yN[i];
        }
        for(int i = acadostruct.acado_NYN; i < acadostruct.acado_NY; ++i )
        {
            acadostruct.y[(acadostruct.acado_N-1)*acadostruct.acado_NY + i] = nmpccmd(i - acadostruct.acado_NYN);
        }
        for (int i = 0; i < acadostruct.acado_N + 1; ++i)
        {
            for(int j = 0; j < acadostruct.acado_NOD; ++j )
                acadostruct.od[(i * acadostruct.acado_NOD) + j] = currentvelrates(j + 3);
        }
    }

}

void NMHE::publish_uvw_mFxFy(struct estimation_struct &estimationstruct)
{
    std::vector<double> uvw_vec = {estimationstruct.u_est, estimationstruct.v_est, estimationstruct.w_est};
    std_msgs::Float64MultiArray uvw_cmd;
    uvw_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    uvw_cmd.layout.dim[0].size = uvw_vec.size();
    uvw_cmd.layout.dim[0].stride = 1;
    uvw_cmd.layout.dim[0].label = "u_vel, v_vel, w_vel (m/sec)";
    uvw_cmd.data.clear();
    uvw_cmd.data.insert(uvw_cmd.data.end(), uvw_vec.begin(), uvw_vec.end());
    nmhe_vel_pub.publish(uvw_cmd);

    std::vector<double> mFxFy_vec = {estimationstruct.m_est, estimationstruct.Fx_dist_est, estimationstruct.Fy_dist_est};
    std_msgs::Float64MultiArray mFxFy_cmd;
    mFxFy_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mFxFy_cmd.layout.dim[0].size = mFxFy_vec.size();
    mFxFy_cmd.layout.dim[0].stride = 1;
    mFxFy_cmd.layout.dim[0].label = "mass (Kg), Fx_dist, Fy_dist (N)";
    mFxFy_cmd.data.clear();
    mFxFy_cmd.data.insert(mFxFy_cmd.data.end(), mFxFy_vec.begin(), mFxFy_vec.end());
    nmhe_mFxFy_pub.publish(mFxFy_cmd);

    std_msgs::Float64 exe_time_cmd;
    exe_time_cmd.data = estimationstruct.exe_time;
    nmhe_exeTime_pub.publish(exe_time_cmd);

    std_msgs::Float64 kkt_tol_cmd;
    kkt_tol_cmd.data = estimationstruct.kkt_tol;
    nmhe_kkt_pub.publish(kkt_tol_cmd);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmhe_mfxfylearning");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);

    NMHE *nmhe = new NMHE();

    ros::Rate rate(1/sampleTime);

    int print_flag_offboard = 1;
    int print_flag_arm = 1;
    int print_flag_altctl = 1;

    bool estimator_stop = false;

    while(ros::ok() && !estimator_stop)
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
            if(print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        while(ros::ok() && current_state.mode == "OFFBOARD" && !estimator_stop)
        {
            if(!nmhe->return_estimator_init_value())
                nmhe->nmhe_init(nmhe->nmhe_struct);

            t_est_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_est_loop - (int)(t_est_loop)), (double)(sampleTime)) == 0)
                std::cout<<"loop time for NMHE: " << t_est_loop << " (sec)"<<"\n";

            nmhe->nmhe_core(nmhe->nmhe_struct, nmhe->nmhe_est_struct, current_vel_rates, nmpc_cmd_ryp, nmpc_cmd_Fz);
            if(std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX)]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX + 1)]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 2]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 3]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 4]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 5]) == true)
            {
                ROS_ERROR_STREAM("Estimator ERROR at time = " << ros::Time::now().toSec() - t <<" (sec)" );
                estimator_stop = true;
                exit;
            }

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            ros::spinOnce();
            rate.sleep();
        }

        nmhe->publish_uvw_mFxFy(nmhe->nmhe_est_struct);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
