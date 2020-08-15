#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
// #include<mavros_msgs/ExtendedState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/Thrust.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#include "nmpc_common.h"
#include "nmpc_auxiliary_functions.h"

extern double sampleTime;

extern NMPCworkspace nmpcWorkspace;
extern NMPCvariables nmpcVariables;

class NMPC_PC
{
	private:
		double m;
		double g;
		double min_Fz_scale;
		double max_Fz_scale;
		
		bool is_control_init;

		Eigen::VectorXd U_ref;
		Eigen::VectorXd W;
		Eigen::VectorXd WN;

		double x_ref_last;
		double y_ref_last;
		double z_ref_last;

	public:
	
		struct acado_struct
		{
			boost::function<int(void)> initializeSolver;
			boost::function<int(void)> preparationStep;
			boost::function<int(void)> feedbackStep;
			boost::function<real_t(void)> getKKT;
			boost::function<void(void)> printDifferentialVariables;
			boost::function<void(void)> printControlVariables;

			int acado_N;
			int acado_NX;
			int acado_NY;
			int acado_NYN;
			int acado_NU;
			int acado_NOD;

			real_t * x0;
			real_t * u;
			real_t * x;
			real_t * od;
			real_t * y;
			real_t * yN;
			real_t * W;
			real_t * WN;

		} nmpc_struct;

		struct command_struct
		{
			double roll_ang;
			double pitch_ang;
			double yaw_ang;
			double Fz;
			double Fz_scaled;	// between 0 and 1
			double exe_time;
			double kkt_tol;

		} nmpc_cmd_struct;

		NMPC_PC();
		~NMPC_PC();

		bool return_control_init_value();

		void nmpc_init(const geometry_msgs::PoseStamped& posref, struct acado_struct& acadostruct);

		void nmpc_core(struct acado_struct &acadostruct, struct command_struct &commandstruct, Eigen::Vector3d reftrajectory, Eigen::Vector3d &nmhemFxFy, geometry_msgs::PoseStamped &currentpos, geometry_msgs::TwistStamped &currentvelrates);

		void publish_rpyFz(struct command_struct &commandstruct);

	protected:
		ros::NodeHandle private_nh;

		void set_measurements(struct acado_struct &acadostruct, Eigen::Vector3d &nmhemFxFy, geometry_msgs::PoseStamped &currentpos, geometry_msgs::TwistStamped &currentvelrates);

		void set_reftrajectory(struct acado_struct &acadostruct, Eigen::Vector3d reftrajectory);

		// Subscribers
//		ros::Subscriber local_traj_sub;
		ros::Subscriber local_pos_sub;
		ros::Subscriber local_vel_rates_sub;
		ros::Subscriber nmhe_mFxFy_sub;
		ros::Subscriber local_traj_sub;

		// Publishers
		ros::Publisher att_throttle_pub;
		ros::Publisher attitude_pub;
		ros::Publisher nmpc_cmd_rpy_pub;
		ros::Publisher nmpc_cmd_Fz_pub;
		ros::Publisher nmpc_cmd_exeTime_pub;
		ros::Publisher nmpc_cmd_kkt_pub;
};

ros::Subscriber state_sub;

Eigen::Vector3d ref_trajectory;
int ref_traj_type;
double t, t_pc_loop;
