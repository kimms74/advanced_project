#ifndef CONTROLLER_H_
#define CONTROLLER_H_

//////////////////////////////////////////////////
//   for both real and simulation environment   //
#define USING_REAL_ROBOT
//////////////////////////////////////////////////

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"
#include "Astar.h"

#ifdef USING_REAL_ROBOT
#include "franka_model_interface.h"
#endif  // USING_REAL_ROBOT

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;
using namespace DyrosMath;

class ArmController
{
	size_t dof_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_;

	// Task space
	Vector3d x_init_;
	Vector3d x_init_2_;
	Vector3d x_dot_init_;
	Vector3d x_;
	Vector3d x_2;
	Matrix3d rotation_;
	Matrix3d rotation_init_;
	Matrix3d rotation_target_;
	Matrix3d rotation_cubic_;
	Vector3d rotation_cubic_dot;
	Vector3d phi_;
	Vector6d x_dot_; // 6D (linear + angular)
	Vector6d x_dot_2_; // 6D (linear + angular)
	Vector6d x_error_;
	Vector3d x_from_q_desired_;
	Matrix3d rotation_from_q_desired_;

	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix

	// For controller
	Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage 
	Matrix<double, 6, 7> j_2;
	Matrix<double, 3, 7> j_2_v_;
	Matrix<double, 6, 7> j_from_q_desired_;

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd j_temp_;	// For RBDL 
	MatrixXd j_temp_2;
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	Vector7d q_cubic_;
	Vector7d qdot_cubic_;
	Vector7d q_target_;
	Vector7d qdot_target_;

	Vector3d x_cubic_;
	Vector3d x_cubic_2_;
	Vector3d x_cubic_old_;
	Vector6d x_cubic_dot_;
	Vector3d x_cubic_dot_1_;
	Vector3d x_cubic_dot_2_;
	Vector3d x_target_;
	Vector3d x_target_2_;
	Vector3d x_dot_target_;

	double h1;
	double h2;

	unsigned long tick_;
	double play_time_;
	double hz_;
	double dt_;
	double control_start_time_;

	std::string control_mode_;
	bool is_mode_changed_;
	bool set_target_;

#ifdef USING_REAL_ROBOT
	FrankaModelInterface model_interface_;
#endif  // USING_REAL_ROBOT

	// for robot model construction
	Math::Vector3d com_position_[DOF];
	Vector3d joint_posision_[DOF];

	shared_ptr<Model> model_;
	unsigned int body_id_[DOF];
	Body body_[DOF];
	Joint joint_[DOF];

	Matrix<double, 6, 6> Kp_jacobian_;
	// HW5-1
	Vector7d target_q;
	Vector3d target_x;
	Vector3d target_x_save;
	Vector3d target_x_dot;
	Matrix3d target_ori;
	Vector3d x_dot;
	Vector3d ori_dot;
	Vector3d f_star;
	Vector3d m_star;
	Matrix3d Kp_, Kv_;
	Matrix7d Kp_joint_, Kv_joint_;
	Matrix6d Lamda;
	Vector6d control_input;
	Vector7d control_input_2;
	Matrix<double, 6, 7> j_t_dyn_cons_inv;

	//HW6-1
	Vector3d obs_x;
	Vector3d obs_dist;
	Vector3d f_star_obs;
	double k_obs;

	// file write
	string path;
	ofstream writeFile;
	bool write_;
	bool write_init_;

	bool is_export_started_;

	// Project
	bool check_slow;
	vector<Vector3d> robot_path1, robot_path2, robot_path3;
	double slow_start_time;
	vector<double> duration_list;
	double accum_duration;
	int path_index;
	double control_time;
	bool control_fisrt;

	bool check_planned;
	bool check_planned1;
	int path_size1, path_size2, path_size3;
	std::vector<Vector3d> target_positions;
	std::vector<Vector3d> pointings;
	std::vector<double> norms;
	std::vector<double> durations;
	double control_start_time_update_;
	double control_start_time_update_before;
	int index;
	bool Isonce = true;
	Vector3d x_desired;
	Vector3d x_dot_desired;
	Matrix3d rot_desired;
	double max_velocity;
	bool print_once;

private:
	void printState();
	void findpath(vector<Vector3d>& robot_path1, vector<Vector3d>& robot_path2, vector<Vector3d>& robot_path3);
	void findDuration(vector<Vector3d>& robot_path, vector<double>& duration_list);
	//void moveJointPosition(const Vector7d &target_position, double duration);
	//void moveJointPositionTorque(const Vector7d &target_position, double duration);

public:
	void closeFile();
	void readData(const Vector7d& position, const Vector7d& velocity, const Vector7d& torque);
	void readData(const Vector7d& position, const Vector7d& velocity);
	const Vector7d& getDesiredPosition();
	const Vector7d& getDesiredTorque();

public:
#ifdef USING_REAL_ROBOT
	ArmController(double hz, franka::Model& model) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false), model_interface_(model)
	{
		initDimension();
	}
	void initPosition(franka::RobotState state);
	void updateTime(double dt);
#else  // USING_REAL_ROBOT
	ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
		initDimension(); initModel();
	}
	void initPosition();
#endif  // USING_REAL_ROBOT


	void setMode(const std::string& mode);
	void initDimension();
	void initModel();
	void compute();
};

#endif
