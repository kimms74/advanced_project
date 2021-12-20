#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>

FILE* fp1 = fopen("../max_vel1.txt", "r");
FILE* fp2 = fopen("../max_vel2.txt", "r");

void ArmController::compute()
{
	// ---------------------------------
	//
	// q_		: joint position
	// qdot_	: joint velocity
	// x_		: end-effector position 
	// j_		: end-effector basic jacobian
	// m_		: mass matrix
	//
	//-------------------------------------------------------------------

	// Kinematics and dynamics calculation
#ifdef USING_REAL_ROBOT
	Eigen::Affine3d transform = model_interface_.getTransform(franka::Frame::kEndEffector, q_);
	Eigen::Affine3d transform2 = model_interface_.getTransform(franka::Frame::kJoint4, q_);
	x_ = transform.translation();
	rotation_ = transform.linear();
	j_ = model_interface_.getJacobianMatrix(franka::Frame::kEndEffector, q_);
	m_ = model_interface_.getMassMatrix(q_);
	// For CLIK
	Eigen::Affine3d transform_from_q_desired = model_interface_.getTransform(franka::Frame::kEndEffector,
		q_desired_);
	x_from_q_desired_ = transform_from_q_desired.translation();
	rotation_from_q_desired_ = transform_from_q_desired.linear();
	j_from_q_desired_ = model_interface_.getJacobianMatrix(franka::Frame::kEndEffector, q_desired_);
	g_.setZero();
	x_dot_ = j_ * qdot_;
#else  // USING_REAL_ROBOT
	dt_ = 1 / hz_;
	q_temp_ = q_;
	qdot_temp_ = qdot_;
	j_temp_.setZero();
	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], true);
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	Matrix3d body_to_ee_rotation;  // rotation at EE
	body_to_ee_rotation.setIdentity();
	body_to_ee_rotation(1, 1) = -1;
	body_to_ee_rotation(2, 2) = -1;
	rotation_ = rotation_ * body_to_ee_rotation; // To Match RBDL model and CoppeliaSim model
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, true);
	for (int i = 0; i < 2; i++)
		j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);
	g_ = g_temp_;
	m_ = m_temp_;
	// Kinematics calculation using q_desired_ (CLIK)
	j_temp_.setZero();
	x_from_q_desired_ = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	rotation_from_q_desired_ = CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose();
	rotation_from_q_desired_ = rotation_from_q_desired_ * body_to_ee_rotation; // To Match RBDL model and CoppeliaSim model
	CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, false);
	for (int i = 0; i < 2; i++)
		j_from_q_desired_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
#endif  // USING_REAL_ROBOT


	// Control
	if (is_mode_changed_)
	{
		is_mode_changed_ = false;
		write_ = false;
		write_init_ = true;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();
		q_desired_ = q_;

		x_init_ = x_;
		x_init_2_ = x_2;
		x_dot_init_ = x_dot_.block<3, 1>(0, 0);
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
		check_planned = true;
		check_slow = false;
		path_index = 0;
		accum_duration = 0;
		control_fisrt = true;

		check_planned = true;
		check_planned1 = true;
		print_once = true;

		
		target_positions.clear();
		pointings.clear();
		norms.clear();
		durations.clear();
	}

	if (control_mode_ == "joint_ctrl_home")
	{
		double duration = 5.0;
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 4, 0.0, M_PI / 2, 0;
		q_desired_ = cubicVector<7>(play_time_, control_start_time_, control_start_time_ + duration, q_init_, target_position, qdot_init_, qdot_target_);
	}
	else if (control_mode_ == "project_home") {
		double duration = 3.0;
		Vector7d target_position;
		target_position << 0.0, M_PI / 6.0, 0.0, -M_PI * 2.0 / 3.0, 0.0, M_PI * 5.0 / 6.0, 0;
		q_desired_ = cubicVector<7>(play_time_, control_start_time_, control_start_time_ + duration, q_init_, target_position, qdot_init_, qdot_target_);
	}
	else if (control_mode_ == "project_init") {
		double duration = 3.0;
		Vector3d project_translation;
		project_translation << 0.58, 0, 0.15;

		Vector3d target_x_position;
		target_x_position << 0.08, -0.18, 0;
		target_x_position += project_translation;
		
		Matrix3d target_rotation;
		target_rotation.block<3, 1>(0, 0) << 1.0, 0.0, 0.0;
		target_rotation.block<3, 1>(0, 1) << 0.0, -1.0, 0.0;
		target_rotation.block<3, 1>(0, 2) << 0.0, 0.0, -1.0;

		Matrix3d rot_to_desired;
		rot_to_desired = rotation_init_.transpose() * target_rotation;

		AngleAxisd angle_axis;
		angle_axis.fromRotationMatrix(rot_to_desired);

		Vector3d x_desired;
		Vector3d x_dot_desired;
		Matrix3d rot_desired;

		for (int i = 0; i < 3; i++)
		{
			x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, x_init_(i), target_x_position(i), 0.0, 0.0);
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, x_init_(i), target_x_position(i), 0.0, 0.0, hz_);
		}

		double angle_desired;
		double angle_dot_desired;

		angle_desired = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis.angle(), 0.0, 0.0);
		angle_dot_desired = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis.angle(), 0.0, 0.0, hz_);

		Vector3d angular_velocity_desired;
		Matrix3d rot_from_angle_axis;

		rot_from_angle_axis = angle_axis.matrix();

		rot_desired = rotation_init_ * rot_from_angle_axis;
		angular_velocity_desired = angle_dot_desired * angle_axis.axis();

		Vector6d v_w_desired;
		Matrix<double, 7, 6> j_pseudo_inverse;

		v_w_desired << x_dot_desired, angular_velocity_desired;

		j_pseudo_inverse = (j_.transpose()) * ((j_ * (j_.transpose())).inverse());

		Vector6d x_rot_error;
		Vector3d x_error;

		Vector3d rot_error;
		Vector6d K_p_dot_x_rot_error;

		Vector7d q_error;

		q_error = q_desired_ - q_;
		x_error = x_desired - x_from_q_desired_;
		rot_error = DyrosMath::getPhi(rot_desired, rotation_from_q_desired_);

		x_rot_error << x_error, rot_error;

		Vector6d K_p;
		Vector3d K_p_x;
		Vector3d K_p_rot;

		K_p_x << 5.0, 5.0, 5.0;
		K_p_rot << 5.0, 5.0, 5.0;
		K_p << K_p_x, K_p_rot;

		for (int i = 0; i < 6; i++)
		{
			K_p_dot_x_rot_error(i) = K_p(i) * x_rot_error(i);
		}
		qdot_ = j_pseudo_inverse * (v_w_desired + K_p_dot_x_rot_error);
		q_desired_ = q_desired_ + qdot_ * dt_;

	}
	//else if (control_mode_ == "project_test") {
	//	
	//	Vector3d project_translation; //������Ʈ ����� ��ǥ��� �̵�
	//	project_translation << 0.58, 0, 0.15;

	//	Vector3d target_3;
	//	target_3 << -0.08, 0.18, 0;
	//	target_3 += project_translation;

	//	Vector3d pointing;
	//	pointing = target_3 - x_init_.head(3);
	//	std::cout << "pointing : " << pointing << std::endl;
	//	double norm = pointing.norm(); //distance �̱⵵ ��

	//	double duration = norm / 0.3;


	//	Matrix3d target_rotation;
	//	target_rotation.block<3, 1>(0, 0) << 1.0, 0.0, 0.0;
	//	target_rotation.block<3, 1>(0, 1) << 0.0, -1.0, 0.0;
	//	target_rotation.block<3, 1>(0, 2) << 0.0, 0.0, -1.0;

	//	Matrix3d rot_to_desired;
	//	rot_to_desired = rotation_init_.transpose() * target_rotation;

	//	AngleAxisd angle_axis;
	//	angle_axis.fromRotationMatrix(rot_to_desired);

	//	Vector3d x_desired;
	//	Vector3d x_dot_desired;
	//	Matrix3d rot_desired;

	//	for (int i = 0; i < 3; i++)
	//	{
	//		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, x_init_(i), target_3(i), 0.0, 0.0);
	//		x_dot_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, x_init_(i), target_3(i), 0.0, 0.0, hz_);
	//	}

	//	double angle_desired;
	//	double angle_dot_desired;

	//	angle_desired = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis.angle(), 0.0, 0.0);
	//	angle_dot_desired = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis.angle(), 0.0, 0.0, hz_);

	//	Vector3d angular_velocity_desired;
	//	Matrix3d rot_from_angle_axis;

	//	rot_from_angle_axis = angle_axis.matrix();

	//	rot_desired = rotation_init_ * rot_from_angle_axis;
	//	angular_velocity_desired = angle_dot_desired * angle_axis.axis();

	//	Vector6d v_w_desired;
	//	Matrix<double, 7, 6> j_pseudo_inverse;

	//	v_w_desired << x_dot_desired, angular_velocity_desired;

	//	j_pseudo_inverse = (j_.transpose()) * ((j_ * (j_.transpose())).inverse());

	//	Vector6d x_rot_error;
	//	Vector3d x_error;

	//	Vector3d rot_error;
	//	Vector6d K_p_dot_x_rot_error;

	//	Vector7d q_error;

	//	q_error = q_desired_ - q_;
	//	x_error = x_desired - x_from_q_desired_;
	//	rot_error = DyrosMath::getPhi(rot_desired, rotation_from_q_desired_);

	//	x_rot_error << x_error, rot_error;

	//	Vector6d K_p;
	//	Vector3d K_p_x;
	//	Vector3d K_p_rot;

	//	K_p_x << 5.0, 5.0, 5.0;
	//	K_p_rot << 5.0, 5.0, 5.0;
	//	K_p << K_p_x, K_p_rot;

	//	for (int i = 0; i < 6; i++)
	//	{
	//		K_p_dot_x_rot_error(i) = K_p(i) * x_rot_error(i);
	//	}
	//	qdot_ = j_pseudo_inverse * (v_w_desired + K_p_dot_x_rot_error);
	//	q_desired_ = q_desired_ + qdot_ / hz_;

	//}
	//else if (control_mode_ == "project_path1") {
	//	
	//	//AStar::CoordinateListf path;

	//	Vector3d project_translation; //������Ʈ ����� ��ǥ��� �̵�
	//	project_translation << 0.58, 0, 0.15;

	//	double max_velocity = 0.3;

	//	Vector3d target_0;
	//	target_0 << 0.08, -0.18, 0;
	//	target_0 += project_translation;

	//	Vector3d target_1; 
	//	target_1 << 0.06, -0.09666, 0;
	//	target_1 += project_translation;

	//	Vector3d pointing_0_1;
	//	pointing_0_1 = target_1 - target_0;
	//	double norm_0_1 = pointing_0_1.norm(); //distance �̱⵵ ��
	//	double duration_0_1 = norm_0_1 / max_velocity;

	//	Vector3d target_2;
	//	target_2 << 0.04, 0.02666, 0;
	//	target_2 += project_translation;

	//	Vector3d pointing_1_2;
	//	pointing_1_2 = target_2 - target_1;
	//	double norm_1_2 = pointing_1_2.norm(); //distance �̱⵵ ��
	//	double duration_1_2 = norm_1_2 / max_velocity;

	//	pointing_1_2(0) = pointing_1_2(0) / norm_1_2 * max_velocity;
	//	pointing_1_2(1) = pointing_1_2(1) / norm_1_2 * max_velocity;
	//	pointing_1_2(2) = pointing_1_2(2) / norm_1_2 * max_velocity;

	//	Vector3d target_3;
	//	target_3 << -0.08, 0.18, 0;
	//	target_3 += project_translation;

	//	Vector3d pointing_2_3;
	//	pointing_2_3 = target_3 - target_2;
	//	double norm_2_3 = pointing_2_3.norm(); //distance �̱⵵ ��
	//	double duration_2_3 = norm_2_3 / max_velocity;

	//	pointing_2_3(0) = pointing_2_3(0) / norm_2_3 * max_velocity;;
	//	pointing_2_3(1) = pointing_2_3(0) / norm_2_3 * max_velocity;;
	//	pointing_2_3(2) = pointing_2_3(0) / norm_2_3 * max_velocity;;

	//	Matrix3d target_rotation;
	//	target_rotation.block<3, 1>(0, 0) << 1.0, 0.0, 0.0;
	//	target_rotation.block<3, 1>(0, 1) << 0.0, -1.0, 0.0;
	//	target_rotation.block<3, 1>(0, 2) << 0.0, 0.0, -1.0;

	//	Matrix3d rot_to_desired;
	//	rot_to_desired = rotation_init_.transpose() * target_rotation;

	//	AngleAxisd angle_axis;
	//	angle_axis.fromRotationMatrix(rot_to_desired);

	//	Vector3d x_desired;
	//	Vector3d x_dot_desired;
	//	Matrix3d rot_desired;

	//	if (play_time_ < control_start_time_ + duration_0_1) {
	//		for (int i = 0; i < 3; i++)
	//		{
	//			x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration_0_1, target_0(i), target_1(i), 0.0, pointing_1_2(i));
	//			x_dot_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration_0_1, target_0(i), target_1(i), 0.0, pointing_1_2(i), hz_);
	//		}
	//	}
	//	else if (play_time_ < control_start_time_ + duration_1_2) {
	//		for (int i = 0; i < 3; i++)
	//		{
	//			x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration_1_2, target_1(i), target_2(i), pointing_1_2(i), pointing_2_3(i));
	//			x_dot_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration_1_2, target_1(i), target_2(i), pointing_1_2(i), pointing_2_3(i), hz_);
	//		}
	//	}
	//	else if (play_time_ < control_start_time_ + duration_2_3) {
	//		for (int i = 0; i < 3; i++)
	//		{
	//			x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration_2_3, target_2(i), target_3(i), pointing_2_3(i), 0.0);
	//			x_dot_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration_2_3, target_2(i), target_3(i), pointing_2_3(i), 0.0, hz_);
	//		}
	//	}
	//	

	//	double angle_desired;
	//	double angle_dot_desired;

	//	angle_desired = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration_0_1 + duration_1_2 + duration_2_3, 0, angle_axis.angle(), 0.0, 0.0);
	//	angle_dot_desired = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration_0_1 + duration_1_2 + duration_2_3, 0, angle_axis.angle(), 0.0, 0.0, hz_);

	//	Vector3d angular_velocity_desired;
	//	Matrix3d rot_from_angle_axis;

	//	rot_from_angle_axis = angle_axis.matrix();

	//	rot_desired = rotation_init_ * rot_from_angle_axis;
	//	angular_velocity_desired = angle_dot_desired * angle_axis.axis();

	//	Vector6d v_w_desired;
	//	Matrix<double, 7, 6> j_pseudo_inverse;

	//	v_w_desired << x_dot_desired, angular_velocity_desired;

	//	j_pseudo_inverse = (j_.transpose()) * ((j_ * (j_.transpose())).inverse());

	//	Vector6d x_rot_error;
	//	Vector3d x_error;

	//	Vector3d rot_error;
	//	Vector6d K_p_dot_x_rot_error;

	//	Vector7d q_error;

	//	q_error = q_desired_ - q_;
	//	x_error = x_desired - x_from_q_desired_;
	//	rot_error = DyrosMath::getPhi(rot_desired, rotation_from_q_desired_);

	//	x_rot_error << x_error, rot_error;

	//	Vector6d K_p;
	//	Vector3d K_p_x;
	//	Vector3d K_p_rot;

	//	K_p_x << 1.0, 1.0, 1.0;
	//	K_p_rot << 5.0, 5.0, 5.0;
	//	K_p << K_p_x, K_p_rot;

	//	for (int i = 0; i < 6; i++)
	//	{
	//		K_p_dot_x_rot_error(i) = K_p(i) * x_rot_error(i);
	//	}
	//	qdot_ = j_pseudo_inverse * (v_w_desired + K_p_dot_x_rot_error);
	//	q_desired_ = q_desired_ + qdot_ / hz_;

	//}
	else if (control_mode_ == "joint_ctrl_init")
	{
		double duration = 5.0;
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0;
		q_desired_ = cubicVector<7>(play_time_, control_start_time_, control_start_time_ + duration, q_init_, target_position, qdot_init_, qdot_target_);
	}
	else if (control_mode_ == "CLIK")
	{
		x_target_ << 0.25, 0.28, 0.65;
		rotation_target_ << 0.7071, 0.7071, 0, 0.7071, -0.7071, 0, 0, 0, -1;

		// q_desired_ = 
	}
	else if (control_mode_ == "Dynamics")
	{
		q_target_ << 0.0, 0.0, 0.0, -60 * DEG2RAD, 0.0, 90 * DEG2RAD, 0.0;

		// torque_desired_ = 
	}

	else if (control_mode_ == "project_function") {
	
		if (write_init_ == true) {
			path = "../data/projecyt.txt";
			writeFile.open(path);
			write_init_ = false;
			write_ = true;
		}
		double duration = 5.0;
		if (play_time_ > control_start_time_ + duration)
		{
			write_ = false;
			closeFile();
		}

		//AStar::CoordinateListf robot_path;
		////target_position[0] << 0.58, 0, 0.15;
		if (check_planned == true) {
			
			findpath(robot_path1, robot_path2, robot_path3);
			robot_path2.erase(robot_path2.begin());
			robot_path3.erase(robot_path3.begin());
			target_positions.insert(target_positions.end(), robot_path1.begin(), robot_path1.end());
			target_positions.insert(target_positions.end(), robot_path2.begin(), robot_path2.end());
			target_positions.insert(target_positions.end(), robot_path3.begin(), robot_path3.end());

			check_planned = false;
		}

		// max_velocity = 0.3;
		fscanf(fp1, "%lf", &max_velocity);
		// std::cout << "file read, max_velocity : " << max_velocity << std::endl;

		Matrix3d target_rotation;
		target_rotation.block<3, 1>(0, 0) << 1.0, 0.0, 0.0;
		target_rotation.block<3, 1>(0, 1) << 0.0, -1.0, 0.0;
		target_rotation.block<3, 1>(0, 2) << 0.0, 0.0, -1.0;

		Matrix3d rot_to_desired;
		rot_to_desired = rotation_init_.transpose() * rotation_init_;

		AngleAxisd angle_axis;
		angle_axis.fromRotationMatrix(rot_to_desired);

		path_size1 = robot_path1.size();
		path_size2 = robot_path2.size();
		path_size3 = robot_path3.size();

		// for (int i = 0; i < target_positions.size() ; i++){
		// std::cout << target_positions[i].transpose() << std::endl;
		// }

		if (check_planned1 == true) {
			for (int i = 0; i < path_size1 + path_size2 + path_size3 - 1; i++) {
				/*if (i == path_size1 + path_size2 + path_size3 - 2) {
				pointings[i] = target_positions[path_size1 + path_size2 + path_size3 - 1] - target_positions[path_size1 + path_size2 + path_size3 - 2];
				}
				else {
				pointings[i] = target_positions[i + 1] - target_positions[i];
				}*/
				pointings.push_back(target_positions[i + 1] - target_positions[i]);
				norms.push_back(pointings[i].norm());
				durations.push_back(norms[i] / max_velocity);
			}
			/*double duration;
			for (int i = 0; i < path_size1 + path_size2 + path_size3 - 1; i++) {
			duration += durations[i];
			}*/

			for (int i = 0; i < path_size1 + path_size2 + path_size3 - 1; i++) {
				pointings[i] = pointings[i] / norms[i] * max_velocity;
			}
			Vector3d zero_vec;
			zero_vec << 0.0, 0.0, 0.0;
			pointings.erase(pointings.begin());
			pointings.insert(pointings.begin(), zero_vec);
			pointings.insert(pointings.end(), zero_vec);
			check_planned1 = false;
		}

		/*control_start_time_update_ = control_start_time_;
		control_start_time_update_before = control_start_time_;*/

		if (play_time_ == control_start_time_)
		{
			index = 0;
			control_start_time_update_ = control_start_time_;
			control_start_time_update_before = control_start_time_;
		}
		if (Isonce == true) {
			control_start_time_update_ = control_start_time_update_ + durations[index];
			Isonce = false;
		}
		if (play_time_ < control_start_time_update_) {

			for (int j = 0; j < 3; j++)
			{
				////std::cout << "fucking" << std::endl;
				//std::cout << "target_positions[index] \n" << target_positions[index] << std::endl << std::endl;
				//std::cout << "target_positions[index+1] \n" << target_positions[index + 1] << std::endl << std::endl;
				//std::cout << "pointings[index] \n" << pointings[index] << std::endl << std::endl;
				//std::cout << "pointings[index+1] \n" << pointings[index + 1] << std::endl << std::endl;
				x_desired(j) = DyrosMath::cubic(play_time_, control_start_time_update_before, control_start_time_update_, target_positions[index](j), target_positions[index + 1](j), pointings[index](j), pointings[index + 1](j));
				x_dot_desired(j) = DyrosMath::cubicDot(play_time_, control_start_time_update_before, control_start_time_update_, target_positions[index](j), target_positions[index + 1](j), pointings[index](j), pointings[index + 1](j), hz_);
			}
		}
		else {
			Isonce = true;
			control_start_time_update_before = control_start_time_update_;
			if (index < path_size1 + path_size2 + path_size3 - 2) {
				index = index + 1;
			}
			else {
				if(print_once == true)
				{
					std::cout << play_time_ - control_start_time_ << std::endl;
					print_once = false;
				}

				Isonce = false;
				// std::cout << x_desired.transpose() << "   " << x_dot_desired.transpose() << std::endl;
				// std::cout << "x desired: " << x_desired.transpose() << "    " << "xdot desired: " <<x_dot_desired.transpose() << std::endl;
				x_desired << 0.5, -0.18, 0.15;
				x_dot_desired << 0.0, 0.0, 0.0;
			}
		}

		double angle_desired;
		double angle_dot_desired;

		angle_desired = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + 2, 0, angle_axis.angle(), 0.0, 0.0);
		angle_dot_desired = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + 2, 0, angle_axis.angle(), 0.0, 0.0, hz_);

		Vector3d angular_velocity_desired;
		Matrix3d rot_from_angle_axis;

		rot_from_angle_axis = angle_axis.matrix();

		rot_desired = rotation_init_ * rot_from_angle_axis;
		angular_velocity_desired = angle_dot_desired * angle_axis.axis();

		Vector6d v_w_desired;
		Matrix<double, 7, 6> j_pseudo_inverse;

		v_w_desired << x_dot_desired, angular_velocity_desired;

		
		Vector6d x_rot_error;
		Vector3d x_error;

		Vector3d rot_error;
		Vector6d K_p_dot_x_rot_error;

		for (int k = 0; k < 1; k++) {
	#ifdef USING_REAL_ROBOT
			Eigen::Affine3d transform_from_q_desired = model_interface_.getTransform(franka::Frame::kEndEffector,
			q_desired_);
			x_from_q_desired_ = transform_from_q_desired.translation();
			rotation_from_q_desired_ = transform_from_q_desired.linear();
			j_from_q_desired_ = model_interface_.getJacobianMatrix(franka::Frame::kEndEffector, q_desired_);	
	#else
			x_from_q_desired_ = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
			rotation_from_q_desired_ = CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose();
			rotation_from_q_desired_ = rotation_from_q_desired_ * body_to_ee_rotation; // To Match RBDL model and CoppeliaSim model
	#endif
			x_error = x_desired - x_from_q_desired_;
			rot_error = DyrosMath::getPhi(rot_desired, rotation_from_q_desired_);

			// j_pseudo_inverse = (j_from_q_desired_.transpose()) * ((j_from_q_desired_ * (j_from_q_desired_.transpose())).inverse());
			Matrix6d damping;
			// damping = EYE(6);
			damping << 1, 0, 0, 0, 0, 0,
						0, 1, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0,
						0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 1, 0,
						0, 0, 0, 0, 0, 1;	
			// damping << 0, 0, 0, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0,
			// 			0, 0, 1, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0;
			j_pseudo_inverse = (j_from_q_desired_.transpose()) * ((j_from_q_desired_ * (j_from_q_desired_.transpose())+0.002*damping).inverse());

		

			x_rot_error << x_error, rot_error;

			Vector6d K_p;
			Vector3d K_p_x;
			Vector3d K_p_rot;

			K_p_x <<1.0, 1.0, 1.0;
			K_p_rot <<1.0, 1.0, 1.0;
			K_p << K_p_x, K_p_rot;

			for (int i = 0; i < 6; i++)
			{
				K_p_dot_x_rot_error(i) = K_p(i) * x_rot_error(i);
			}
			if (k == 0) {
				qdot_ = j_pseudo_inverse * (v_w_desired + K_p_dot_x_rot_error);
			}
			else {
				qdot_ = j_pseudo_inverse * K_p_dot_x_rot_error;
			}
			q_desired_ = q_desired_ + qdot_ / hz_;
		}
		if (write_ == true) {
		writeFile << (play_time_ - control_start_time_) << "\t" <<  x_.transpose() << "\t" << x_dot_.head(3).transpose() << "\n";
		}
	}

	/////////////////////////////////////////////////////////////////
	else if (control_mode_ == "project_function2") {
	
		if (write_init_ == true) {
			path = "../data/projecyt.txt";
			writeFile.open(path);
			write_init_ = false;
			write_ = true;
		}
		double duration = 5.0;
		if (play_time_ > control_start_time_ + duration)
		{
			write_ = false;
			closeFile();
		}

		//AStar::CoordinateListf robot_path;
		////target_position[0] << 0.58, 0, 0.15;
		if (check_planned == true) {
			findpath(robot_path1, robot_path2, robot_path3);
			robot_path2.erase(robot_path2.begin());
			robot_path3.erase(robot_path3.begin());
			target_positions.insert(target_positions.end(), robot_path1.begin(), robot_path1.end());
			target_positions.insert(target_positions.end(), robot_path2.begin(), robot_path2.end());
			target_positions.insert(target_positions.end(), robot_path3.begin(), robot_path3.end());

			check_planned = false;
		}

		// max_velocity = 0.3;
		fscanf(fp2, "%lf", &max_velocity);
		// std::cout << "file read, max_velocity : " << max_velocity << std::endl;

		Matrix3d target_rotation;
		target_rotation.block<3, 1>(0, 0) << 1.0, 0.0, 0.0;
		target_rotation.block<3, 1>(0, 1) << 0.0, -1.0, 0.0;
		target_rotation.block<3, 1>(0, 2) << 0.0, 0.0, -1.0;

		Matrix3d rot_to_desired;
		rot_to_desired = rotation_init_.transpose() * rotation_init_;

		AngleAxisd angle_axis;
		angle_axis.fromRotationMatrix(rot_to_desired);

		path_size1 = robot_path1.size();
		path_size2 = robot_path2.size();
		path_size3 = robot_path3.size();

		// for (int i = 0; i < target_positions.size() ; i++){
		// std::cout << target_positions[i].transpose() << std::endl;
		// }

		if (check_planned1 == true) {
			for (int i = 0; i < path_size1 + path_size2 + path_size3 - 1; i++) {
				/*if (i == path_size1 + path_size2 + path_size3 - 2) {
				pointings[i] = target_positions[path_size1 + path_size2 + path_size3 - 1] - target_positions[path_size1 + path_size2 + path_size3 - 2];
				}
				else {
				pointings[i] = target_positions[i + 1] - target_positions[i];
				}*/
				pointings.push_back(target_positions[i + 1] - target_positions[i]);
				norms.push_back(pointings[i].norm());
				durations.push_back(norms[i] / max_velocity); //duration gain?
			}
			/*double duration;
			for (int i = 0; i < path_size1 + path_size2 + path_size3 - 1; i++) {
			duration += durations[i];
			}*/

			for (int i = 0; i < path_size1 + path_size2 + path_size3 - 1; i++) {
				pointings[i] = pointings[i] / norms[i] * max_velocity;
			}
			Vector3d zero_vec;
			zero_vec << 0.0, 0.0, 0.0;
			pointings.erase(pointings.begin());
			pointings.insert(pointings.begin(), zero_vec);
			pointings.insert(pointings.end(), zero_vec);
			pointings[path_size1-1] = zero_vec;
			pointings[path_size1+path_size2-1] = zero_vec;
			check_planned1 = false;
		}

		/*control_start_time_update_ = control_start_time_;
		control_start_time_update_before = control_start_time_;*/

		if (play_time_ == control_start_time_)
		{
			index = 0;
			control_start_time_update_ = control_start_time_;
			control_start_time_update_before = control_start_time_;
		}
		if (Isonce == true) {
			control_start_time_update_ = control_start_time_update_ + durations[index];
			Isonce = false;
		}
		if (play_time_ < control_start_time_update_) {

			for (int j = 0; j < 3; j++)
			{
				////std::cout << "fucking" << std::endl;
				//std::cout << "target_positions[index] \n" << target_positions[index] << std::endl << std::endl;
				//std::cout << "target_positions[index+1] \n" << target_positions[index + 1] << std::endl << std::endl;
				//std::cout << "pointings[index] \n" << pointings[index] << std::endl << std::endl;
				//std::cout << "pointings[index+1] \n" << pointings[index + 1] << std::endl << std::endl;
				x_desired(j) = DyrosMath::cubic(play_time_, control_start_time_update_before, control_start_time_update_, target_positions[index](j), target_positions[index + 1](j), pointings[index](j), pointings[index + 1](j));
				x_dot_desired(j) = DyrosMath::cubicDot(play_time_, control_start_time_update_before, control_start_time_update_, target_positions[index](j), target_positions[index + 1](j), pointings[index](j), pointings[index + 1](j), hz_);
			}
		}
		else {
			Isonce = true;
			control_start_time_update_before = control_start_time_update_;
			if (index < path_size1 + path_size2 + path_size3 - 2) {
				index = index + 1;
			}
			else {
				if(print_once == true)
				{
					std::cout << play_time_ - control_start_time_ << std::endl;
					print_once = false;
				}

				Isonce = false;
				// std::cout << x_desired.transpose() << "   " << x_dot_desired.transpose() << std::endl;
				// std::cout << "x desired: " << x_desired.transpose() << "    " << "xdot desired: " <<x_dot_desired.transpose() << std::endl;
				x_desired << 0.5, -0.18, 0.15;
				x_dot_desired << 0.0, 0.0, 0.0;
			}
		}

		double angle_desired;
		double angle_dot_desired;

		angle_desired = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + 2, 0, angle_axis.angle(), 0.0, 0.0);
		angle_dot_desired = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + 2, 0, angle_axis.angle(), 0.0, 0.0, hz_);

		Vector3d angular_velocity_desired;
		Matrix3d rot_from_angle_axis;

		rot_from_angle_axis = angle_axis.matrix();

		rot_desired = rotation_init_ * rot_from_angle_axis;
		angular_velocity_desired = angle_dot_desired * angle_axis.axis();

		Vector6d v_w_desired;
		Matrix<double, 7, 6> j_pseudo_inverse;

		v_w_desired << x_dot_desired, angular_velocity_desired;

		
		Vector6d x_rot_error;
		Vector3d x_error;

		Vector3d rot_error;
		Vector6d K_p_dot_x_rot_error;

		for (int k = 0; k < 1; k++) {
	#ifdef USING_REAL_ROBOT
			Eigen::Affine3d transform_from_q_desired = model_interface_.getTransform(franka::Frame::kEndEffector,
			q_desired_);
			x_from_q_desired_ = transform_from_q_desired.translation();
			rotation_from_q_desired_ = transform_from_q_desired.linear();
			j_from_q_desired_ = model_interface_.getJacobianMatrix(franka::Frame::kEndEffector, q_desired_);	
	#else
			x_from_q_desired_ = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
			rotation_from_q_desired_ = CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose();
			rotation_from_q_desired_ = rotation_from_q_desired_ * body_to_ee_rotation; // To Match RBDL model and CoppeliaSim model
	#endif
			x_error = x_desired - x_from_q_desired_;
			// x_error = x_desired - x_;
			rot_error = DyrosMath::getPhi(rot_desired, rotation_from_q_desired_);

			// j_pseudo_inverse = (j_from_q_desired_.transpose()) * ((j_from_q_desired_ * (j_from_q_desired_.transpose())).inverse());
			Matrix6d damping;
			// damping = EYE(6);
			damping << 1, 0, 0, 0, 0, 0,
						0, 1, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0,
						0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 1, 0,
						0, 0, 0, 0, 0, 1;	
			// damping << 0, 0, 0, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0,
			// 			0, 0, 1, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0,
			// 			0, 0, 0, 0, 0, 0;				
			j_pseudo_inverse = (j_from_q_desired_.transpose()) * ((j_from_q_desired_ * (j_from_q_desired_.transpose()) + 0.003*damping).inverse());
		

			x_rot_error << x_error, rot_error;

			Vector6d K_p;
			Vector3d K_p_x;
			Vector3d K_p_rot;

			K_p_x <<1.0, 1.0, 1.0;
			K_p_rot <<1.0, 1.0, 1.0;
			K_p << K_p_x, K_p_rot;

			for (int i = 0; i < 6; i++)
			{
				K_p_dot_x_rot_error(i) = K_p(i) * x_rot_error(i);
			}
			if (k == 0) {
				qdot_ = j_pseudo_inverse * (v_w_desired + K_p_dot_x_rot_error);
			}
			else {
				qdot_ = j_pseudo_inverse * K_p_dot_x_rot_error;
			}
			q_desired_ = q_desired_ + qdot_ / hz_;
		}
		if (write_ == true) {
		writeFile << (play_time_ - control_start_time_) << "\t" <<  x_.transpose() << "\t" << x_dot_.head(3).transpose() << "\n";
		}
	}

	// else if (control_mode_ == "astar")
	// {
	// 	if (check_planned == true) {
	// 		findpath(robot_path1, robot_path2, robot_path3);
	// 		for (auto& coordinate : robot_path1) {
	// 			std::cout << coordinate.transpose() << "\n";
	// 		}

	// 		std::cout << "  " << std::endl;
	// 		for (auto& coordinate : robot_path2) {
	// 			std::cout << coordinate.transpose() << "\n";
	// 		}

	// 		std::cout << "  " << std::endl;
	// 		for (auto& coordinate : robot_path3) {
	// 			std::cout << coordinate.transpose() << "\n";
	// 		}
			
	// 		check_planned = false;
	// 	}
	// 	double duration = 5.0;



	// 	if (write_init_ == true) {
	// 		path = "../data/6_1.txt";
	// 		writeFile.open(path);
	// 		write_init_ = false;
	// 		write_ = true;
	// 	}
	// 	if (play_time_ > control_start_time_ + duration)
	// 	{
	// 		write_ = false;
	// 		closeFile();
	// 	}
	// 	findDuration(robot_path1, duration_list);

	// 	double max_velocity = 0.3;

	// 	if (play_time_ - control_start_time_ >= accum_duration)
	// 	{
	// 		std::cout << (play_time_ - control_start_time_) << " " << accum_duration << std::endl;
	// 		if (path_index != (robot_path1.size()-1))
	// 		{
	// 			control_time = play_time_;
	// 			accum_duration += duration_list[path_index];
	// 			if (control_fisrt == true)
	// 			{
	// 				control_fisrt = false;
	// 				std::cout << robot_path1[path_index].transpose() << std::endl;
	// 			}
	// 			else
	// 			{
	// 				path_index++;
	// 				std::cout << robot_path1[path_index].transpose() << std::endl;
	// 			}
	// 		}
	// 	}

	// 	x_cubic_ = cubicVector<3>(play_time_, control_time + accum_duration, control_time + accum_duration + duration_list[path_index], robot_path1[path_index], robot_path1[path_index + 1], x_dot_init_, x_dot_target_);
	// 	x_cubic_dot_1_ = cubicDotVector<3>(play_time_, control_time + accum_duration, control_time + accum_duration + duration_list[path_index], robot_path1[path_index], robot_path1[path_index + 1], x_dot_init_, x_dot_target_, hz_);

	// 	//double vel1 = cubic(play_time_, control_start_time_, control_start_time_ + max_velocity, 0, max_velocity, 0.0, 0.0);
	// 	//if ((robot_path1[1] - x_cubic_).norm() <= (pow(max_velocity,2)/2))
	// 	//{
	// 	//	if (check_slow == false)
	// 	//	{
	// 	//		slow_start_time = play_time_;
	// 	//		check_slow = true;
	// 	//	}
	// 	//	vel1 = cubic(play_time_, slow_start_time, slow_start_time + max_velocity, max_velocity, 0, 0.0, 0.0);
	// 	//}

	// 	//double vel1_norm = (robot_path1[1] - robot_path1[0]).norm();
	// 	//Vector3d dir1 = (robot_path1[1] - robot_path1[0]) / vel1_norm;
	// 	//x_cubic_dot_1_ = dir1 * vel1;
	// 	//x_cubic_ = x_cubic_ + x_cubic_dot_1_ * dt_;

	// 	x_target_ << 0.25, 0.28, 0.65;
	// 	rotation_target_ << 0.7071, 0.7071, 0, 0.7071, -0.7071, 0, 0, 0, -1;
	// 	x_dot_target_ << 0.0, 0.0, 0.0;


	// 	Matrix3d rotation_init_to_target, rotation_from_angle_axis, rotation_desired;
	// 	AngleAxisd angle_axis_target, angle_axis_desired;

	// 	//rotation_init_to_target = rotation_init_.transpose() * rotation_target_;
	// 	rotation_init_to_target = rotation_init_.transpose() * rotation_init_;
	// 	angle_axis_target.fromRotationMatrix(rotation_init_to_target);

	// 	double angle_desired, angle_dot_desired;

	// 	Vector6d x_rot_error;
	// 	Vector7d qdot_desired;
	// 	Matrix6d Kp;

	// 	//x_cubic_ = cubicVector<3>(play_time_, control_start_time_, control_start_time_ + duration, x_init_, x_target_, x_dot_init_, x_dot_target_);
	// 	//x_cubic_dot_1_ = cubicDotVector<3>(play_time_, control_start_time_, control_start_time_ + duration, x_init_, x_target_, x_dot_init_, x_dot_target_, hz_);

	// 	angle_desired = cubic(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis_target.angle(), 0.0, 0.0);
	// 	angle_dot_desired = cubicDot(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis_target.angle(), 0.0, 0.0, hz_);
	// 	x_cubic_dot_2_ = angle_dot_desired * angle_axis_target.axis();

	// 	x_cubic_dot_ << x_cubic_dot_1_, x_cubic_dot_2_;

	// 	angle_axis_desired = AngleAxisd(angle_desired, angle_axis_target.axis());
	// 	rotation_from_angle_axis = angle_axis_desired.toRotationMatrix();
	// 	rotation_desired = rotation_init_ * rotation_from_angle_axis;


	// 	j_inverse_ = (j_from_q_desired_.transpose()) * ((j_from_q_desired_ * (j_from_q_desired_.transpose())).inverse());

	// 	x_rot_error.head(3) = x_cubic_ - x_from_q_desired_;
	// 	x_rot_error.tail(3) = getPhi(rotation_desired, rotation_);

	// 	Kp.block<3, 3>(0, 0) = EYE(3) * 1.0;
	// 	Kp.block<3, 3>(3, 3) = EYE(3) * 2.0;

	// 	qdot_desired = j_inverse_ * (x_cubic_dot_ + Kp * x_rot_error);
	// 	q_desired_ = q_desired_ + qdot_desired * dt_;

	// 	if (write_ == true) {
	// 		writeFile << (play_time_ - control_start_time_) << "\t" << x_target_.transpose() << "\t" << x_.transpose() << "\t" << x_rot_error.tail(3).transpose() << "\n";
	// 	}
	// }

	else
	{
		torque_desired_.setZero();
	}

	//printState();

	tick_++;

#ifndef USING_REAL_ROBOT
	play_time_ = tick_ / hz_;	// second
#endif
}


void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 20.)
	{
		DBG_CNT = 0;

		cout << "q desired:\t";
		cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		cout << "q cubic:\t";
		cout << std::fixed << std::setprecision(3) << q_cubic_.transpose() << endl;
		cout << "q now    :\t";
		cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		cout << "x desired:\t";
		cout << x_cubic_.transpose() << endl;
		cout << "x        :\t";
		cout << x_.transpose() << endl;
		cout << "torque desired:\t";
		cout << torque_desired_.transpose() << endl;
		// cout << "rotation init:\t";
		// cout << rotation_init_.transpose() << endl;
		// cout << "rotation        :\t";
		// cout << rotation_.transpose() << endl;
	}

	if (play_time_ == control_start_time_ + 3)
	{
		cout << "---------------------------------------------------------------------" << endl;
		cout << "                     control time finished                           " << endl;
		cout << "---------------------------------------------------------------------" << endl;
	}
}
void ArmController::findpath(vector<Vector3d>& robot_path1, vector<Vector3d>& robot_path2, vector<Vector3d>& robot_path3)
{
	int grid_multiplier = 2;    //make grid size up
	int height = 20;
	int width = 40;

	//distance from centor of map using panda coordinate & diameter
	// AStar::Obs2i real_obs1{ 0.02,0.0,0.08 };
	// AStar::Obs2i real_obs2{ -0.01,0.145,0.1 };
	// AStar::Obs2i real_obs3{ -0.05,-0.11,0.1 };
	
	// //first (max vel 0.26 damping 0.003, 4.224s)
	// AStar::Obs2i real_obs1{ -0.01,0.145,0.1 };
	// AStar::Obs2i real_obs2{ -0.05,-0.11,0.09 };
	// AStar::Obs2i real_obs3{ 0.02,0.0,0.08 };
	
	// //second (max vel 0.26 damping 0.003, 3.679s)
	// AStar::Obs2i real_obs1{ 0.05,0.0,0.1 };
	// AStar::Obs2i real_obs2{ -0.025,0.12,0.09 };
	// AStar::Obs2i real_obs3{ -0.09,0.0,0.08 };	
	
	// //third (max vel 0.26 damping 0.003, 3.909s)
	// AStar::Obs2i real_obs1{ 0.05,    0.0,     0.1 };
	// AStar::Obs2i real_obs2{ -0.05,   0.1,     0.09 };
	// AStar::Obs2i real_obs3{ -0.05,   -0.1,    0.08 };
	
	// //fourth (max vel 0.26 damping 0.003, 4.096s)
	// AStar::Obs2i real_obs1{ 0.0,     0.15,    0.1 };
	// AStar::Obs2i real_obs2{ -0.05,   0.0,     0.09 };
	// AStar::Obs2i real_obs3{ 0.05,    -0.08,   0.08 };
	
	// //fifth (max vel 0.26 damping 0.003, 3.803s)
	// AStar::Obs2i real_obs1{ 0.05,    -0.1,    0.1 };
	// AStar::Obs2i real_obs2{ 0.05,    0.05,    0.09 };
	// AStar::Obs2i real_obs3{ -0.06,   -0.03,   0.08 };
	
	//sixth
	//(max vel 0.26 damping 0.003, 4.14s)
	//(max vel 0.28 damping 0.003, 3.845s) z:+0.1594 zzzzzzzzz
	//(max vel 0.275 damping 0.003, 3.915s) z:+0.1586
	//(max vel 0.27 damping 0.003, 3.987s) z:+0.1572
	AStar::Obs2i real_obs1{ -0.01,   0.16,    0.1 };
	AStar::Obs2i real_obs2{ -0.01,   0.015,   0.09 };
	AStar::Obs2i real_obs3{ -0.02,   -0.12,   0.08 };
	
	// //seventh
	// //(max vel 0.26, damping 0.003, 4.115s)
	// //(max vel 0.26, damping 0.003, duration gain 0.95, 3.91s)
	// //(max vel 0.25, damping 0.003, duration gain 0.90, 3.852s)
	// AStar::Obs2i real_obs1{ 0.01,    0.16,    0.1 };
	// AStar::Obs2i real_obs2{ -0.01,   0.015,   0.09 };
	// AStar::Obs2i real_obs3{ -0.02,   -0.12,   0.08 };

	double safety_value = 0.02;	//real size		

	AStar::Obs2i obs1;
	AStar::Obs2i obs2;
	AStar::Obs2i obs3;

	AStar::Vec3d real_start{ 0.66, -0.18, 0.15 };
	AStar::Vec3d real_target1{ 0.5, 0.18, 0.15 };
	AStar::Vec3d real_target2{ 0.66, 0.18, 0.15 };
	AStar::Vec3d real_target3{ 0.5, -0.18, 0.15 };

	AStar::Vec2i start{ 2 * grid_multiplier, (width - 2) * grid_multiplier - 1 };
	AStar::Vec2i target1{ (height - 2) * grid_multiplier - 1, 2 * grid_multiplier };
	AStar::Vec2i target2{ 2 * grid_multiplier, 2 * grid_multiplier };
	AStar::Vec2i target3{ (height - 2) * grid_multiplier - 1, (width - 2) * grid_multiplier - 1 };

	AStar::Generator generator;
	generator.setWorldSize({ height, width }, grid_multiplier);
	generator.setHeuristic(AStar::Heuristic::euclidean);
	generator.setDiagonalMovement(true);
	generator.changeGridObs(real_obs1, real_obs2, real_obs3, obs1, obs2, obs3, safety_value);
	generator.addCollision(obs1, obs2, obs3);

	// std::cout << obs1.x << " " << obs1.y << " " << obs1.d << std::endl;
	// std::cout << obs2.x << " " << obs2.y << " " << obs2.d << std::endl;
	// std::cout << obs3.x << " " << obs3.y << " " << obs3.d << std::endl;

	std::cout << "Generate path ... \n";
	//To do: make real input position to grid position?
	AStar::CoordinateList path1, path2, path3, path;

	generator.findPath(path3, target2, target3);
	int path_size3 = path3.size();
	std::reverse(path3.begin(), path3.end());
	generator.Simplepath(path3, obs1, obs2, obs3);


	generator.findPath(path2, target1, target2);
	int path_size2 = path2.size();
	std::reverse(path2.begin(), path2.end());
	generator.Simplepath(path2, obs1, obs2, obs3);


	generator.findPath(path1, start, target1);
	int path_size1 = path1.size();
	std::reverse(path1.begin(), path1.end());
	generator.Simplepath(path1, obs1, obs2, obs3);


	path.insert(path.end(), path1.begin(), path1.end());
	path_size1 = path.size();
	path.insert(path.end(), path2.begin(), path2.end());
	path_size2 = path.size() - path_size1;
	path.insert(path.end(), path3.begin(), path3.end());
	path.push_back(target3);
	path_size3 = path.size() - path_size1 - path_size2;

	//AStar::CoordinateList path;

	//generator.findPath(path, target2, target3);
	//int path_size3 = path.size();
	//generator.findPath(path, target1, target2);
	//int path_size2 = path.size() - path_size3;
	//generator.findPath(path, start, target1);
	//int path_size1 = path.size() - path_size2 - path_size3;

	//std::reverse(path.begin(), path.end());

	//generator.Simplepath(path, obs1, obs2, obs3);

	std::cout << "  " << std::endl;
	for (auto& coordinate : path) {
		std::cout << coordinate.x << " " << coordinate.y << "\n";
	}

	generator.printMap();

	//To do: write code changeRealPath
	double scalef = grid_multiplier;
	std::vector<AStar::Vec3d> robot_path1_temp, robot_path2_temp, robot_path3_temp;
	generator.changeRealPath(path1, robot_path1_temp, scalef, real_start, real_target1);
	generator.changeRealPath(path2, robot_path2_temp, scalef, real_target1, real_target2);
	generator.changeRealPath(path3, robot_path3_temp, scalef, real_target2, real_target3);

	robot_path1.clear(); 
	robot_path2.clear();
	robot_path3.clear();

	for(int i =0; i < robot_path1_temp.size();i++)
	{
		Vector3d path_temp;
		path_temp << robot_path1_temp[i].x, robot_path1_temp[i].y, robot_path1_temp[i].z;
		robot_path1.push_back(path_temp);
	}
	
	for (int i = 0; i < robot_path2_temp.size(); i++)
	{
		Vector3d path_temp;
		path_temp << robot_path2_temp[i].x, robot_path2_temp[i].y, robot_path2_temp[i].z;
		robot_path2.push_back(path_temp);

	}

	for (int i = 0; i < robot_path3_temp.size(); i++)
	{
		Vector3d path_temp;
		path_temp << robot_path3_temp[i].x, robot_path3_temp[i].y, robot_path3_temp[i].z;
		robot_path3.push_back(path_temp);
	}

	//std::cout << "task1: " << robot_path[0].x << " " << robot_path[0].y << " " << path_size1 << "\n";
	//std::cout << "task1: " << robot_path[1].x << " " << robot_path[1].y << " " << path_size1 << "\n";
	//std::cout << "task1: " << robot_path[2].x << " " << robot_path[2].y << " " << path_size1 << "\n";
	//std::cout << "task1: " << robot_path[3].x << " " << robot_path[3].y << " " << path_size1 << "\n";
	//std::cout << "task1: " << robot_path[4].x << " " << robot_path[4].y << " " << path_size1 << "\n";
	//std::cout << "task2: " << robot_path[path_size1].x << " " << robot_path[path_size1].y << " " << path_size2 << "\n";
	//std::cout << "task3: " << robot_path[path_size1 + path_size2].x << " " << robot_path[path_size1 + path_size2].y << " " << path_size3 << "\n";
	//std::cout << "task end: " << robot_path[path_size1 + path_size2 + path_size3 - 1].x << " " << robot_path[path_size1 + path_size2 + path_size3 - 1].y << " " << "\n";

	//std::cout << "  " << std::endl;
	//for (auto& coordinate : robot_path) {
	//	std::cout << coordinate.x << " " << coordinate.y << "\n";
	//}
}

// Controller Core Methods ----------------------------


void ArmController::findDuration(vector<Vector3d>& robot_path, vector<double>& duration_list)
{
	duration_list.clear();
	for (int i = 0; i < robot_path.size()-1; i++)
	{
		double length = (robot_path[i + 1] - robot_path[i]).norm();
		duration_list.push_back(length / 0.4);
	}
}

void ArmController::setMode(const std::string& mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}

void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);
}

#ifdef USING_REAL_ROBOT
void ArmController::initPosition(franka::RobotState state)
{
	q_init_ = q_;
	q_desired_ = q_init_;
	model_interface_.setRobotState(state);
}

void ArmController::updateTime(double dt)
{
	dt_ = dt;
	play_time_ += dt;
}
#else  // USING_REAL_ROBOT
void ArmController::initModel()
{
	model_ = make_shared<Model>();

	model_->gravity = Vector3d(0., 0, -GRAVITY);

	double mass[DOF];
	mass[0] = 1.0;
	mass[1] = 1.0;
	mass[2] = 1.0;
	mass[3] = 1.0;
	mass[4] = 1.0;
	mass[5] = 1.0;
	mass[6] = 1.0;

	Vector3d axis[DOF];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0 * Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0 * Eigen::Vector3d::UnitY();
	axis[6] = -1.0 * Eigen::Vector3d::UnitZ();


	Eigen::Vector3d global_joint_position[DOF];

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	// in global frame
	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	//com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);  // for homework
	com_position_[6] = Vector3d(0.0871, 0.0, 0.9089);  // for project

	for (int i = 0; i < DOF; i++)
		com_position_[i] -= global_joint_position[i];

	Math::Vector3d inertia[DOF];
	for (int i = 0; i < DOF; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

	for (int i = 0; i < DOF; i++) {
		body_[i] = Body(mass[i], com_position_[i], inertia[i]);
		joint_[i] = Joint(JointTypeRevolute, axis[i]);
		if (i == 0)
			body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
	}
}

void ArmController::initPosition()
{
	q_init_ = q_;
	q_desired_ = q_init_;
}
#endif  // USING_REAL_ROBOT

void ArmController::readData(const Vector7d& position, const Vector7d& velocity, const Vector7d& torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		//torque_(i) = torque(i);
		torque_(i) = -torque(i);
	}
}
void ArmController::readData(const Vector7d& position, const Vector7d& velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

const Vector7d& ArmController::getDesiredPosition()
{
	return q_desired_;
}

const Vector7d& ArmController::getDesiredTorque()
{
	return torque_desired_;
}

void ArmController::closeFile()
{
	if (writeFile.is_open())
		writeFile.close();
}

// ----------------------------------------------------

