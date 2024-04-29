#ifndef TOWR_IK_A1_H_
#define TOWR_IK_A1_H_

#include <cmath>
#include <array>
#include <Eigen/Dense>
#include <xpp_vis/inverse_kinematics.h>
#include <iostream>
// #include <towr_ros/towr_ros_interface.h>

namespace towr
{

    class A1LegIKController
    {
    public:
        static constexpr int kJointLegNum = 3;

        explicit A1LegIKController(double hip_length, double thigh_length, double calf_length,
                                   double leg_offset_x, double leg_offset_y, double trunk_offset_z,
                                   double hip_max, double hip_min,
                                   double thigh_max, double thigh_min,
                                   double calf_max, double calf_min,
                                   double trunk_ixx, double hip_ixx, double thigh_ixx, double calf_ixx);
        // Solve inverse kinematics to obtain joint angles
        const Eigen::Matrix<double, 6, 1> SolveIK(Eigen::Vector3d &current_cartesian_position,
                                                  Eigen::Matrix<double, kJointLegNum, 1> previous_joint_angles,
                                                  Eigen::Vector3d &current_cartesian_velocities, Eigen::Vector3d base_position, Eigen::Vector3d &base_velocity, int hip_id, double time_step_);
        Eigen::Matrix<double, kJointLegNum, 1> Geomotory_and_InverseKinematics(Eigen::Vector3d &target_position, Eigen::Vector3d base_position, Eigen::Matrix3d &R_base_to_hip, int hip_id);
        void EnforceAngleLimits(double &val, double max, double min) const;
        Eigen::Vector3d ComputeJacobian_and_ForwardKinematics(Eigen::Matrix<double, kJointLegNum, 1> &joint_angles, Eigen::Matrix<double, kJointLegNum, kJointLegNum> &Jacobian_Matrix, int hip_id);
        Eigen::Vector3d TransformWorldFrameToHipFrame(Eigen::Vector3d &foot_position_in_world_frame, Eigen::Vector3d &base_position_in_world_frame, int hip_id);
        Eigen::Vector3d TransformHipFrameToWorldFrame(Eigen::Vector3d &foot_position_in_hip_frame, Eigen::Vector3d &base_position_in_world_frame, int hip_id);
        Eigen::VectorXd ComputeJointVelocities(Eigen::Matrix<double, kJointLegNum, 1> previous_joint_angles,
                                               Eigen::Vector3d &cartesian_velocities, Eigen::Matrix<double, kJointLegNum, kJointLegNum> jacobian, int hip_id);
        void SingleLegAngleSaturation(Eigen::Vector3d &joint_angle);
        Eigen::Vector3d Flag_Detect(int hip_id);

    private:
        // Dynamics parameters
        double hip_length_, thigh_length_, calf_length_;
        double leg_offset_x_, leg_offset_y_, trunk_offset_z_;
        double hip_max_, hip_min_;
        double thigh_max_, thigh_min_;
        double calf_max_, calf_min_;
        double trunk_ixx_, hip_ixx_, thigh_ixx_, calf_ixx_;
        bool flag;

        // Constants for dynamics calculation

        Eigen::Matrix<double, kJointLegNum, kJointLegNum> jacobian;
        Eigen::Matrix<double, kJointLegNum, kJointLegNum> jacobian_test;
        Eigen::MatrixXd jacobian_pseudo_inverse;
        Eigen::Vector3d cartesian_velocities;
        Eigen::Vector3d cartesian_position;
        Eigen::Matrix<double, kJointLegNum, 1> joint_velocities_eigen;
        Eigen::Matrix<double, kJointLegNum, 1> joint_angle_eigen;
        Eigen::Matrix<double, kJointLegNum, 1> joint_velocities;
        Eigen::Matrix<double, kJointLegNum, 1> joint_angles;
        Eigen::Vector3d foot_position_in_hip_frame;
        Eigen::Vector3d foot_position_in_world_frame;
        Eigen::Vector3d hip_position_in_world_frame;
        Eigen::Matrix<double, 2 * kJointLegNum, 1> joint_info;
        Eigen::Matrix<double, kJointLegNum, 1> perturbed_angles;
        Eigen::Vector3d perturbed_position;
        Eigen::Vector3d last_base_velocity;
    };
}
#endif
