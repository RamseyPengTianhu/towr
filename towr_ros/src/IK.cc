// IK.cc
#include <array>
#include <Eigen/Dense>
#include "towr_ros/IK.h"
#include <iostream>
#include <towr_ros/towr_ros_interface.h>
#include "/home/tianhu/TO/src/towr/towr_ros/math/constants.h"
#include <algorithm>
// #include "/home/tianhu/TO/src/towr/towr_ros/third_party/glog/src/glog/logging.h"
#include <glog/logging.h>

namespace
{
  constexpr double kTiny = 1e-3;
}
namespace towr
{
  A1LegIKController::A1LegIKController(double hip_length, double thigh_length, double calf_length,
                                       double leg_offset_x, double leg_offset_y, double trunk_offset_z,
                                       double hip_max, double hip_min,
                                       double thigh_max, double thigh_min,
                                       double calf_max, double calf_min,
                                       double trunk_ixx, double hip_ixx, double thigh_ixx, double calf_ixx)
      : hip_length_(hip_length), thigh_length_(thigh_length), calf_length_(calf_length),
        leg_offset_x_(leg_offset_x), leg_offset_y_(leg_offset_y), trunk_offset_z_(trunk_offset_z),
        hip_max_(hip_max), hip_min_(hip_min),
        thigh_max_(thigh_max), thigh_min_(thigh_min),
        calf_max_(calf_max), calf_min_(calf_min),
        trunk_ixx_(trunk_ixx), hip_ixx_(hip_ixx), thigh_ixx_(thigh_ixx), calf_ixx_(calf_ixx)
  {
    jacobian = Eigen::MatrixXd::Zero(3, 3);
    jacobian_pseudo_inverse = Eigen::MatrixXd::Zero(3, 3);
    cartesian_velocities = Eigen::Vector3d::Zero();
    joint_velocities_eigen = Eigen::MatrixXd::Zero(kJointLegNum, 1);
    joint_velocities = Eigen::MatrixXd::Zero(kJointLegNum, 1);
    joint_angles = Eigen::MatrixXd::Zero(kJointLegNum, 1);
    perturbed_angles = Eigen::MatrixXd::Zero(kJointLegNum, 1);
    perturbed_position = Eigen::Vector3d::Zero();
    last_base_velocity = Eigen::Vector3d::Zero();
    flag = false;
  }

  const Eigen::Matrix<double, 6, 1> A1LegIKController::SolveIK(Eigen::Vector3d current_cartesian_position,
                                                               Eigen::Matrix<double, kJointLegNum, 1> previous_joint_angles,
                                                               Eigen::Vector3d current_cartesian_velocities, Eigen::Vector3d base_position, Eigen::Vector3d base_velocity, Eigen::Matrix3d Rot_matrix, int hip_id, int robot_type)
  {
    if (robot_type == 5)
    {
      hip_id = hip_id + 2;
    }
    cartesian_position = current_cartesian_position;
    // world to base ori transform
    joint_angles = Geomotory_and_InverseKinematics(cartesian_position, base_position, Rot_matrix, hip_id, robot_type);
    // std::cout << "joint_velocities_eigen:\n"
    //           << joint_velocities_eigen << std::endl;
    std::cout << "Calculated_joint_angles:\n"
              << joint_angles << std::endl;
    Eigen::Vector3d test_pos = ComputeJacobian_and_ForwardKinematics(joint_angles, jacobian, hip_id, robot_type);
    joint_velocities = ComputeJointVelocities(previous_joint_angles, current_cartesian_velocities, jacobian, hip_id, robot_type);

    // std::cout << "Foot pos in world frame:\n"
    //           << cartesian_position << std::endl;
    Eigen::Vector3d foot_pos = TransformWorldFrameToHipFrame(cartesian_position, base_position, Rot_matrix, hip_id, robot_type);
    std::cout << "Foot pos in hip frame:\n"
              << foot_pos << std::endl;

    std::cout
        << "Test Pos:\n"
        << test_pos << std::endl;
    std::cout << "Pos error is:\n"
              << (foot_pos - test_pos).lpNorm<1>() << std::endl;

    if ((foot_pos - test_pos).lpNorm<1>() > 10 * kTiny)
    {

      // exit(1);

      // return true;
    }
    if (isnan(joint_angles[0]) || isnan(joint_angles[1]) || isnan(joint_angles[2]))
    {
      exit(1); // Exit the program if any angle is NaN
    }

    joint_info << joint_angles, joint_velocities;

    return joint_info;
  }

  // Compute the Jacobian
  Eigen::Vector3d A1LegIKController::ComputeJacobian_and_ForwardKinematics(Eigen::Matrix<double, kJointLegNum, 1> joint_angles, Eigen::Matrix<double, kJointLegNum, kJointLegNum> &Jacobian_Matrix, int hip_id, int robot_type)
  {

    double l1 = hip_length_;
    double l2 = thigh_length_;
    double l3 = calf_length_;
    if (robot_type == 5)
    {
      joint_angles[1] = joint_angles[1] - M_PI_2;
    }

    // Calssify the leg position in left or right
    int sideSign = (hip_id == 0 || hip_id == 2) ? 1 : -1;

    Eigen::Vector3d position;
    double s1 = std::sin(joint_angles[0]); // hip joint
    double s2 = std::sin(joint_angles[1]); // thigh joint
    double s3 = std::sin(joint_angles[2]); // calf joint
    double s4 = std::sin(joint_angles[3]); // calf joint

    double c1 = std::cos(joint_angles[0]); // hip joint
    double c2 = std::cos(joint_angles[1]); // thigh joint
    double c3 = std::cos(joint_angles[2]); // calf joint
    double c4 = std::cos(joint_angles[3]); // calf joint

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;
    double c34 = c3 * c4 - s3 * s4;
    double s34 = s3 * c4 + c3 * s4;

    // Store the Jacobian Matrix
    Jacobian_Matrix(0, 0) = 0;
    Jacobian_Matrix(1, 0) = -sideSign * l1 * s1 + l2 * c2 * c1 + l3 * c23 * c1;
    Jacobian_Matrix(2, 0) = sideSign * l1 * c1 + l2 * c2 * s1 + l3 * c23 * s1;
    Jacobian_Matrix(0, 1) = -l3 * c23 - l2 * c2;
    Jacobian_Matrix(1, 1) = -l2 * s2 * s1 - l3 * s23 * s1;
    Jacobian_Matrix(2, 1) = l2 * s2 * c1 + l3 * s23 * c1;

    Jacobian_Matrix(0, 2) = -l3 * c23;
    Jacobian_Matrix(1, 2) = -l3 * s23 * s1;
    Jacobian_Matrix(2, 2) = l3 * s23 * c1;

    // if (robot_type == 5)
    // {
    //   position[2] = -l3 * s23 - l2 * s2;                                    // x-coordinate
    //   position[1] = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;    // y-coordinate
    //   position[0] = -(l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2); // z-coordinate
    // }
    // else
    // {
    //   position[0] = -l3 * s23 - l2 * s2;                                 // x-coordinate
    //   position[1] = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1; // y-coordinate
    //   position[2] = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2; // z-coordinate
    // }

    position[0] = -l3 * s23 - l2 * s2;                                 // x-coordinate
    position[1] = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1; // y-coordinate
    position[2] = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2; // z-coordinate

    return position;
  }

  // Compute the Jacobian
  Eigen::Matrix<double, 3, 1> A1LegIKController::Geomotory_and_InverseKinematics(Eigen::Vector3d target_position, Eigen::Vector3d base_position, Eigen::Matrix3d Rot_matrix, int hip_id, int robot_type)
  {
    // Define leg parameters
    double l1 = hip_length_;
    double l2 = thigh_length_;
    double l3 = calf_length_;

    double q_Hip, q_Thigh, q_Calf; // Joint angles for hip, thigh, and calf joints
    Eigen::Vector3d foot_posi;
    Eigen::Vector3d base_posi;
    Eigen::Vector3d Trans_pos;

    // if (robot_type == 5)
    // {
    //   foot_posi[0] = target_position[2];
    //   foot_posi[2] = target_position[0];
    //   base_posi[2] = base_position[0];
    //   base_posi[0] = base_position[2];
    //   Trans_pos = TransformWorldFrameToHipFrame(foot_posi, base_posi, Rot_matrix, hip_id);
    // }
    // else
    // {
    //   Trans_pos = TransformWorldFrameToHipFrame(target_position, base_position, Rot_matrix, hip_id);
    // }
    Trans_pos = TransformWorldFrameToHipFrame(target_position, base_position, Rot_matrix, hip_id, robot_type);

    // Extract target coordinates

    double x = Trans_pos[0];
    double y = Trans_pos[1];
    double z = Trans_pos[2];
    // std::cout << "Trans_pos:\n"
    //           << Trans_pos << std::endl;
    // std::cout << "x:\n"
    //           << x << std::endl;
    // std::cout << "y:\n"
    //           << y << std::endl;
    // std::cout << "z:\n"
    //           << z << std::endl;

    int sideSign = (hip_id == 0 || hip_id == 2) ? 1 : -1;

    // double gamma_1, gamma_2;
    // double dyz = std::sqrt(y * y + z * z);
    // double lyz = std::sqrt(dyz * dyz - l1 * l1);

    // // // Classify the leg position in left or right

    // gamma_1 = std::atan2(z, y);
    // gamma_2 = std::atan2(lyz, l1);
    // q_Hip = gamma_1 + sideSign * gamma_2;

    // double lxz = std::sqrt(lyz * lyz + x * x);

    // double n = (lxz * lxz - l3 * std::cos(q_Hip) * l3 * std::cos(q_Hip) - l2 * std::cos(q_Hip) * l2 * std::cos(q_Hip)) / (2 * l2 * std::cos(q_Hip) * l3 * std::cos(q_Hip));
    // q_Calf = sideSign * ::acos(std::clamp(n, -1.0, 1.0));
    // double m = (l3 * std::cos(q_Hip) * l3 * std::cos(q_Hip) - lxz * lxz - l2 * std::cos(q_Hip) * l2 * std::cos(q_Hip)) / (2 * l2 * std::cos(q_Hip) * lxz);

    // double alpha_1, alpha_2;
    // alpha_1 = std::atan2(lyz, x);
    // alpha_2 = std::acos(std::clamp(m, -1.0, 1.0));
    // q_Thigh = -sideSign * (M_PI / 2 - sideSign * (alpha_1 + alpha_2));

    double dyz = std::sqrt(y * y + z * z);
    double lyz = std::sqrt(dyz * dyz - l1 * l1);

    double gamma_1 = std::atan2(z, y);
    double gamma_2 = std::atan2(lyz, l1);

    // double R_gamma = gamma_1 - gamma_2;
    // double L_gamma = gamma_1 + gamma_2;
    double gamma = gamma_1 + sideSign * gamma_2;
    q_Hip = gamma;

    double lxz = std::sqrt(lyz * lyz + x * x);

    double n = (lxz * lxz - l3 * l3 - l2 * l2) / (2 * l2 * l3);

    double beta = -std::acos(std::clamp(n, -1.0, 1.0));

    q_Calf = beta;
    double m = (l3 * l3 - lxz * lxz - l2 * l2) / (2 * l2 * lxz);

    double alpha_1 = atan2(lyz, x);
    double alpha_2 = -std::acos(std::clamp(m, -1.0, 1.0));

    q_Thigh = -sideSign * (M_PI_2 - sideSign * (alpha_1 + alpha_2));

    // std::cout << "q_Thigh:\n"
    //           << q_Thigh << std::endl;
    if (q_Thigh > M_PI_2)
    {
      q_Thigh = q_Thigh - M_PI;
    }
    else if (q_Thigh < -M_PI_2)
    {
      q_Thigh = q_Thigh + M_PI;
    }

    if (q_Hip > M_PI_2)
    {
      q_Hip = q_Hip - M_PI;
    }
    else if (q_Hip < -M_PI_2)
    {
      q_Hip = q_Hip + M_PI;
    }

    if (robot_type == 5)
    {
      q_Thigh += M_PI_2;
    }

    // double d = std::sqrt(std::pow(y, 2) + std::pow(y, 2));

    // // Calculate l
    // double l = std::sqrt(std::pow(d, 2) - std::pow(l1, 2));

    // // Calculate n
    // double n = (std::pow(l, 2) + std::pow(x, 2) - std::pow(l3, 2) - std::pow(l2, 2)) / (2 * l2);

    // // Calculate s
    // double s = std::sqrt(std::pow(l, 2) + std::pow(x, 2));

    // // Calculate theta1
    // double theta1 = std::atan(std::abs(y / z)) - std::atan(std::abs(l1 / l));

    // // Calculate theta2
    // double theta2 = -std::atan(x / l) + std::acos((l2 + n) / s);

    // // Calculate theta3
    // double theta3 = -std::acos(std::abs((std::pow(l, 2) + std::pow(x, 2) - std::pow(l3, 2) - std::pow(l2, 2)) / (2 * l2 * l3)));
    // q_Hip = sideSign * theta1;
    // q_Thigh = theta2;
    // q_Calf = theta3;

    joint_angles
        << q_Hip,
        q_Thigh, q_Calf; // Return joint angles for hip, thigh, and calf joints
    // SingleLegAngleSaturation(joint_angles);

    return joint_angles;
  }
  void
  A1LegIKController::EnforceAngleLimits(double &val, double max, double min) const
  {
    // totally exaggerated joint angle limits

    // reduced joint angles for optimization
    val = val > max ? max : val;

    val = val < min ? min : val;
  }

  Eigen::Vector3d A1LegIKController::TransformWorldFrameToHipFrame(
      Eigen::Vector3d foot_position_in_world_frame, Eigen::Vector3d base_position_in_world_frame, Eigen::Matrix3d Rot_matrix, int hip_id, int robot_type)
  {
    Eigen::Vector3d hip_position_in_world_frame;
    Eigen::Vector3d hip_offset;
    // hip_id = hip_id + 2;

    // Determine hip offset based on hip_id
    if (robot_type == 5)
    {
      switch (hip_id)
      {
      case 0: // FL Hip Joint
        hip_offset = Eigen::Vector3d(0, leg_offset_y_, -leg_offset_x_);

        break;
      case 1: // FR Hip Joint
        hip_offset = Eigen::Vector3d(0, -leg_offset_y_, -leg_offset_x_);

        break;
      case 2: // RL Hip Joint
        hip_offset = Eigen::Vector3d(0, leg_offset_y_, -leg_offset_x_);

        break;
      case 3: // RR Hip Joint
        hip_offset = Eigen::Vector3d(0, -leg_offset_y_, -leg_offset_x_);

        break;
      default:
        return Eigen::Vector3d::Zero();
      }
    }
    else
    {
      switch (hip_id)
      {
      case 0: // FL Hip Joint
        hip_offset = Eigen::Vector3d(leg_offset_x_, leg_offset_y_, 0);

        break;
      case 1: // FR Hip Joint
        hip_offset = Eigen::Vector3d(leg_offset_x_, -leg_offset_y_, 0);

        break;
      case 2: // RL Hip Joint
        hip_offset = Eigen::Vector3d(-leg_offset_x_, leg_offset_y_, 0);

        break;
      case 3: // RR Hip Joint
        hip_offset = Eigen::Vector3d(-leg_offset_x_, -leg_offset_y_, 0);

        break;
      default:
        // Invalid hip_id, return zero vector or handle error
        return Eigen::Vector3d::Zero();
      }
    }

    // Calculate hip position in world frame
    hip_position_in_world_frame = base_position_in_world_frame + hip_offset;

    // Transform foot position from world frame to hip frame
    Eigen::Vector3d foot_position_in_hip_frame = Rot_matrix.transpose() * (foot_position_in_world_frame - hip_position_in_world_frame);

    return foot_position_in_hip_frame;
  }
  Eigen::Vector3d A1LegIKController::TransformHipFrameToWorldFrame(
      Eigen::Vector3d foot_position_in_hip_frame, Eigen::Vector3d base_position_in_world_frame, int hip_id)
  {
    // Determine the hip joint's position in the world frame based on hip_id

    switch (hip_id)
    {
    case 0:
      // FL Hip Joint
      hip_position_in_world_frame = base_position_in_world_frame + Eigen::Vector3d(leg_offset_x_, leg_offset_y_, 0);
      break;
    case 1:
      // FR Hip Joint
      hip_position_in_world_frame = base_position_in_world_frame + Eigen::Vector3d(leg_offset_x_, -leg_offset_y_, 0);
      break;
    case 2:
      // RL Hip Joint
      hip_position_in_world_frame = base_position_in_world_frame + Eigen::Vector3d(-leg_offset_x_, leg_offset_y_, 0);
      break;
    case 3:
      // RR Hip Joint
      hip_position_in_world_frame = base_position_in_world_frame + Eigen::Vector3d(-leg_offset_x_, -leg_offset_y_, 0);
      break;
    default:
      // Handle invalid hip_id (optional)
      break;
    }

    // Transform foot position from hip frame to world frame
    Eigen::Vector3d foot_position_in_world_frame = foot_position_in_hip_frame + hip_position_in_world_frame;

    return foot_position_in_world_frame;
  }

  Eigen::VectorXd A1LegIKController::ComputeJointVelocities(Eigen::Matrix<double, kJointLegNum, 1> previous_joint_angles,
                                                            Eigen::Vector3d &cartesian_velocities, Eigen::Matrix<double, kJointLegNum, kJointLegNum> jacobian, int hip_id, int robot_type)
  {
    // Assuming a simple linear relationship for demonstration purposes.
    // You should replace this with the actual computation based on your system dynamics.

    Eigen::Vector3d FK_Pos = ComputeJacobian_and_ForwardKinematics(joint_angles, jacobian, hip_id, robot_type);
    jacobian_pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    return jacobian_pseudo_inverse * cartesian_velocities;
  }

  void A1LegIKController::SingleLegAngleSaturation(Eigen::Vector3d &joint_angle)
  {

    // Input: q_hip and q_knee.
    for (int i = 0; i < joint_angle.size(); i++)
    {
      if (joint_angle(i) > M_PI / 2.0)
      {
        joint_angle(i) -= M_PI;
      }
      else if (joint_angle(i) < -M_PI / 2.0)
      {
        joint_angle(i) += M_PI;
      }
    }
  }
  Eigen::Vector3d A1LegIKController::Flag_Detect(
      int hip_id)
  {
    Eigen::Vector3d Trans;
    // Each hip position offest respect to base
    Eigen::Vector3d FR_Hip_Trans(0.1805, -0.052, 0);
    Eigen::Vector3d FL_Hip_Trans(0.1805, 0.052, 0);
    Eigen::Vector3d RR_Hip_Trans(-0.1805, -0.052, 0);
    Eigen::Vector3d RL_Hip_Trans(-0.1805, 0.052, 0);
    switch (hip_id)
    {
    case 0:
      // FL Hip Joint
      Trans = FL_Hip_Trans;
      break;
    case 1:
      // FR Hip Joint
      Trans = FR_Hip_Trans;

      break;
    case 2:
      // RL Hip Joint
      Trans = RL_Hip_Trans;

      break;
    case 3:
      // RR Hip Joint
      Trans = RR_Hip_Trans;

      break;
    default:
      break;
    }

    // Transform foot position from world frame to hip frame

    return Trans;
  }

} // namespace towr
