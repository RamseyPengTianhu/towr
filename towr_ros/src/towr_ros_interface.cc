/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros_interface.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>
// #include "/home/tianhu/TO/src/xpp/robots/xpp_hyq/include/xpp_hyq/IK_a1.h"
#include "towr_ros/IK.h"
// #include <xp>
#include <xpp_hyq/inverse_kinematics_a1.h>
#include "/home/tianhu/TO/src/xpp/robots/xpp_hyq/include/xpp_hyq/inverse_kinematics_a1.h"
// #include <towr_ros/>

#include <rosbag/view.h>
#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Dense>
// #include <qt_5/QtCore/qvector.h>

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&...args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

namespace towr
{
  FILE *f_traj;
  constexpr double kPi = 3.14159265358979323846;

  TowrRosInterface::TowrRosInterface()
  {
    ::ros::NodeHandle n;

    user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                    &TowrRosInterface::UserCommandCallback, this);

    initial_state_pub_ = n.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1);

    initial_joint_state_pub_ = n.advertise<xpp_msgs::RobotStateJoint>(xpp_msgs::joint_desired, 1);

    robot_parameters_pub_ = n.advertise<xpp_msgs::RobotParameters>(xpp_msgs::robot_parameters, 1);

    solver_ = std::make_shared<ifopt::IpoptSolver>();

    visualization_dt_ = 0.01;
    double ang2rad = (M_PI / 180);

    ik_controller = make_unique<A1LegIKController>(0.0838, 0.2, 0.2, 0.1805, 0.0512, 0.01675,
                                                   46 * ang2rad, -46 * ang2rad, 240 * ang2rad, -60 * ang2rad, -52.5 * ang2rad, -154.5 * ang2rad,
                                                   0.064713601, 0.000552929, 0.001367788, 0.000032426);
    // auto a1_ik = std::make_shared<xpp::InverseKinematicsA1>();
    Jacobian_Matrix = Eigen::Matrix3d::Zero();
  }

  BaseState
  TowrRosInterface::GetGoalState(const TowrCommandMsg &msg) const
  {
    BaseState goal;
    goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
    goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
    goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
    goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

    return goal;
  }

  void
  TowrRosInterface::UserCommandCallback(const TowrCommandMsg &msg)
  {
    // robot model
    formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot));
    auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
    robot_parameters_pub_.publish(robot_params_msg);

    // terrain
    auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
    formulation_.terrain_ = HeightMap::MakeTerrain(terrain_id);

    int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
    formulation_.params_ = GetTowrParameters(n_ee, msg);
    formulation_.final_base_ = GetGoalState(msg);

    SetTowrInitialState();

    // solver parameters
    SetIpoptParameters(msg);

    // visualization
    PublishInitialState();

    // Defaults to /home/user/.ros/
    std::string bag_file = "towr_trajectory.bag";
    if (msg.optimize || msg.play_initialization)
    {
      nlp_ = ifopt::Problem();
      for (auto c : formulation_.GetVariableSets(solution))
        nlp_.AddVariableSet(c);
      for (auto c : formulation_.GetConstraints(solution))
        nlp_.AddConstraintSet(c);
      for (auto c : formulation_.GetCosts())
        nlp_.AddCostSet(c);

      solver_->Solve(nlp_);
      SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
    }

    // playback using terminal commands
    if (msg.replay_trajectory || msg.play_initialization || msg.optimize)
    {
      int success = system(("rosbag play --topics " + xpp_msgs::robot_state_desired + " " + xpp_msgs::terrain_info + " -r " + std::to_string(msg.replay_speed) + " --quiet " + bag_file).c_str());
    }

    if (msg.plot_trajectory)
    {
      int success = system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
    }

    // to publish entire trajectory (e.g. to send to controller)
    // xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(GetTrajectory());
  }

  void
  TowrRosInterface::PublishInitialState()
  {
    int n_ee = formulation_.initial_ee_W_.size();
    int n_joints_per_ee = 3;
    xpp::RobotStateCartesian xpp(n_ee);
    xpp::RobotStateJoint xpp_joint(n_ee, n_joints_per_ee);
    xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
    xpp.base_.ang.q = EulerConverter::GetQuaternionBaseToWorld(formulation_.initial_base_.ang.p());
    init_base_height = xpp.base_.lin.p_.z();
    init_base_pos = xpp.base_.lin.p_;
    init_joint_angle_single_leg = Eigen::Vector3d(0, kPi / 4, -kPi / 2);
    init_joint_velocity_single_leg = Eigen::Vector3d::Zero();

    for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr)
    {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
      std::cout << "ee_xpp:\n"
                << ee_xpp << std::endl;

      xpp.ee_contact_.at(ee_xpp) = true;
      xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
      Eigen::Vector3d init_foot_pos = xpp.ee_motion_.at(ee_xpp).p_;
      xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
      xpp_joint.q_.at(ee_xpp) = init_joint_angle_single_leg;
      xpp_joint.qd_.at(ee_xpp) = init_joint_velocity_single_leg;
      Eigen::Matrix3d R_base_to_hip;
      Eigen::Vector3d test_angle = ik_controller->Geomotory_and_InverseKinematics(init_foot_pos, xpp.base_.lin.p_, R_base_to_hip, ee_xpp);
      std::cout << "test_angle:\n"
                << test_angle << std::endl;
      std::cout << "init_joint_angle_single_leg:\n"
                << init_joint_angle_single_leg << std::endl;

      // if ((test_angle_1 - init_joint_angle_single_leg).lpNorm<1>() > 0.001)
      // {
      //   exit(1);
      // }

      Eigen::Vector3d test_pos = ik_controller->ComputeJacobian_and_ForwardKinematics(test_angle, Jacobian_Matrix, ee_xpp);
      std::cout << "Test_position in hip frame:\n"
                << test_pos << std::endl;
      test_pos = ik_controller->TransformHipFrameToWorldFrame(test_pos, init_base_pos, ee_xpp);
      std::cout << "Test_position in world frame:\n"
                << test_pos << std::endl;
      std::cout
          << "Current position:\n"
          << xpp.ee_motion_.at(ee_xpp).p_ << std::endl;
      // for (int i = 0; i < 2; i++)
      // {
      //   test_pos(i) = test_pos(i) - xpp.base_.lin.p_(i);
      // }
      // std::cout << "!!!!!!!!!Test_Carteasian space position after:\n"
      //           << test_pos << std::endl;
    }

    std::cout << "init_base_height:\n"
              << init_base_height << std::endl;

    initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
    initial_joint_state_pub_.publish(xpp::Convert::ToRos(xpp_joint));
  }

  std::vector<TowrRosInterface::XppVec>
  TowrRosInterface::GetIntermediateSolutions(std::vector<XppJoint> &Joint_trajectories)
  {
    std::vector<XppVec> trajectories;

    for (int iter = 0; iter < nlp_.GetIterationCount(); ++iter)
    {
      nlp_.SetOptVariables(iter);
      trajectories.push_back(GetTrajectory(Joint_trajectory));
      Joint_trajectories.push_back(Joint_trajectory);
    }

    return trajectories;
  }

  TowrRosInterface::XppVec
  TowrRosInterface::GetTrajectory(XppJoint &Joint_trajectory) const
  {
    // f_traj = fopen("/home/tianhu/TO/src/towr/trajectory/traj.csv","wb");
    // if (!(f_traj)) std::cout << "create traj file failed!<n" ;
    // else std::cout << "create file success!\n";
    // Vector3d pos,vel,w,forceL,forceR;
    // Vector4d theta;
    XppVec trajectory;
    double t = 0.0;
    double T = solution.base_linear_->GetTotalTime();
    Eigen::Matrix<double, 3, 1> Current_joint_angles_single;
    Eigen::Matrix<double, 6, 1> Joint_info_single;
    Eigen::Matrix<double, 3, 1> Joint_pos;
    Eigen::Matrix<double, 3, 1> Joint_vel;

    EulerConverter base_angular(solution.base_angular_);
    int n_ee = solution.ee_motion_.size();
    int n_joints_per_ee = 3;
    xpp::RobotStateJoint Joint_state(n_ee, n_joints_per_ee);
    Eigen::Vector3d init_joint_angle = Eigen::Vector3d(0, kPi / 4, -kPi / 2);
    Eigen::Matrix<double, 3, 3> Jacobian;

    for (int i = 0; i < n_ee; ++i)
    {
      int ee = ToXppEndeffector(n_ee, i).first;

      Joint_state.q_.at(ee) = Eigen::Vector3d(0, kPi / 4, -kPi / 2);
      // Eigen::Vector3d poss = ik_controller->ComputeJacobian_and_ForwardKinematics(Joint_state.q_.at(ee), Jacobian, ee);;
    }

    while (t <= T + 1e-5)
    {
      xpp::RobotStateCartesian state(n_ee);

      state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

      state.base_.ang.q = base_angular.GetQuaternionBaseToWorld(t);
      state.base_.ang.w = base_angular.GetAngularVelocityInWorld(t);
      state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);
      Eigen::Vector3d base_position = state.base_.lin.p_;
      Eigen::Vector3d base_velocity = state.base_.lin.v_;

      for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr)
      {
        int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
        std::cout << "!!!!!!!!!ee_xpp:\n"
                  << ee_xpp << std::endl;

        state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
        state.ee_motion_.at(ee_xpp) = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
        state.ee_forces_.at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
        Eigen::Vector3d foot_position = state.ee_motion_.at(ee_xpp).p_; // Extract pos component
        Eigen::Vector3d foot_velocity = state.ee_motion_.at(ee_xpp).v_; // Extract pos component

        Joint_info_single = ik_controller->SolveIK(foot_position, Joint_state.q_.at(ee_xpp), foot_velocity, base_position, base_velocity, ee_towr, visualization_dt_);

        Joint_pos = Joint_info_single.block(0, 0, 3, 1);
        Joint_vel = Joint_info_single.block(3, 0, 3, 1);

        // std::cout << "!!!!!!!!!Current Joint_angle:\n"<<current_joint_angles<<std::endl;
        // std::cout << "!!!!!!!!!ee_motion_as_vector:\n"<<foot_pos<<std::endl;

        Joint_state.q_.at(ee_xpp) << Joint_pos(0), Joint_pos(1), Joint_pos(2);
        Joint_state.qd_.at(ee_xpp) << Joint_vel(0), Joint_vel(1), Joint_vel(2);
      }

      // forceL <<state.ee_forces_.at(0);
      // forceR <<state.ee_forces_.at(1);

      state.t_global_ = t;
      trajectory.push_back(state);
      // std::cout << "!!!!!!!!!Joint_state.q_:\n"<<Joint_state.q_<<std::endl;

      Joint_trajectory.push_back(Joint_state);
      t += visualization_dt_;
    }
    // std::cout << "closed traj file\n";

    return trajectory;
  }

  xpp_msgs::RobotParameters
  TowrRosInterface::BuildRobotParametersMsg(const RobotModel &model) const
  {
    xpp_msgs::RobotParameters params_msg;
    auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
    params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

    auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
    int n_ee = nominal_B.size();
    for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr)
    {
      Vector3d pos = nominal_B.at(ee_towr);
      params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
      params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
    }

    params_msg.base_mass = model.dynamic_model_->m();

    return params_msg;
  }

  void
  TowrRosInterface::SaveOptimizationAsRosbag(const std::string &bag_name,
                                             const xpp_msgs::RobotParameters &robot_params,
                                             const TowrCommandMsg user_command_msg,
                                             bool include_iterations)
  {
    rosbag::Bag bag;
    bag.open(bag_name, rosbag::bagmode::Write);
    ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

    // save the a-priori fixed optimization variables
    bag.write(xpp_msgs::robot_parameters, t0, robot_params);
    bag.write(towr_msgs::user_command + "_saved", t0, user_command_msg);

    XppJoint Joint_trajectory;
    std::vector<XppJoint> Joint_trajectories;
    // save the trajectory of each iteration
    if (include_iterations)
    {
      auto trajectories = GetIntermediateSolutions(Joint_trajectories);
      int n_iterations = trajectories.size();
      for (int i = 0; i < n_iterations; ++i)
        SaveTrajectoryInRosbag(bag, trajectories.at(i), Joint_trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

      // save number of iterations the optimizer took
      std_msgs::Int32 m;
      m.data = n_iterations;
      bag.write(towr_msgs::nlp_iterations_count, t0, m);
    }

    // save the final trajectory
    auto final_trajectory = GetTrajectory(Joint_trajectory);

    SaveTrajectoryInRosbag(bag, final_trajectory, Joint_trajectory, xpp_msgs::robot_state_desired);

    bag.close();
    // bag.open(bag_name, rosbag::bagmode::Read);

    // rosbag::View view(bag);

    //   // Collect unique topics
    //   std::set<std::string> topics;
    //   for (const rosbag::MessageInstance& msg : view) {
    //       topics.insert(msg.getTopic());
    //        rosbag::View topic_view(bag, rosbag::TopicQuery(msg.getTopic()));

    //   }

    //   // Print information about the bag
    //   std::cout << "Bag information:\n";
    //   std::cout << "  Filename: " << bag.getFileName() << "\n";
    //   std::cout << "  Size: " << bag.getSize() << " bytes\n";
    //   std::cout << "  Topics: " << topics.size() << "\n";

    //   // Iterate over topics and print them
    //   std::cout << "  Topics in the bag:\n";
    //   for (const std::string& topic : topics) {
    //       std::cout << "    " << topic << "\n";
    //   }

    //   std::cout << "Bag opened successfully in Read mode.\n";
  }

  void TowrRosInterface::SaveTrajectoryInRosbag(rosbag::Bag &bag,
                                                const XppVec &traj,
                                                const XppJoint &joint_traj,
                                                const std::string &topic) const
  {
    // std::cout << "!!!!!!!!!traj[i]:\n"<<traj<<std::endl;

    for (size_t i = 0; i < traj.size(); ++i)
    {
      const auto &state = traj[i];
      const auto &joint_state = joint_traj[i];

      auto timestamp = ::ros::Time(state.t_global_ + 1e-6);

      xpp_msgs::RobotStateCartesian cartesian_msg;

      cartesian_msg = xpp::Convert::ToRos(state);
      // std::cout << "!!!!!!!!!cartesian_msg:\n"<<cartesian_msg<<std::endl;

      bag.write(topic, timestamp, cartesian_msg);

      xpp_msgs::RobotStateJoint joint_msg;
      joint_msg = xpp::Convert::ToRos(joint_state);
      bag.write(xpp_msgs::joint_desired, timestamp, joint_msg);

      xpp_msgs::TerrainInfo terrain_msg;
      for (auto ee : state.ee_motion_.ToImpl())
      {
        Vector3d n = formulation_.terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
        terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
        terrain_msg.friction_coeff = formulation_.terrain_->GetFrictionCoeff();
      }

      bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
    }
  }

} /* namespace towr */
