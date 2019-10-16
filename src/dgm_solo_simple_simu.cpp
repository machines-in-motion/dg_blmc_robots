/**
 * \file dgm_solo_simple_simu.cpp
 * \brief The hardware wrapper of the solo naive simulation
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the DGMQuadrupedSimu class.
 */

#include "dg_blmc_robots/dgm_solo_simple_simu.hpp"

namespace dg_blmc_robots
{

  DGMQuadrupedSimu::DGMQuadrupedSimu()
  {
    motor_target_currents_.fill(0.0);
    motor_torques_.fill(0.0);
    motor_target_torques_.fill(0.0);
    motor_encoder_indexes_.fill(0.0);
    joint_positions_.fill(0.0);
    joint_velocities_.fill(0.0);
    joint_torques_.fill(0.0);
    joint_target_torques_.fill(0.0);
    contact_sensors_.fill(0.0);
    slider_positions_.fill(0.0);
    motor_enabled_.fill(0.0);
    motor_ready_.fill(0.0);
    motor_board_enabled_.fill(0.0);
    motor_board_errors_.fill(0.0);
    ctrl_joint_torques_.fill(0.0);

    motors_inertia_ = 0.0;
    motors_torque_constant_ = 0.0;
    motors_gear_ratio_ = 0.0;
  }

  DGMQuadrupedSimu::~DGMQuadrupedSimu()
  {
  }

  void DGMQuadrupedSimu::initialize_hardware_communication_process()
  {
    motors_inertia_ = params_["motor_I"].as<double>();
    motors_torque_constant_ = params_["motor_KT"].as<double>();
    motors_gear_ratio_ = params_["motor_gear_ratio"].as<double>();
  }

  bool DGMQuadrupedSimu::is_in_safety_mode()
  {
    return false;
  }


  void DGMQuadrupedSimu::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    motor_target_currents_ = ctrl_joint_torques_ / motors_torque_constant_;
    motor_torques_ = ctrl_joint_torques_ / motors_gear_ratio_;
    motor_target_torques_ = ctrl_joint_torques_ / motors_gear_ratio_;
    motor_encoder_indexes_.fill(0.0);
    joint_positions_ = joint_positions_ + 
                       control_period_sec_ * joint_velocities_ +
                       control_period_sec_ * control_period_sec_ * 0.5 *
                       motors_inertia_ * ctrl_joint_torques_;
    joint_velocities_ = joint_velocities_ +
                        control_period_sec_ * motors_inertia_ *
                        ctrl_joint_torques_;
    joint_torques_ = ctrl_joint_torques_;
    joint_target_torques_ = ctrl_joint_torques_;
    contact_sensors_.fill(0.0);
    slider_positions_.fill(0.0);
    motor_enabled_.fill(1.0);
    motor_ready_.fill(1.0);
    motor_board_enabled_.fill(1.0);
    motor_board_errors_.fill(1.0);
    
    map.at("motor_target_currents") = motor_target_currents_;
    map.at("motor_torques") = motor_torques_;
    map.at("motor_target_torques") = motor_target_torques_;
    map.at("motor_encoder_indexes") = motor_encoder_indexes_;
    map.at("joint_positions") = joint_positions_;
    map.at("joint_velocities") = joint_velocities_;
    map.at("joint_torques") = joint_torques_;
    map.at("joint_target_torques") = joint_target_torques_;
    map.at("contact_sensors") = contact_sensors_;
    map.at("slider_positions") = slider_positions_;
    map.at("motor_enabled") = motor_enabled_;
    map.at("motor_ready") = motor_ready_;
    map.at("motor_board_enabled") = motor_board_enabled_;
    map.at("motor_board_errors") = motor_board_errors_;
  }

  void DGMQuadrupedSimu::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    ctrl_joint_torques_ = map.at("ctrl_joint_torques");
  }

} // namespace dg_blmc_robots
