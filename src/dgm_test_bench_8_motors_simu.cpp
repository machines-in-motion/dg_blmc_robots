/**
 * \file dg_test_bench_8_motors_simu.cpp
 * \brief The simu wrapper of the test bench with 8 blmc motors
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "dg_blmc_robots/dgm_test_bench_8_motors_simu.hpp"

namespace dg_blmc_robots
{

  DGMTestBench8MotorsSimu::DGMTestBench8MotorsSimu()
  {
    motor_currents_.fill(0.0);
    motor_positions_.fill(0.0);
    motor_velocities_.fill(0.0);
    slider_positions_.fill(0.0);
  }

  void DGMTestBench8MotorsSimu::initialize_hardware_communication_process()
  {
    motors_inertia_ = params_["motor_I"].as<double>();
    motors_torque_constant_ = params_["motor_KT"].as<double>();
  }

  void DGMTestBench8MotorsSimu::get_sensors_to_map(
      dynamic_graph::VectorDGMap& map)
  {
    motor_currents_ = ctrl_motor_currents_;
    motor_positions_ = motor_positions_ +
                       control_period_sec_ * motor_velocities_ +
                       control_period_sec_ * control_period_sec_ * 0.5 *
                       motors_inertia_ * motors_torque_constant_ *
                       motor_currents_;

    motor_velocities_ = motor_velocities_ +
                        control_period_sec_ * motors_inertia_ *
                        motors_torque_constant_ * motor_currents_;
    slider_positions_.fill(0.0);

    map.at("joint_positions") = motor_positions_;
    map.at("joint_velocities") = motor_velocities_;
    map.at("joint_torques") = motor_currents_;
    map.at("slider_positions") = slider_positions_;
  }

  void DGMTestBench8MotorsSimu::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    ctrl_motor_currents_ = map.at("ctrl_joint_torques");
  }

} // namespace dg_blmc_robots
