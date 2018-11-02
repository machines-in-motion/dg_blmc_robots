/**
 * \file dg_test_bench_8_motors.cpp
 * \brief The hardware wrapper of the the test bench with 8 blmc motors
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "dg_blmc_robots/test_bench_8_motors/dgm_test_bench_8_motors.hh"

namespace dg_blmc_robots
{

  DGMTestBench8Motors::DGMTestBench8Motors()
  {
  }

  void DGMTestBench8Motors::initialize_hardware_communication_process()
  {
    test_bench_.initialize();
  }

  void DGMTestBench8Motors::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    test_bench_.acquire_sensors();
    map.at("motor_positions") = test_bench_.get_motor_positions();
    map.at("motor_velocities") = test_bench_.get_motor_velocities();
    map.at("motor_currents") = test_bench_.get_motor_currents();
    map.at("slider_positions") = test_bench_.get_slider_positions();
  }

  void DGMTestBench8Motors::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    test_bench_.send_target_current(map.at("torques"));
  }

} // namespace dg_blmc_robots
