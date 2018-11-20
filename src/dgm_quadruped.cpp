/**
 * \file dg_test_bench_8_motors.cpp
 * \brief The hardware wrapper of the the test bench with 8 blmc motors
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "dg_blmc_robots/dgm_quadruped.hpp"

namespace dg_blmc_robots
{

  DGMQuadruped::DGMQuadruped()
  {
  }

  DGMQuadruped::~DGMQuadruped()
  {
  }

  void DGMQuadruped::initialize_hardware_communication_process()
  {
//    test_bench_.initialize();
  }

  void DGMQuadruped::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
//    try{
//      test_bench_.acquire_sensors();
//      map.at("motor_positions") = test_bench_.get_motor_positions();
//      map.at("motor_velocities") = test_bench_.get_motor_velocities();
//      map.at("motor_currents") = test_bench_.get_motor_currents();
//      map.at("slider_positions") = test_bench_.get_slider_positions();
//    }catch(...){
//      printf("Error in acquiring the sensors data\n");
//      map.at("motor_positions").fill(0.0);
//      map.at("motor_velocities").fill(0.0);
//      map.at("motor_currents").fill(0.0);
//      map.at("slider_positions").fill(0.0);
//    }
  }

  void DGMQuadruped::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
//    try{
//      test_bench_.send_target_current(map.at("ctrl_motor_currents"));
//    }catch(...){
//      printf("Error sending controls\n");
//    }
  }

} // namespace dg_blmc_robots
