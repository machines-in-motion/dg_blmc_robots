/**
 * \file dg_quadruped_8_motors.cpp
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
    quadruped_.initialize();
  }

  void DGMQuadruped::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    try{
      quadruped_.acquire_sensors();

      /**
        * Motor data
        */
      map.at("motor_positions") = quadruped_.get_motor_positions();
      map.at("motor_velocities") = quadruped_.get_motor_velocities();
      map.at("motor_currents") = quadruped_.get_motor_currents();
      map.at("motor_target_currents") = quadruped_.get_motor_target_currents();
      map.at("motor_torques") = quadruped_.get_motor_torques();
      map.at("motor_target_torques") = quadruped_.get_target_motor_torques();
      map.at("motor_encoder_indexes") = quadruped_.get_motor_encoder_indexes();

      /**
        * Joint data
        */
      map.at("joint_positions") = quadruped_.get_joint_positions();
      map.at("joint_velocities") = quadruped_.get_joint_velocities();
      map.at("joint_torques") = quadruped_.get_joint_torques();
      map.at("joint_target_torques") = quadruped_.get_joint_target_torques();

      /**
        * Additional data
        */
      map.at("contact_sensors") = quadruped_.get_contact_sensors_states();
      map.at("slider_positions") = quadruped_.get_slider_positions();

    }catch(...){
      printf("Error in acquiring the sensors data\n");
      printf("Setting all of them 0.0\n");
      /**
        * Motor data
        */
      map.at("motor_positions").fill(0.0);
      map.at("motor_velocities").fill(0.0);
      map.at("motor_currents").fill(0.0);
      map.at("motor_target_currents").fill(0.0);
      map.at("motor_torques").fill(0.0);
      map.at("motor_target_torques").fill(0.0);
      map.at("motor_encoder_indexes").fill(0.0);

      /**
        * Joint data
        */
      map.at("joint_positions").fill(0.0);
      map.at("joint_velocities").fill(0.0);
      map.at("joint_torques").fill(0.0);
      map.at("joint_target_torques").fill(0.0);

      /**
        * Additional data
        */
      map.at("contact_sensors").fill(0.0);
      map.at("slider_positions").fill(0.0);
    }
  }

  void DGMQuadruped::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    try{
      ctrl_joint_torques_ = map.at("ctrl_joint_torques");
      quadruped_.send_target_joint_torque(ctrl_joint_torques_);
    }catch(...){
      printf("Error sending controls\n");
    }
  }

} // namespace dg_blmc_robots
