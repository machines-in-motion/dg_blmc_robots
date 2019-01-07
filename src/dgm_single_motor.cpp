/**
 * \file dg_single_motor.cpp
 * \brief DGM wrapper around the single_motor "robot".
 * \author Julian Viereck
 * \date 2019
 */

#include "dg_blmc_robots/dgm_single_motor.hpp"

namespace dg_blmc_robots
{

  DGMSingleMotor::DGMSingleMotor(): was_in_safety_mode_(false)
  {
  }

  DGMSingleMotor::~DGMSingleMotor()
  {
  }

  void DGMSingleMotor::initialize_hardware_communication_process()
  {
    single_motor_.initialize();
  }

  // bool DGMSingleMotor::is_in_safety_mode()
  // {

  //   was_in_safety_mode_ |= single_motor_.get_joint_velocities().cwiseAbs().maxCoeff() > 10.;
  //   if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode()) {
  //     was_in_safety_mode_ = true;
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  void DGMSingleMotor::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    try{
      single_motor_.acquire_sensors();

      /**
        * Joint data
        */
      map.at("joint_positions") = single_motor_.get_joint_positions();
      map.at("joint_velocities") = single_motor_.get_joint_velocities();
      map.at("joint_torques") = single_motor_.get_joint_torques();
      map.at("joint_target_torques") = single_motor_.get_joint_target_torques();
      map.at("joint_encoder_index") = single_motor_.get_joint_encoder_index();

      /**
        * Additional data
        */
      map.at("slider_positions") = single_motor_.get_slider_positions();
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
      map.at("height_sensors").fill(0.0);

      map.at("ati_force").fill(0.0);
      map.at("ati_torque").fill(0.0);
    }
  }

  void DGMSingleMotor::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    try{
      ctrl_joint_torques_ = map.at("ctrl_joint_torques");
      single_motor_.send_target_joint_torque(ctrl_joint_torques_);
    }catch(...){
      printf("Error sending controls\n");
    }
  }

} // namespace dg_blmc_robots
