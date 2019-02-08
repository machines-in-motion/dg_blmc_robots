/**
 * \file dg_teststand.cpp
 * \brief DGM wrapper around the teststand robot.
 * \author Julian Viereck
 * \date 2018
 */

#include "dg_blmc_robots/dgm_teststand.hpp"

namespace dg_blmc_robots
{

  DGMTeststand::DGMTeststand(): was_in_safety_mode_(false)
  {
  }

  DGMTeststand::~DGMTeststand()
  {
  }

  void DGMTeststand::initialize_hardware_communication_process()
  {
    teststand_.initialize();
  }

  bool DGMTeststand::is_in_safety_mode()
  {
    was_in_safety_mode_ |= teststand_.get_joint_velocities().cwiseAbs().maxCoeff() > 10.;
    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode()) {
      was_in_safety_mode_ = true;
      return true;
    } else {
      return false;
    }
  }

  void DGMTeststand::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    try{
      teststand_.acquire_sensors();

      /**
        * Joint data
        */
      map.at("joint_positions") = teststand_.get_joint_positions();
      map.at("joint_velocities") = teststand_.get_joint_velocities();
      map.at("joint_torques") = teststand_.get_joint_torques();
      map.at("joint_target_torques") = teststand_.get_joint_target_torques();
      map.at("joint_encoder_index") = teststand_.get_joint_encoder_index();

      /**
        * Additional data
        */
      map.at("contact_sensors") = teststand_.get_contact_sensors_states();
      map.at("slider_positions") = teststand_.get_slider_positions();
      map.at("height_sensors") = teststand_.get_height_sensors();

      map.at("ati_force") = teststand_.get_ati_force();
      map.at("ati_torque") = teststand_.get_ati_torque();
    }catch(...){
      printf("Error in acquiring the sensors data\n");
      printf("Setting all of them 0.0\n");

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

  void DGMTeststand::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    try{
      ctrl_joint_torques_ = map.at("ctrl_joint_torques");
      teststand_.send_target_joint_torque(ctrl_joint_torques_);
    }catch(...){
      printf("Error sending controls\n");
    }
  }

} // namespace dg_blmc_robots
