/**
 * \file dgm_stuggihop.cpp
 * \brief DGM wrapper around the stuggihop robot.
 * \author Steve Heim
 * \date 2019
 */

#include "dg_blmc_robots/dgm_stuggihop.hpp"

namespace dg_blmc_robots
{

  DGMStuggihop::DGMStuggihop(): was_in_safety_mode_(false)
  {
  }

  DGMStuggihop::~DGMStuggihop()
  {
  }

  void DGMStuggihop::initialize_hardware_communication_process()
  {
    stuggihop_.initialize();
  }

  bool DGMStuggihop::is_in_safety_mode()
  {
    was_in_safety_mode_ |= stuggihop_.get_joint_velocities().cwiseAbs().maxCoeff() > 10.;
    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode()) {
      was_in_safety_mode_ = true;
      return true;
    } else {
      return false;
    }
  }

  void DGMStuggihop::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    try{
      stuggihop_.acquire_sensors();

      /**
        * Joint data
        */
      map.at("joint_positions") = stuggihop_.get_joint_positions();
      map.at("joint_velocities") = stuggihop_.get_joint_velocities();
      map.at("joint_torques") = stuggihop_.get_joint_torques();
      map.at("joint_target_torques") = stuggihop_.get_joint_target_torques();
      map.at("joint_encoder_index") = stuggihop_.get_joint_encoder_index();

      /**
        * Additional data
        */
      // map.at("contact_sensors") = stuggihop_.get_contact_sensors_states();
      // map.at("slider_positions") = stuggihop_.get_slider_positions();
      // map.at("height_sensors") = stuggihop_.get_height_sensors();

      // map.at("ati_force") = stuggihop_.get_ati_force();
      // map.at("ati_torque") = stuggihop_.get_ati_torque();
      map.at("base_positions")  = stuggihop_.get_base_positions();
      map.at("base_velocities") = stuggihop_.get_base_velocities();

    }catch(...){
      printf("Error in acquiring the sensors data\n");
      printf("Setting all of them 0.0\n");
      // /**
      //   * Motor data
      //   * TODO: erroneous?
      //   */
      // map.at("motor_positions").fill(0.0);
      // map.at("motor_velocities").fill(0.0);
      // map.at("motor_currents").fill(0.0);
      // map.at("motor_target_currents").fill(0.0);
      // map.at("motor_torques").fill(0.0);
      // map.at("motor_target_torques").fill(0.0);
      // map.at("motor_encoder_indexes").fill(0.0);

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

      map.at("base_positions").fill(0.0);
      map.at("base_velocities").fill(0.0);

    }
  }

  void DGMStuggihop::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    try{
      ctrl_joint_torques_ = map.at("ctrl_joint_torques");
      stuggihop_.send_target_joint_torque(ctrl_joint_torques_);
    }catch(...){
      printf("Error sending controls\n");
    }
  }

} // namespace dg_blmc_robots
