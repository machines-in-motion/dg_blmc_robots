/**
 * \file dgm_solo.cpp
 * \brief The hardware wrapper of the solo robot
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "dg_blmc_robots/dgm_solo.hpp"

namespace dg_blmc_robots
{

  DGMSolo::DGMSolo()
  {
    was_in_safety_mode_ = false;
  }

  DGMSolo::~DGMSolo()
  {
  }

  void DGMSolo::initialize_hardware_communication_process()
  {
    solo_.initialize();
  }

//  bool DGMSolo::is_in_safety_mode()
//  {
//    was_in_safety_mode_ |= solo_.get_joint_velocities().cwiseAbs().maxCoeff() > 100000003.875;
//    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode()) {
//      was_in_safety_mode_ = true;
//      printf("Killing robot because velocity limit exceeded...\n");
//      return true;
 //   } else {
//      return false;
//    }
//  }


  void DGMSolo::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    solo_.acquire_sensors();

    /**
      * Joint data
      */
    map.at("joint_positions") = solo_.get_joint_positions();
    map.at("joint_velocities") = solo_.get_joint_velocities();
    map.at("joint_torques") = solo_.get_joint_torques();
    map.at("joint_target_torques") = solo_.get_joint_target_torques();
    map.at("joint_encoder_index") = solo_.get_joint_encoder_index();

    /**
      * Additional data
      */
    map.at("contact_sensors") = solo_.get_contact_sensors_states();
    map.at("slider_positions") = solo_.get_slider_positions();

    /**
     * Robot status
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled = map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    const std::array<bool, 8>& motor_enabled = solo_.get_motor_enabled();
    const std::array<bool, 8>& motor_ready = solo_.get_motor_ready();
    const std::array<bool, 4>& motor_board_enabled = solo_.get_motor_board_enabled();
    const std::array<int, 4>& motor_board_errors = solo_.get_motor_board_errors();

    for(unsigned i=0 ; i<8 ; ++i)
    {
      map_motor_enabled[i] = motor_enabled[i];
      map_motor_ready[i] = motor_ready[i];
    }
    for(unsigned i=0 ; i<4 ; ++i)
    {
      map_motor_board_enabled[i] = motor_board_enabled[i];
      map_motor_board_errors[i] = motor_board_errors[i];
    }
  }

  void DGMSolo::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    try{
      // here we need to perform and internal copy. Otherwize the compilator
      // complains
      ctrl_joint_torques_ = map.at("ctrl_joint_torques");
      // Actually send the control to the robot
      solo_.send_target_joint_torque(ctrl_joint_torques_);
    }catch(const std::exception& e){
      rt_printf("DGMSolo::set_motor_controls_from_map: "
                "Error sending controls, %s\n", e.what());
    }
  }

} // namespace dg_blmc_robots
