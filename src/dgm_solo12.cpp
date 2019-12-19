/**
 * \file dgm_solo.cpp
 * \brief The hardware wrapper of the solo robot
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include <dynamic_graph_manager/ros_init.hh>
#include "dg_blmc_robots/dgm_solo12.hpp"

namespace dg_blmc_robots
{

  DGMSolo12::DGMSolo12()
  {
    was_in_safety_mode_ = false;
  }

  DGMSolo12::~DGMSolo12()
  {
  }

  void DGMSolo12::initialize_hardware_communication_process()
  {
    /**
     * Load the calibration parameters
     */
    blmc_robots::Vector8d joint_index_to_zero;
    YAML::ReadParameter(params_["hardware_communication"]["calibration"],
                        "index_to_zero_angle", zero_to_index_angle_from_file_);

    // get the hardware communication ros node handle
    ros::NodeHandle& ros_node_handle = dynamic_graph::ros_init(
      dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);

    /** initialize the user commands */
    ros_user_commands_.push_back(ros_node_handle.advertiseService(
        "calibrate_joint_position",
        &DGMSolo12::calibrate_joint_position_callback, this));

    std::string network_id;
    YAML::ReadParameter(params_["hardware_communication"],
                        "network_id", network_id);

    solo_.initialize(network_id);
  }

//  bool DGMSolo12::is_in_safety_mode()
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

  void DGMSolo12::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
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
    map.at("slider_positions") = solo_.get_slider_positions();

    /**
     * Robot status
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled = map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    const std::array<bool, 12>& motor_enabled = solo_.get_motor_enabled();
    const std::array<bool, 12>& motor_ready = solo_.get_motor_ready();
    const std::array<bool, 6>& motor_board_enabled = solo_.get_motor_board_enabled();
    const std::array<int, 6>& motor_board_errors = solo_.get_motor_board_errors();

    for(size_t i=0 ; i<motor_enabled.size() ; ++i)
    {
      map_motor_enabled[i] = motor_enabled[i];
      map_motor_ready[i] = motor_ready[i];
    }
    for(size_t i=0 ; i<motor_board_enabled.size() ; ++i)
    {
      map_motor_board_enabled[i] = motor_board_enabled[i];
      map_motor_board_errors[i] = motor_board_errors[i];
    }
  }

  void DGMSolo12::set_motor_controls_from_map(
      const dynamic_graph::VectorDGMap& map)
  {
    try{
      // here we need to perform and internal copy. Otherwise the compilator
      // complains
      ctrl_joint_torques_ = map.at("ctrl_joint_torques");
      // Actually send the control to the robot
      solo_.send_target_joint_torque(ctrl_joint_torques_);
    }catch(const std::exception& e){
      rt_printf("DGMSolo12::set_motor_controls_from_map: "
                "Error sending controls, %s\n", e.what());
    }
  }

  bool DGMSolo12::calibrate_joint_position_callback(
    dg_blmc_robots::JointCalibration::Request& req,
    dg_blmc_robots::JointCalibration::Response& res)
  {
    // parse and register the command for further call.
    add_user_command(std::bind(&DGMSolo12::calibrate_joint_position, 
                     this, zero_to_index_angle_from_file_));

    // return whatever the user want
    res.sanity_check = true;
    
    // the service has been executed properly
    return true;
  }

  void DGMSolo12::calibrate_joint_position(
    const blmc_robots::Vector12d& zero_to_index_angle)
  {
    solo_.calibrate(zero_to_index_angle);
  }

} // namespace dg_blmc_robots
