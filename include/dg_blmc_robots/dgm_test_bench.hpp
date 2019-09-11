/**
 * @file dgm_teststandx.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau 
 * @author Julian Viereck
 * @author Johannes Pfleging 
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 */

#ifndef DGM_TESTSTAND_HH
#define DGM_TESTSTAND_HH

#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <blmc_robots/teststand.hpp>
#include "dg_blmc_robots/TeststandCalibration.h"
#include "blmc_robots/common_header.hpp"

namespace dg_blmc_robots
{

  class DGMTeststand : public dynamic_graph::DynamicGraphManager
  {
  public:
    /**
     * @brief DGMTeststand is the constructor.
     */
    DGMTeststand();

    /**
     * @brief ~DGMTeststand is the destructor.
     */
    ~DGMTeststand();

    /**
     * @brief initialize_hardware_communication_process is the function that
     * initialize the hardware.
     */
    void initialize_hardware_communication_process();

    /**
     * @brief get_sensors_to_map acquires the sensors data and feeds it to the
     * input/output map
     * @param[in][out] map is the sensors data filled by this function.
     */
    void get_sensors_to_map(dynamic_graph::VectorDGMap& map);

    /**
     * @brief set_motor_controls_from_map reads the input map that contains the
     * controls and send these controls to the hardware.
     * @param map
     */
    void set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map);

    /**
     * @brief is_in_safety_mode Implement custom safe-mode detection.
     */
    virtual bool is_in_safety_mode();

    bool calibrate_joint_position_callback(
      dg_blmc_robots::TeststandCalibration::Request& req,
      dg_blmc_robots::TeststandCalibration::Response& res);

  private:
    /**
     * @brief Calibrate the robot joint position
     * 
     * @param soft or mechanical calibration?
     * @param zero_to_index_angle is the angle between the theoretical zero and
     * the next positive angle.
     * @param index_angle is the positition of the next index.
     */
    void calibrate_joint_position(
      bool mechanical_calibration,
      std::array<double, 2>& zero_to_index_angle,
      std::array<double, 2>& index_angle);

    /**
     * Entries for the real hardware.
     */

    /**
      * @brief test_bench_ the real test bench hardware drivers.
      */
    blmc_robots::Teststand teststand_;//////////////////


    void initialization();
    /**
    * @brief can_buses_ are the 2 can buses on the robot.
    */
    blmc_robots::CanBus_ptr can_buses_;
    /**
    * @brief can_motor_boards_ are the 2 can motor board.
    */
    blmc_robots::CanBusMotorBoard_ptr can_motor_boards_;
    /**
    * @brief motors_ are the objects allowing us to send motor commands and
    * receive data
    */
    blmc_robots::Motor_ptr motors_;


    /**
    * @brief ctrl_joint_torques_ the joint torques to be sent
    */
    Eigen::Vector2d ctrl_joint_torques_;

    /**
     * @brief These are the calibration value extracted from the paramters.
     * They represent the distance between the theorical zero joint angle and
     * the next jont index.
     */
    std::array<double, 2> zero_to_index_angle_from_file_;

    /**
     * @brief Results of the calibration.
     */
    std::array<double, 2> zero_to_index_angle_;
    
    /**
     * @brief Results of the calibration.
     */
    std::array<double, 2> index_angle_;

    /**
     * @brief was_in_safety_mode_ Toggle to keep in safety mode once it was entered.
     */
    bool was_in_safety_mode_;
  };


} // namespace dg_blmc_robots

#endif // DGM_TEST_BENCH_8_MOTORS_HH