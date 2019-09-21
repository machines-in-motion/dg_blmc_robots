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
#include "dg_blmc_robots/TeststandCalibration.h"
#include <blmc_robots/blmc_joint_module.hpp>
#include "blmc_robots/common_header.hpp"

namespace dg_blmc_robots
{
  class DGMTestBench : public dynamic_graph::DynamicGraphManager
  {
  public:
    /**
     * @brief DGMTeststand is the constructor.
     */
    DGMTestBench();

    /**
     * @brief ~DGMTeststand is the destructor.
     */
    ~DGMTestBench();

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








      /**
      * @brief get_slider_positions
      * WARNING !!!! The method acquire_sensors() has to be called prior to
      * any getter to have up to date data.
      *
      * @return the current sliders positions.
      */
      const Eigen::Ref<blmc_robots::Vector2d> get_slider_positions()
      {
          return slider_positions_;
      }
      /**
      * @brief send_target_torques sends the target currents to the motors
      */
      bool send_target_joint_torque(const Eigen::Ref<blmc_robots::Vector2d> target_joint_torque);

      void initialization();

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
    std::array<blmc_robots::Motor_ptr, 2> motors_;
    /**
    * @brief joints_ are the objects allowing us to send commands and receive
    * data at the joint level. It also ones some self calibration routines.
    */
    std::array<blmc_robots::BlmcJointModule_ptr, 2> joints_;
    /**
    * @brief This function will run a small controller that will move the joints
    * untils the next joint index and reset the joint zero with this knowledge.
    *
    * @return true if success
    * @return false if failure
    */
    bool calibrate(std::array<double, 2>& ,
            std::array<double, 2>& ,
            bool mechanical_calibration = false);
    /**
    * @brief Threads to calibrate all joints at the same time.
    */
    std::array<real_time_tools::RealTimeThread, 2> calibration_threads_;
    bool mechanical_calibration_;
    bool calibrate_one_joint(int joint_index)
    {
        return joints_[joint_index]->calibrate(zero_to_index_angle_[joint_index],
                index_angle_[joint_index],mechanical_calibration_);
    }
    static void* calibrate_hfe(void *context)
    {
        static_cast<DGMTestBench *>(context)->calibrate_one_joint(0);
        return nullptr;
    }

    static void* calibrate_kfe(void *context)
    {
        static_cast<DGMTestBench *>(context)->calibrate_one_joint(1);
        return nullptr;
    }
    bool acquire_sensors();
    /**
    * Joint data
    */
    /**
    * @brief joint_positions_ is the measured data from the onboard card
    * converted at the joint level.
    */
    blmc_robots::Vector2d joint_positions_;
    /**
    * @brief joint_velocities_ is the measured data from the onboard card
    * converted at the joint level.
    */
    blmc_robots::Vector2d joint_velocities_;
    /**
    * @brief joint_torques_ is the measured data from the onboard card
    * converted at the joint level.
    */
    blmc_robots::Vector1d joint_torques_;
    /**
    * @brief joint_target_torques_ is the last given command to be sent.
    */
    blmc_robots::Vector1d joint_target_torques_;
    /**
    * @brief joint_gear_ratios are the joint gear ratios
    */
    blmc_robots::Vector2d joint_gear_ratios_;
    /**
    * @brief joint_encoder_index_ The last observed encoder_index at the joints.
    */
    blmc_robots::Vector2d joint_encoder_index_;

    /**
    * @brief joint_zero_positions_ is the configuration considered as zero
    * position
    */
    blmc_robots::Vector2d joint_zero_positions_;/**
    * Additional data
    */
    /**
    * @brief slider_positions_ is the position of the linear potentiometer.
    * Can be used as a joystick input.
    */
    blmc_robots::Vector2d slider_positions_;
    /**
    * @brief sliders_ these are analogue input from linear potentiometers.
    */
    std::array<blmc_robots::Slider_ptr, 2> sliders_;
    /**
    * @brief get_joint_positions
    * WARNING !!!! The method acquire_sensors() has to be called prior to
    * any getter to have up to date data.
    *
    * @return  the joint angle of each module
    */
    Eigen::Ref<blmc_robots::Vector2d> get_joint_positions()
    {
        return joint_positions_;
    }

    /**
    * @brief get_joint_velocities
    * WARNING !!!! The method acquire_sensors() has to be called prior to
    * any getter to have up to date data.
    *
    * @return the joint velocities
    */
    Eigen::Ref<blmc_robots::Vector2d> get_joint_velocities()
    {
        return joint_velocities_;
    }

    /**
    * @brief get_joint_torques
    * WARNING !!!! The method acquire_sensors() has to be called prior to
    * any getter to have up to date data.
    *
    * @return the joint torques
    */
    Eigen::Ref<blmc_robots::Vector1d> get_joint_torques()
    {
        return joint_torques_;
    }

    /**
    * @brief get_joint_torques
    * @return the target joint torques
    */
    Eigen::Ref<blmc_robots::Vector1d> get_joint_target_torques()
    {
        return joint_target_torques_;
    }

    /**
    * @brief get_joint_gear_ratios
    * @return  the joint gear ratios
    */
    Eigen::Ref<blmc_robots::Vector2d> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
    * @brief get_joint_encoder_index
    * WARNING !!!! The method acquire_sensors() has to be called prior to
    * any getter to have up to date data.
    *
    * @return The last observed encoder index in joint coordinates.
    */
    Eigen::Ref<blmc_robots::Vector2d> get_joint_encoder_index()
    {
        return joint_encoder_index_;
    }

    /**
    * @brief get_zero_positions
    * @return the position where the robot should be in "zero" configuration
    */
    Eigen::Ref<blmc_robots::Vector2d> get_zero_positions()
    {
        return joint_zero_positions_;
    }

    /**
    * @brief motor_inertias_
    */
    blmc_robots::Vector2d motor_inertias_;
    /**
    * @brief motor_torque_constants_ are the motor torque constants
    */
    blmc_robots::Vector2d motor_torque_constants_;
    /**
    * @brief max_current_ is the maximum current that can be sent to the motors,
    * this a safe guard for development
    */
    blmc_robots::Vector2d motor_max_current_;

    /**
    * @brief This gives the status (enabled/disabled) of each motors using the
    * joint ordering convention.
    */
    std::array<bool, 4> motor_enabled_;

    /**
    * @brief This gives the status (enabled/disabled) of each motors using the
    * joint ordering convention.
    */
    std::array<bool, 4> motor_ready_;

    /**
    * @brief This gives the status (enabled/disabled of the onboard control cards)
    */
    std::array<bool, 2> motor_board_enabled_;

    /**
    * @brief This gives the status (enabled/disabled of the onboard control cards)
    */
    std::array<int, 2> motor_board_errors_;
    /**
    * @brief This gives the difference between two encoders
    */
    double initial_error_;
    /**
    * @brief ctrl_joint_torques_ the joint torques to be sent
    */
    blmc_robots::Vector1d ctrl_joint_torques_;







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
