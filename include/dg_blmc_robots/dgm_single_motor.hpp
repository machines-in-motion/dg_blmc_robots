/**
 * \file dgm_single_motor.hh
 * \brief This file defines the dynamic graph manager responsible of the control
 * of the single motor "robot".
 * \author Julian Viereck
 * \date 2019
 *
 * This file define the dynamic graph manager responsible of the control
 * of the single motor "robot".
 */

#ifndef DGM_SINGLE_MOTOR_HH
#define DGM_SINGLE_MOTOR_HH

#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <blmc_robots/single_motor.hpp>

namespace dg_blmc_robots
{

  typedef Eigen::Matrix<double, 1, 1> Vector1d;

  class DGMSingleMotor : public dynamic_graph::DynamicGraphManager
  {
  public:
    /**
     * @brief DGMSingleMotor is the constructor.
     */
    DGMSingleMotor();

    /**
     * @brief ~DGMSingleMotor is the destructor.
     */
    ~DGMSingleMotor();

    /**
     * @brief initialize_hardware_communication_process is the function that
     * initialize the hardware.
     */
    void initialize_hardware_communication_process();

    /**
     * @brief get_sensors_to_map acquires the sensors data and feed it to the
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
  private:

    /**
     * Entries for the real hardware.
     */

    /**
    * @brief single_motor_ the real motor hardware drivers.
    */
    blmc_robots::SingleMotor single_motor_;

    /**
    * @brief ctrl_joint_torques_ the motor torque to be sent
    */
    Vector1d ctrl_joint_torques_;

    /**
     * @brief was_in_safety_mode_ Toggle to keep in safety mode once it was entered.
     */
    bool was_in_safety_mode_;
  };


} // namespace dg_blmc_robots

#endif // DGM_SINGLE_MOTOR_HH
