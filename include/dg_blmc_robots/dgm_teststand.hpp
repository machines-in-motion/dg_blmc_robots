/**
 * \file dgm_teststand.hh
 * \brief This file define the dynamic graph manager responsible of the control
 * of the teststand robot.
 * \author Julian Viereck
 * \date 2018
 *
 * This file define the dynamic graph manager responsible of the control
 * of the teststand robot.
 */

#ifndef DGM_TESTSTAND_HH
#define DGM_TESTSTAND_HH

#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <blmc_robots/teststand.hpp>

namespace dg_blmc_robots
{

  class DGMTeststand : public dynamic_graph::DynamicGraphManager
  {
  public:
    /**
     * @brief DemoSingleMotor is the constructor.
     */
    DGMTeststand();

    /**
     * @brief ~DemoSingleMotor is the destructor.
     */
    ~DGMTeststand();

    /**
     * @brief initialize_hardware_communication_process is the function that
     * initialize the hardware.
     */
    void initialize_hardware_communication_process();

    /**
     * @brief get_sensors_to_map acquieres the sensors data and feed it to the
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
    * @brief test_bench_ the real test bench hardware drivers.
    */
    blmc_robots::Teststand teststand_;

    /**
    * @brief ctrl_joint_torques_ the joint torques to be sent
    */
    Eigen::Vector2d ctrl_joint_torques_;
  };


} // namespace dg_blmc_robots

#endif // DGM_TEST_BENCH_8_MOTORS_HH
