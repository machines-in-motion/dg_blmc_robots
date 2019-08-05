/**
 * @file dgm_test_bench_8_motors_simu.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau 
 * @author Julian Viereck
 * @author Johannes Pfleging 
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 */

#ifndef DGM_TEST_BENCH_8_MOTORS_HH
#define DGM_TEST_BENCH_8_MOTORS_HH

#include <vector>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <blmc_robots/test_bench_8_motors.hpp>

namespace dg_blmc_robots
{

  class DGMTestBench8MotorsSimu : public dynamic_graph::DynamicGraphManager
  {
  public:
    /**
     * @brief DemoSingleMotor is the constructor.
     */
    DGMTestBench8MotorsSimu();

    /**
     * @brief ~DemoSingleMotor is the destructor.
     */
    ~DGMTestBench8MotorsSimu(){}

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
     * Entries for the simulated hardware.
     */
    blmc_robots::Vector8d ctrl_motor_currents_;
    blmc_robots::Vector8d motor_positions_;
    blmc_robots::Vector8d motor_velocities_;
    blmc_robots::Vector8d motor_currents_;
    blmc_robots::Vector8d slider_positions_;

    double motors_inertia_;
    double motors_torque_constant_;
  };


} // namespace dg_blmc_robots

#endif // DGM_TEST_BENCH_8_MOTORS_HH
