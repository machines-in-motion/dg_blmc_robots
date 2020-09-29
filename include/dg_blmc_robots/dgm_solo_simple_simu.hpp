/**
 * @file dgm_quadruped_simu.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau
 * @author Julian Viereck
 * @author Johannes Pfleging
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellshaft.
 */

#ifndef DGM_QUADRUPED_HH
#define DGM_QUADRUPED_HH

#include "dynamic_graph_manager/dynamic_graph_manager.hpp"

namespace dg_blmc_robots
{
/**
 * @brief Vector8d shortcut for the eigen vector of size 8.
 */
typedef Eigen::Matrix<double, 8, 1> Vector8d;

/**
 * @brief Vector8d shortcut for the eigen vector of size 4.
 */
typedef Eigen::Matrix<double, 8, 1> Vector4d;

class DGMQuadrupedSimu : public dynamic_graph_manager::DynamicGraphManager
{
public:
    /**
     * @brief DemoSingleMotor is the constructor.
     */
    DGMQuadrupedSimu();

    /**
     * @brief ~DemoSingleMotor is the destructor.
     */
    ~DGMQuadrupedSimu();

    /**
     * @brief This function make also sure that the joint velocity do not exceed
     * a certain value
     */
    bool is_in_safety_mode();

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
    void get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map);

    /**
     * @brief set_motor_controls_from_map reads the input map that contains the
     * controls and send these controls to the hardware.
     * @param map
     */
    void set_motor_controls_from_map(
        const dynamic_graph_manager::VectorDGMap& map);

private:
    /**
     * Entries for the simulated hardware.
     */
    Vector8d motor_target_currents_;
    Vector8d motor_torques_;
    Vector8d motor_target_torques_;
    Vector8d motor_encoder_indexes_;

    Vector8d joint_positions_;
    Vector8d joint_velocities_;
    Vector8d joint_torques_;
    Vector8d joint_target_torques_;

    Vector4d contact_sensors_;
    Vector4d slider_positions_;

    Vector8d motor_enabled_;
    Vector8d motor_ready_;

    Vector4d motor_board_enabled_;
    Vector4d motor_board_errors_;

    Vector8d ctrl_joint_torques_;

    double motors_inertia_;
    double motors_torque_constant_;
    double motors_gear_ratio_;
};

}  // namespace dg_blmc_robots

#endif  // DGM_TEST_BENCH_8_MOTORS_HH
