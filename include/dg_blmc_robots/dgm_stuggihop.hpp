/**
 * \file dgm_teststand.hh
 * \brief This file define the dynamic graph manager responsible of the control
 * of the stuggihop robot.
 * \author Steve Heim
 * \date 2019
 *
 * This file define the dynamic graph manager responsible of the control
 * of the stuggihop robot.
 */

#ifndef DGM_STUGGIHOP_HH
#define DGM_STUGGIHOP_HH

#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <blmc_robots/stuggihop.hpp>

namespace dg_blmc_robots
{

  class DGMStuggihop : public dynamic_graph::DynamicGraphManager
  {
  public:
    /**
     * @brief DGMStuggihop is the constructor.
     */
    DGMStuggihop();

    /**
     * @brief ~DGMStuggihop is the destructor.
     */
    ~DGMStuggihop();

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
  private:

    /**
     * Entries for the real hardware.
     */

    /**
    * @brief test_bench_ the real test bench hardware drivers.
    */
    blmc_robots::Stuggihop stuggihop_;

    /**
    * @brief ctrl_joint_torques_ the joint torques to be sent
    */
    Eigen::Vector2d ctrl_joint_torques_;

    /**
     * @brief was_in_safety_mode_ Toggle to keep in safety mode once it was entered.
     */
    bool was_in_safety_mode_;
  };


} // namespace dg_blmc_robots

#endif // DGM_STUGGIHOP_HH
