/**
* \file dg_teststand.cpp
* \brief DGM wrapper around the teststand robot.
* \author Julian Viereck
* \date 2018
*/

#include <dynamic_graph_manager/ros_init.hh>
#include "dg_blmc_robots/dgm_test_bench.hpp"

namespace dg_blmc_robots
{

    DGMTestBench::DGMTestBench(): was_in_safety_mode_(false)
    {
        /**
        * Motor data
        */
        motor_inertias_.setZero();
        motor_torque_constants_.setZero();

        for(unsigned i=0 ; i<motor_enabled_.size(); ++i)
        {
            motor_enabled_[i] = false;
            motor_ready_[i] = false;
        }

        for(unsigned i=0 ; i<motor_board_enabled_.size(); ++i)
        {
            motor_board_enabled_[0] = false;
            motor_board_errors_[0] = 0;
        }

        /**
          * Joint data
          */
        joint_positions_.setZero();
        joint_velocities_.setZero();
        joint_torques_.setZero();
        joint_target_torques_.setZero();
        joint_gear_ratios_.setZero();
        joint_zero_positions_.setZero();

        /**
          * Additional data
          */
        slider_positions_.setZero();
        motor_max_current_.setZero();

        /**
          * Setup some known data
          */

        // Max current in Amp
        motor_max_current_.fill(12);
        motor_torque_constants_.fill(0.025);
        motor_inertias_.fill(0.045);
        joint_gear_ratios_.fill(1.0);

        // calibration data
        zero_to_index_angle_.fill(0.0);
        index_angle_.fill(0.0);
        mechanical_calibration_ = false;
        initial_error_= 0.0;
    }

    DGMTestBench::~DGMTestBench()
    {
    }
    void DGMTestBench::initialization(){
        // initialize the communication with the can cards
        can_buses_ = std::make_shared<blmc_drivers::CanBus>("can0");
        can_motor_boards_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_);
        // MOTOR_HFE
        motors_[0] = std::make_shared<blmc_drivers::Motor> (can_motor_boards_, 0);
        // MOTOR_KFE
        motors_[1] = std::make_shared<blmc_drivers::Motor> (can_motor_boards_, 1);
        // wait until all board are ready and connected

        // JOINT_HFE
        joints_[0] = std::make_shared<blmc_robots::BlmcJointModule> (
                motors_[0], motor_torque_constants_[0], 1.0, 0.0, false,
                motor_max_current_[0]);
        // JOINT_KFE
        joints_[1] = std::make_shared<blmc_robots::BlmcJointModule> (
                motors_[1], motor_torque_constants_[1], 1.0, 0.0, true,
                motor_max_current_[1]);

        // can 1
        sliders_[0] =
                std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_, 0);
        sliders_[1] =
                std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_, 1);

        // Wait to make sure there is a first package when acquire_sensors() later.
        real_time_tools::Timer::sleep_sec(0.5);


        // wait until all board are ready and connected
        can_motor_boards_->wait_until_ready();

        // Compute the difference between the two encoders and save the data internally
        acquire_sensors();
        initial_error_ = 0;

    }

    void DGMTestBench::initialize_hardware_communication_process()
    {
        try{
            std::vector<double> zero_to_index_angle =
            params_["hardware_communication"]["calibration"]["zero_to_index_angle"].as<std::vector<double> >();
            assert(zero_to_index_angle.size() == zero_to_index_angle_from_file_.size());
            for(unsigned i=0; i<zero_to_index_angle_from_file_.size() ; ++i)
            {
                zero_to_index_angle_from_file_[i] = zero_to_index_angle[i];
            }
        }catch(...){
          throw std::runtime_error("Error in reading yaml param:"
                                   "[hardware_communication][calibration]"
                                   "[zero_to_index_angle]");
        }

        // get the hardware communication ros node handle
        ros::NodeHandle& ros_node_handle = dynamic_graph::ros_init(dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);

        /** initialize the user commands */
        ros_user_commands_.push_back(ros_node_handle.advertiseService(
            "calibrate", &DGMTestBench::calibrate_joint_position_callback, this));

        initialization();
    }

    bool DGMTestBench::is_in_safety_mode()
    {
        was_in_safety_mode_ |= get_joint_velocities().cwiseAbs().maxCoeff() > 10000.;
        if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode()) {
            was_in_safety_mode_ = true;
            return true;
        } else {
            return false;
        }
    }

    bool DGMTestBench::acquire_sensors()
    {
        try{
            /**
              * Joint data
              */

            // acquire the joint position
            joint_positions_(0) = joints_[0]->get_measured_angle();
            // acquire the joint position
            joint_positions_(1) = joints_[1]->get_measured_angle();
            // acquire the joint torques
            joint_torques_(0) = joints_[0]->get_measured_torque();
            // acquire the target joint torques
            joint_target_torques_(0) = joints_[0]->get_sent_torque();
            for (unsigned i=0 ; i<joints_.size() ; ++i) {
                // acquire the joint velocities
                joint_velocities_(i) = joints_[i]->get_measured_velocity();
                // acquire the joint index
                joint_encoder_index_(i) = joints_[i]->get_measured_index_angle();
            }
            /**
              * Additional data
              */
            // acquire the slider positions
            for (unsigned i=0 ; i < slider_positions_.size() ; ++i)
            {
                // acquire the slider
                slider_positions_(i) = sliders_[i]->get_measurement()->newest_element();
            }
        }catch(std::exception ex)
        {
            rt_printf("HARDWARE: Something went wrong during the sensor reading.\n");
            rt_printf("error is: %s\n", ex.what());
            return false;
        }
        return true;
    }

    void DGMTestBench::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
    {
        try{
            acquire_sensors();
            /**
            * Joint data
            */
            map.at("joint_positions") = get_joint_positions();
            map.at("joint_velocities") = get_joint_velocities();
            map.at("joint_torques") = get_joint_torques();
            map.at("joint_target_torques") = get_joint_target_torques();
            map.at("joint_encoder_index") = get_joint_encoder_index();

            /**
            * Additional data
            */
            map.at("slider_positions") = get_slider_positions();
        }catch(...){
            printf("Error in acquiring the sensors data\n");
            printf("Setting all of them 0.0\n");

            /**
            * Joint data
            */
            map.at("joint_positions").fill(0.0);
            map.at("joint_velocities").fill(0.0);
            map.at("joint_torques").fill(0.0);
            map.at("joint_target_torques").fill(0.0);
            map.at("joint_encoder_index").fill(0.0);

            /**
            * Additional data
            */
            map.at("slider_positions").fill(0.0);
        }
    }
    bool DGMTestBench::calibrate(std::array<double, 2>& zero_to_index_angle,
                              std::array<double, 2>& index_angle,
                              bool mechanical_calibration)
    {
        zero_to_index_angle_ = zero_to_index_angle;
        mechanical_calibration_ = mechanical_calibration;

        calibration_threads_[0].create_realtime_thread(calibrate_hfe, this);
        calibration_threads_[1].create_realtime_thread(calibrate_kfe, this);

        calibration_threads_[0].join();
        calibration_threads_[1].join();

        index_angle = index_angle_;
        return true;
    }
    void DGMTestBench::set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map)
    {
        try{
            ctrl_joint_torques_ = map.at("ctrl_joint_torques");
            joints_[0]->set_torque(ctrl_joint_torques_(0));
            joints_[0]->send_torque();
        }catch(...){
          printf("Error sending controls\n");
        }
    }

    bool DGMTestBench::calibrate_joint_position_callback(
    dg_blmc_robots::TeststandCalibration::Request& req,
    dg_blmc_robots::TeststandCalibration::Response& res)
    {
        // parse and register the command for further call.
        add_user_command(std::bind(&DGMTestBench::calibrate_joint_position,
                         this, req.mechanical_calibration, zero_to_index_angle_,
                         index_angle_));

        // return whatever the user want
        res.sanity_check = true;

        // the service has been executed properly
        return true;
    }

    void DGMTestBench::calibrate_joint_position(
    bool mechanical_calibration,
    std::array<double, 2>& zero_to_index_angle,
    std::array<double, 2>& index_angle)
    {
        index_angle.fill(0.0);
        if(mechanical_calibration)
        {
            zero_to_index_angle.fill(0.0);
        }else{
            zero_to_index_angle = zero_to_index_angle_from_file_;
        }

        calibrate(zero_to_index_angle, index_angle, mechanical_calibration);

        for(unsigned i=0 ; i<2 ; ++i)
        {
            rt_printf("zero_to_index_angle[%d] = %f\n", i, zero_to_index_angle[i]);
            rt_printf("index_angle[%d] = %f\n", i, index_angle[i]);
        }
    }
    bool DGMTestBench::send_target_joint_torque(
            const Eigen::Ref<blmc_robots::Vector2d> target_joint_torque)
    {
        for (unsigned i=0 ; i<joints_.size() ; ++i)
        {
            joints_[i]->set_torque(target_joint_torque(i));
            joints_[i]->send_torque();
        }
        return true;
    }

} // namespace dg_blmc_robots
