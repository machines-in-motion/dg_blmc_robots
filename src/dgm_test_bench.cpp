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

    DGMTeststand::DGMTeststand(): was_in_safety_mode_(false)
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
        joint_gear_ratios_.fill(9.0);

        // calibration data
        zero_to_index_angle_.fill(0.0);
        index_angle_.fill(0.0);
        mechanical_calibration_ = false;
    }

    DGMTeststand::~DGMTeststand()
    {
    }
    void DGMTeststand::initialization(){
        // initialize the communication with the can cards
        can_buses_ = std::make_shared<blmc_drivers::CanBus>("can0");
        can_motor_boards_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_);
        // MOTOR_HFE
        motors_[0] = std::make_shared<blmc_drivers::Motor> (can_motor_boards_, 1);
        // MOTOR_KFE
        motors_[1] = std::make_shared<blmc_drivers::Motor> (can_motor_boards_, 0);
        // wait until all board are ready and connected

        // JOINT_HFE
        joints_[0] = std::make_shared<blmc_robots::BlmcJointModule> (
                motors_[0], motor_torque_constants_[0], joint_gear_ratios_[0], 0.0, false,
                motor_max_current_[0]);
        // JOINT_KFE
        joints_[1] = std::make_shared<blmc_robots::BlmcJointModule> (
                motors_[1], motor_torque_constants_[1], joint_gear_ratios_[1], 0.0, false,
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


    }

    void DGMTeststand::initialize_hardware_communication_process()
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
            "calibrate", &DGMTeststand::calibrate_joint_position_callback, this));

        initialization();
    }

    bool DGMTeststand::is_in_safety_mode()
    {
        was_in_safety_mode_ |= get_joint_velocities().cwiseAbs().maxCoeff() > 10000.;
        if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode()) {
            was_in_safety_mode_ = true;
            return true;
        } else {
            return false;
        }
    }

    bool DGMTeststand::acquire_sensors()
    {
        try{
            /**
              * Joint data
              */
            for (unsigned i=0 ; i<joints_.size() ; ++i) {
                // acquire the joint position
                joint_positions_(i) = joints_[i]->get_measured_angle();
                // acquire the joint velocities
                joint_velocities_(i) = joints_[i]->get_measured_velocity();
                // acquire the joint torques
                joint_torques_(i) = joints_[i]->get_measured_torque();
                // acquire the joint index
                joint_encoder_index_(i) = joints_[i]->get_measured_index_angle();
                // acquire the target joint torques
                joint_target_torques_(i) = joints_[i]->get_sent_torque();
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

    void DGMTeststand::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
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
    bool DGMTeststand::calibrate(std::array<double, 2>& zero_to_index_angle,
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
    void DGMTeststand::set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map)
    {
        try{
            ctrl_joint_torques_ = map.at("ctrl_joint_torques");
            for (unsigned i=0 ; i<joints_.size() - 1 ; ++i)
            {
                joints_[i]->set_torque(ctrl_joint_torques_(i));
                joints_[i]->send_torque();
            }
        }catch(...){
          printf("Error sending controls\n");
        }
    }

    bool DGMTeststand::calibrate_joint_position_callback(
    dg_blmc_robots::TeststandCalibration::Request& req,
    dg_blmc_robots::TeststandCalibration::Response& res)
    {
        // parse and register the command for further call.
        add_user_command(std::bind(&DGMTeststand::calibrate_joint_position,
                         this, req.mechanical_calibration, zero_to_index_angle_,
                         index_angle_));

        // return whatever the user want
        res.sanity_check = true;

        // the service has been executed properly
        return true;
    }

    void DGMTeststand::calibrate_joint_position(
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
    bool DGMTeststand::send_target_joint_torque(
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

std::atomic_bool StopDemos (false);

void my_handler(int s){
    StopDemos = true;
}

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    dg_blmc_robots::DGMTeststand& robot = *(static_cast<dg_blmc_robots::DGMTeststand*>(robot_void_ptr));

    blmc_robots::Vector2d desired_torque;


    std::vector<std::deque<double> > sliders_filt_buffer(robot.get_slider_positions().size());
    int max_filt_dim = 200;
    for(unsigned i=0 ; i<sliders_filt_buffer.size() ; ++i)
    {
        sliders_filt_buffer[i].clear();
    }
    size_t count = 0;
    bool success_acquiring_sensor = true;
    bool success_sending_torques = true;
    while(!StopDemos && success_acquiring_sensor && success_sending_torques)
    {

    }//endwhile
    // send zero torques after the control loop.
    desired_torque.fill(0.0);
    robot.send_target_joint_torque(desired_torque);
    StopDemos = true;

    return THREAD_FUNCTION_RETURN_VALUE;
}// end control_loop

/*int main(int argc, char **argv)
{
    std::cout << "!!!!!!!!";
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    real_time_tools::RealTimeThread thread;

    dg_blmc_robots::DGMTeststand robot;

    robot.initialization();

    rt_printf("controller is set up \n");

    thread.create_realtime_thread(&control_loop, &robot);

    // Wait until the application is killed.
    while(!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }
    thread.join();

    rt_printf("Exit cleanly \n");

    return 0;
}*/