/**
 * \file demo_dg_stuggihop.cpp
 * \brief
 * \author Steve Heim
 * \date 2019
 *
 * This file uses is the entry point for python demo files
 */

#include "dg_blmc_robots/dgm_stuggihop.hpp"

int main(int, char* [])
{
    std::cout << "Loading paramters from " << YAML_PARAMS << std::endl;
    YAML::Node param = YAML::LoadFile(YAML_PARAMS);
    dg_blmc_robots::DGMStuggihop dgm;

    dgm.initialize(param);
    dgm.run();
    std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
    ros::waitForShutdown();
    std::cout << "Shutdown is performed." << std::endl;
}
