/**
 * \file demo_dg_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include "dg_blmc_robots/dgm_teststand.hpp"

int main(int , char* []) {
  std::cout << "Loading paramters from "
            << YAML_PARAMS
            << std::endl;
  YAML::Node param = YAML::LoadFile(YAML_PARAMS);
  dg_blmc_robots::DGMTeststand dgm;

  dgm.initialize(param);
  dgm.run();
  std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
  ros::waitForShutdown();
}

