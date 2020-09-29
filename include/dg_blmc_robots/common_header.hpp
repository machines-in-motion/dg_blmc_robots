/**
 * @file common_header.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau
 * @author Julian Viereck
 * @author Johannes Pfleging
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellshaft.
 */

#ifndef COMMON_HEADER_H
#define COMMON_HEADER_H

#include <blmc_drivers/devices/analog_sensor.hpp>
#include <blmc_drivers/devices/motor.hpp>

namespace blmc_motors
{
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;

}  // namespace blmc_motors

#endif  // COMMON_HEADER_H
