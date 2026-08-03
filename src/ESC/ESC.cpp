#include "ESC/ESC.hpp"

ESC::ESC(double mass, double max_current, double max_voltage, double resistance_ohms, Eigen::Vector3d position) 
    : mass(mass), max_current(max_current), max_voltage(max_voltage), resistance_ohms(resistance_ohms), current_throttle(0.0), position(position) {}

void ESC::set_throttle(double throttle) {
    if (throttle < 0.0) throttle = 0.0;
    if (throttle > 1.0) throttle = 1.0;
    this->current_throttle = throttle;
}

double ESC::calculate_output_voltage(double battery_voltage, double motor_current_draw) {
    // 1. The Magic Smoke Check (Safety Cut-off)
    if (motor_current_draw > this->max_current || battery_voltage > this->max_voltage) {
        return 0.0; // The ESC has burned out or shut down to protect itself
    }
    // 1. Calculate the ideal voltage based on throttle
    double ideal_voltage = this->current_throttle * battery_voltage;

    // 2. Calculate voltage drop due to internal resistance (I * R)
    double voltage_drop = motor_current_draw * this->resistance_ohms;

    // 3. Calculate final output voltage
    double output_voltage = ideal_voltage - voltage_drop;

    // 4. Clip to max voltage if necessary
    if (output_voltage > this->max_voltage) {
        output_voltage = this->max_voltage;
    }

    // 5. Ensure we don't return negative voltage
    if (output_voltage < 0.0) {
        output_voltage = 0.0;
    }
    this->current_voltage = output_voltage; // Store for telemetry
    return output_voltage;
}
