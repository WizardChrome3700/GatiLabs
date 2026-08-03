#pragma once
#include <Eigen/Dense>

class ESC {
private:
    double mass;
    double max_current; // Maximum current the ESC can handle (Amps)
    double max_voltage; // Maximum voltage the ESC can handle (Volts)
    double resistance_ohms; // Internal resistance of the ESC (Ohms)
    double current_throttle; // Current throttle setting (0.0 to 1.0)
    double current_voltage; // Voltage currently being output to the motor, for telemetry
    Eigen::Vector3d position; // Position of the ESC on the quadcopter frame (for torque calculations)
public:
    ESC() {};
    ESC(double mass, double max_current, double max_voltage, double resistance_ohms, Eigen::Vector3d position);
    void set_throttle(double throttle); // Set throttle (0.0 to 1.0)
    double calculate_output_voltage(double battery_voltage, double motor_current_draw);
    double get_mass() const { return this->mass; }
    double get_current_voltage() const { return this->current_voltage; }
};