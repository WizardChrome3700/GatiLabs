#pragma once
#include <Eigen/Dense>
#include <utility>
#include "BETRotor.hpp"

enum magnet_class {
    N42 = 1,
    N52 = 2,
    N55 = 3,
    N48 = 4
};

class BLDC_Motor {
private:
    BETRotor propeller;
    Eigen::Vector3d local_position; 
    int spin_direction;             // 1 for CCW, -1 for CW
    
    // State variables
    double current_omega;           // rad/s
    double previous_thrust;         // Memory for the BEMT inflow calculation
    double induced_velocity;       // Memory for the BEMT inflow calculation
    double thrust;
    double torque;

    // (Future variables will go here: Kv rating, motor inertia, internal resistance, etc.)
    double KV_rating; // RPM per Volt, for future use in voltage-to-omega integration
    double K_t_nominal; // Nominal torque constant, for future use in voltage-to-omega integration
    double phase_resistance; // Internal resistance of the motor phases, for future use in voltage-to-omega integration
    double no_load_current; // Current drawn at zero load, for future use in voltage-to-omega integration
    magnet_class magnet_grade; // For future use in determining max torque based on magnetic saturation
    double max_magnetic_torque;

    double voltage_input; // For future use in voltage-to-omega integration
    double current_draw;

    double motor_height;
    double motor_diameter;
    double motor_mass;
    double motor_MOI;
    double calculate_angular_acceleration(double test_omega, double drag_torque);

public:
    // Constructor
    BLDC_Motor() {};
    BLDC_Motor(const BETRotor& prop, const Eigen::Vector3d& pos, const int direction, double KV, double resistance, double no_load_current, double max_current_draw, magnet_class magnet_grade, double height, double diameter, double mass);

    void update(double drag_torque, double delta_time); // For future use in integrating omega based on voltage input and motor characteristics
    
    // Requests the aerodynamic forces based on its current state
    std::pair<Eigen::Vector3d, Eigen::Vector3d> get_forces_and_torques(const Eigen::Vector3d& body_airspeed, double air_density, double delta_motor_update_time);

    void set_voltage(double volts) {
        this->voltage_input = volts;
    }

    // Getters
    Eigen::Vector3d get_position() const;

    double get_current_thrust() const { return this->thrust; }
    double get_current_torque() const { return this->torque; }
    double get_omega() const { return this->current_omega; }
    double get_current() const { return this->current_draw; }
    double get_mass() const { return this->motor_mass; }
};