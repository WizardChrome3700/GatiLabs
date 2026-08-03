#include "BLDC_Motor.hpp"

BLDC_Motor::BLDC_Motor(const BETRotor& prop, const Eigen::Vector3d& pos, const int direction, double KV, double resistance, double no_load_current, double max_current_draw, magnet_class magnet_grade, double height, double diameter, double mass) 
    : propeller(prop), local_position(pos), spin_direction(direction), 
      current_omega(0.0), previous_thrust(0.0), induced_velocity(0.0), thrust(0.0),
      KV_rating(KV), phase_resistance(resistance), no_load_current(no_load_current), magnet_grade(magnet_grade),
      motor_height(height), motor_diameter(diameter), motor_mass(mass)
{
    // The body is now empty because the initializer list did all the work!
    K_t_nominal = 60.0 / (2.0 * M_PI * KV); // Torque constant in Nm/A, derived from KV rating
    double B;
    switch (magnet_grade) {
        case N42:
            B = 1.3 * 0.3; // Placeholder value, in Nm
            break;
        case N48:
            B = 1.4 * 0.3; // Placeholder value, in Nm
            break;
        case N52:
            B = 1.45 * 0.3; // Placeholder value, in Nm
            break;
        case N55:
            B = 1.5 * 0.3; // Placeholder value, in Nm
            break;
        default:
            B = 1.45 * 0.3; // Default placeholder value, in Nm
    }
    double magnetic_permeability = 4 * M_PI * 1e-7; // in T*m/A
    max_magnetic_torque = (B*B/magnetic_permeability) * (M_PI * pow((diameter/2.0), 2)) * height; // in Nm, simplified from the more complex formula for magnetic torque
    motor_MOI = 0.5 * motor_mass * pow((diameter/2.0), 2); // MOI for a solid cylinder, in kg*m^2
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> BLDC_Motor::get_forces_and_torques(const Eigen::Vector3d& body_airspeed, double air_density, double delta_motor_update_time) {
    Eigen::Vector3d forces, torques;
    forces.z() = this->thrust;
    std::tie(forces, torques) = this->propeller.calculate_aerodynamics(this->current_omega, body_airspeed, this->induced_velocity, this->thrust, this->spin_direction, air_density);
    double relaxation_factor = 0.05; 
    this->thrust = (relaxation_factor * forces.z()) + ((1.0 - relaxation_factor) * this->thrust);
    this->torque = (relaxation_factor * torques.z()) + ((1.0 - relaxation_factor) * this->torque);
    this->update(torques.z(), delta_motor_update_time); // Update motor state with drag torque and the specified time step
    return std::make_pair(forces, torques);
}

double BLDC_Motor::calculate_angular_acceleration(double test_omega, double drag_torque) {
    // 1. Calculate current draw based on the TEST omega
    double test_current_draw = (this->voltage_input - (test_omega * this->K_t_nominal)) / this->phase_resistance;
    if (test_current_draw < 0) test_current_draw = 0;

    // 2. Calculate electrical torque
    double torque_elec = 0.0;
    if (test_current_draw > this->no_load_current) {
        double torque_ideal = this->K_t_nominal * (test_current_draw - this->no_load_current);
        torque_elec = std::tanh(torque_ideal / this->max_magnetic_torque) * this->max_magnetic_torque;
    }

    // 3. Return the derivative (Angular Acceleration)
    // Note: We subtract std::abs(drag_torque) because drag ALWAYS opposes motion
    return (torque_elec - std::abs(drag_torque)) / this->motor_MOI;
}


void BLDC_Motor::update(double drag_torque, double delta_time) {
    // Save our starting omega for this frame
    double w_n = this->current_omega;

    // k1: Acceleration at the start of the timestep
    double k1 = calculate_angular_acceleration(w_n, drag_torque);

    // k2: Acceleration at the midpoint (using k1 to project halfway)
    double w_mid1 = w_n + (0.5 * delta_time * k1);
    double k2 = calculate_angular_acceleration(w_mid1, drag_torque);

    // k3: Acceleration at the midpoint (using k2 to project halfway)
    double w_mid2 = w_n + (0.5 * delta_time * k2);
    double k3 = calculate_angular_acceleration(w_mid2, drag_torque);

    // k4: Acceleration at the end of the timestep (using k3 to project all the way)
    double w_end = w_n + (delta_time * k3);
    double k4 = calculate_angular_acceleration(w_end, drag_torque);

    // RK4 Integration: Weighted average of the slopes
    this->current_omega += (delta_time / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // Prevent reverse rotation if drag overcomes inertia completely at low throttle
    if (this->current_omega < 0.0) {
        this->current_omega = 0.0;
    }

    // Optional: Log the actual current draw for this frame using the final omega
    // This is useful later when feeding the total current demand to the Battery class
    this->current_draw = (this->voltage_input - (this->current_omega * this->K_t_nominal)) / this->phase_resistance;
    if (this->current_draw < 0) this->current_draw = 0;
}

Eigen::Vector3d BLDC_Motor::get_position() const {
    return this->local_position;
}