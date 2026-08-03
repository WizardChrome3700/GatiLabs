#include "Vehicles/Quadcopter.hpp"
#include <fstream> // For logging

// ---------------------------------------------------------
// Constructor
// Passes mass and MOI up to the Vehicle base class, 
// and stores our specific Powertrain components.
// ---------------------------------------------------------
Quadcopter::Quadcopter(double mass, Eigen::Matrix3d MOI, 
                       const LipoBattery& batt, 
                       const std::array<ESC, 4>& esc_array, 
                       const std::array<BLDC_Motor, 4>& motor_array) 
    // 3. Initialize the Base Class first, then the members!
    : battery(batt), escs(esc_array), motors(motor_array), Vehicle(calculate_total_mass(mass, batt, esc_array, motor_array), MOI)
{
    this->motor_commands.fill(0.0); 
}

std::array<BLDC_Motor, 4> Quadcopter::get_motors() const {
    return this->motors;
}

std::array<ESC, 4> Quadcopter::get_escs() const {
    return this->escs;
}

LipoBattery Quadcopter::get_battery() const {
    return this->battery;
}

// ---------------------------------------------------------
// Flight Controller Input
// ---------------------------------------------------------
void Quadcopter::set_motor_commands(const std::array<double, 4>& commands) {
    this->motor_commands = commands;
}

void Quadcopter::calculate_forces_torques(double dt) {
    
    // 1. Reset Accumulators for this frame
    this->clear_forces(); 
    this->current_torque.setZero(); 

    DroneState current_state = this->rigid_body.getState();
    Eigen::Vector3d body_airspeed = current_state.body_velocity; 
    double air_density = 1.225; 

    // 2. Global Power Demand
    double total_current_demand = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        total_current_demand += motors[i].get_current(); 
    }

    // 3. Global Power Supply
    battery.update(total_current_demand, dt);
    double global_voltage = battery.get_terminal_voltage();

    // 4. Powertrain Loop
    for (size_t i = 0; i < 4; ++i) {
        escs[i].set_throttle(this->motor_commands[i]);
        double effective_voltage = escs[i].calculate_output_voltage(global_voltage, motors[i].get_current());
        motors[i].set_voltage(effective_voltage);
        
        auto [motor_force, motor_torque] = motors[i].get_forces_and_torques(body_airspeed, air_density, dt);
        
        this->add_force_at_point(motor_force, motors[i].get_position());
        this->current_torque += motor_torque; 
        
    }
    
}

std::map<std::string, double> Quadcopter::get_telemetry(double timestamp) const {
    std::map<std::string, double> telemetry;
    telemetry["timestamp"] = timestamp;
    telemetry["position_x"] = this->get_position().x();
    telemetry["position_y"] = this->get_position().y();
    telemetry["position_z"] = this->get_position().z();
    telemetry["velocity_x"] = this->rigid_body.getState().body_velocity.x();
    telemetry["velocity_y"] = this->rigid_body.getState().body_velocity.y();
    telemetry["velocity_z"] = this->rigid_body.getState().body_velocity.z();
    telemetry["orientation_w"] = this->rigid_body.getState().quat_attitude.w();
    telemetry["orientation_x"] = this->rigid_body.getState().quat_attitude.x();
    telemetry["orientation_y"] = this->rigid_body.getState().quat_attitude.y();
    telemetry["orientation_z"] = this->rigid_body.getState().quat_attitude.z();
    telemetry["angular_velocity_x"] = this->rigid_body.getState().body_rotation_rate.x();
    telemetry["angular_velocity_y"] = this->rigid_body.getState().body_rotation_rate.y();
    telemetry["angular_velocity_z"] = this->rigid_body.getState().body_rotation_rate.z();
    std::array<BLDC_Motor, 4> motors = this->get_motors();
    std::array<ESC, 4> escs = this->get_escs();
    LipoBattery battery = this->get_battery();
    for (size_t i = 0; i < 4; ++i) {
        std::string prefix_motor = "motor_" + std::to_string(i) + "_";
        std::string prefix_esc = "esc_" + std::to_string(i) + "_";
        telemetry[prefix_motor + "force"] = motors[i].get_current_thrust();
        telemetry[prefix_motor + "torque"] = motors[i].get_current_torque();
        telemetry[prefix_motor + "omega"] = motors[i].get_omega();
        telemetry[prefix_motor + "current"] = motors[i].get_current();
        telemetry[prefix_esc + "voltage"] = escs[i].get_current_voltage();
    }
    telemetry["battery_current"] = battery.get_current();
    telemetry["battery_voltage"] = battery.get_terminal_voltage();
    return telemetry;
}