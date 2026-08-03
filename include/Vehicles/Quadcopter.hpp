#pragma once

#include "Vehicles/Vehicle.hpp"
#include "Battery/LipoBattery.hpp"
#include "ESC/ESC.hpp"
#include "Motor/BLDC_Motor.hpp"
#include <array>

class Quadcopter : public Vehicle {
private:
    // The Powertrain
    LipoBattery battery;
    std::array<ESC, 4> escs;
    std::array<BLDC_Motor, 4> motors;

    // Flight Controller Input
    std::array<double, 4> motor_commands; // Throttle inputs (0.0 to 1.0)
    static double calculate_total_mass(double frame_mass, const LipoBattery& b, 
                                   const std::array<ESC, 4>& e, 
                                   const std::array<BLDC_Motor, 4>& m) {
        double total = frame_mass + b.get_mass();
        for (const ESC& esc : e) total += esc.get_mass();
        for (const BLDC_Motor& motor : m) total += motor.get_mass();
        return total;
    }

public:
    // Constructor
    Quadcopter(double mass, Eigen::Matrix3d MOI, 
               const LipoBattery& batt, 
               const std::array<ESC, 4>& esc_array, 
               const std::array<BLDC_Motor, 4>& motor_array);

    // Override the pure virtual function from Vehicle
    void calculate_forces_torques(double dt) override;
    std::map<std::string, double> get_telemetry(double timestamp) const override;

    // Setter for the flight controller to pass commands
    void set_motor_commands(const std::array<double, 4>& commands);

    // Getters for Data Logging
    std::array<BLDC_Motor, 4> get_motors() const;
    std::array<ESC, 4> get_escs() const;
    LipoBattery get_battery() const;
};