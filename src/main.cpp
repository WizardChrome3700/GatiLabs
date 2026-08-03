#include <iostream>
#include <array>
#include <Eigen/Dense>
#include <json.hpp>
#include <filesystem>

namespace fs = std::filesystem;

#include "Vehicles/Quadcopter.hpp"
#include "Battery/LipoBattery.hpp"
#include "ESC/ESC.hpp"
#include "Motor/BLDC_Motor.hpp"
#include "BETRotor/BETRotor.hpp"
#include "DataLogger/DataLogger.hpp"
#include "Vehicles/VehicleFactory.hpp"

int main() {

    // 1. Get the generic vehicle pointer
    fs::path config_dir = "configs";
    fs::path vehicle_config = "default";
    fs::path vehicle_config_path = config_dir / vehicle_config;
    if( !fs::exists(vehicle_config_path) ) {
        throw std::runtime_error("Vehicle configuration not found at: " + vehicle_config_path.string());
    }
    std::unique_ptr<Vehicle> my_drone = VehicleFactory::build_vehicle(vehicle_config_path.string());

    // 2. Downcast it to a Quadcopter reference
    Quadcopter& quad = dynamic_cast<Quadcopter&>(*my_drone);
    // ==========================================
    // 3. TELEMETRY SETUP
    // ==========================================
    
    DataLogger logger;
    
    double dt = 0.0025;              // 400Hz physics loop
    double flight_time = 15.0;       // Simulate 15 seconds of flight
    size_t expected_frames = static_cast<size_t>(flight_time / dt) + 1;
    
    logger.preallocate(expected_frames);
    // quad.set_motor_commands({1.0, 1.0, 1.0, 1.0}); // 50% throttle to all motors for a hover test

    // Define the ramp outside the loop
    double target_throttle = 1.0; // The final throttle you want to reach
    double target_pitch_input = 0.005;
    double ramp_duration = flight_time * 0.01; 
    
    for (double current_time = 0.0; current_time <= flight_time; current_time += dt) {

        // 1. Calculate the soft-start throttle dynamically
        double current_throttle;
        double current_pitch_input;
        if (current_time < ramp_duration) {
            // Linearly interpolate from 0 to target_throttle
            current_throttle = target_throttle * (current_time / ramp_duration);
            current_pitch_input = target_pitch_input * (current_time / ramp_duration);
        } else {
            // Cap it at the target once the ramp is over
            current_throttle = target_throttle;
            current_pitch_input = target_pitch_input;
        }

        // Apply it safely to all motors
        quad.set_motor_commands({current_throttle - current_pitch_input, current_throttle - current_pitch_input, current_throttle + current_pitch_input, current_throttle + current_pitch_input});

        // A. Accumulate all aerodynamic and powertrain forces/torques
        quad.calculate_forces_torques(dt);

        // B. Step the rigid body kinematics forward in time
        quad.update(dt);

        // C. Capture the flattened telemetry dictionary to RAM
        logger.log_frame(quad, current_time);
    }

    // ==========================================
    // 5. EXPORT
    // ==========================================
    
    std::cout << "Simulation complete. Dumping RAM buffer to hard drive...\n";
    logger.export_to_csv("flight_log.csv");
    std::cout << "Done.\n";

    return 0;
}