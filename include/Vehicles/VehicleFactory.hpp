#pragma once

#include <Eigen/Eigen>
#include <json.hpp>
#include <string>
#include <vector>
#include <memory>
#include "Quadcopter.hpp"

class VehicleFactory {
    public:
    static bool validate_config(const std::string& config_path);
    static BETRotor build_prop(const std::string& config_path);
    static LipoBattery build_battery(const std::string& config_path);
    static std::vector<ESC> build_escs(const std::string& config_path);
    static std::vector<BLDC_Motor> build_motors(const std::string& config_path, BETRotor prop);
    static std::unique_ptr<Vehicle> build_vehicle(const std::string& config_path);
};