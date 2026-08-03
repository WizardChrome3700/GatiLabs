#include "Vehicles/VehicleFactory.hpp"
#include <json.hpp>
#include <fstream>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;

std::unique_ptr<Vehicle> VehicleFactory::build_vehicle(const std::string& config_path) {
    // 1. Parse JSON
    fs::path config_folder = config_path;
    fs::path file = "frame.json";
    fs::path frame_path = config_folder / file;
    // std::string frame_path = config_path + "\\frame.json";
    std::ifstream config_file(frame_path.string());
    nlohmann::json config_data;
    config_file >> config_data;

    std::string type = config_data["type"];

    // 2. Route the build logic based on the type
    if (type == "quadcopter") {
        
        // ... extract your 4 motors, 4 escs, etc. ...
        file = "motor.json";
        fs::path motor_path = config_folder / file;
        file = "battery.json";
        fs::path battery_path = config_folder / file;
        file = "esc.json";
        fs::path esc_path = config_folder / file;
        file = "propeller.json";
        fs::path propeller_path = config_folder / file;

        if( !fs::exists(battery_path) ) {
            throw std::runtime_error("Battery configuration not found at: " + battery_path.string());
        }
        LipoBattery battery = VehicleFactory::build_battery(battery_path.string());
        if( !fs::exists(propeller_path) ) {
            throw std::runtime_error("Propeller configuration not found at: " + propeller_path.string());
        }
        BETRotor prop = VehicleFactory::build_prop(propeller_path.string());
        std::array<ESC, 4> escs;
        if( !fs::exists(esc_path) ) {
            throw std::runtime_error("ESC configuration not found at: " + esc_path.string());
        }
        std::vector<ESC> esc_temp = VehicleFactory::build_escs(esc_path.string());
        for(int i = 0; i < 4; i++) {
            escs[i] = esc_temp[i];
        }
        std::array<BLDC_Motor, 4> motors;
        if( !fs::exists(motor_path) ) {
            throw std::runtime_error("Motor configuration not found at: " + motor_path.string());
        }
        std::vector<BLDC_Motor> motor_temp = VehicleFactory::build_motors(motor_path.string(), prop);
        for(int i = 0; i < 4; i++) {
            motors[i] = motor_temp[i];
        }
        double mass = config_data["dry_mass_kg"];
        Eigen::Matrix3d MOI = Eigen::Vector3d(config_data["moi_diagonal_kgm2"]["xx"], config_data["moi_diagonal_kgm2"]["yy"], config_data["moi_diagonal_kgm2"]["zz"]).asDiagonal();
        
        
        // Create the Quadcopter, but return it disguised as a generic Vehicle!
        return std::make_unique<Quadcopter>(mass, MOI, battery, escs, motors);
        
    } else {
        throw std::runtime_error("Unknown vehicle type in JSON!");
    }
}

LipoBattery VehicleFactory::build_battery(const std::string& config_path) {
    std::ifstream battery_file(config_path);
    nlohmann::json battery_data;
    battery_file >> battery_data;

    uint8_t number_of_cells = battery_data["cells"];
    double capacity_Ah = battery_data["capacity_Ah"];
    double internal_resistance = battery_data["internal_resistance_ohms"];
    double c_cont = battery_data["c_rating_continuous"];
    double c_burst = battery_data["c_rating_burst"];

    return LipoBattery(number_of_cells, capacity_Ah, internal_resistance, c_cont, c_burst);
}

std::vector<BLDC_Motor> VehicleFactory::build_motors(const std::string& config_path, BETRotor prop) {
    std::vector<BLDC_Motor> motors;
    std::ifstream config_file(config_path);
    nlohmann::json config_data;
    config_file >> config_data;
    motors.resize(config_data.size());
    
    // This loops through every item in the JSON object
    for (auto& item : config_data.items()) {
        std::string key = item.key();          // "1", "2", etc.
        nlohmann::json motor_specs = item.value();       // The actual block of data
        
        // Now you can extract the specific data:
        double kv = motor_specs["kv_rating"];
        Eigen::Vector3d pos;
        pos.x() = motor_specs["pos"]["x"];
        pos.y() = motor_specs["pos"]["y"];
        pos.z() = motor_specs["pos"]["z"];
        const int direction = motor_specs["spin_direction"];
        double resistance = motor_specs["phase_resistance_ohms"];
        double no_load_current = motor_specs["no_load_current_amps"];
        double max_current_draw = motor_specs["max_current_amps"];
        magnet_class magnet_grade;
        std::string magnet_str = motor_specs["magnet_grade"];
        if (magnet_str == "N42") {
            magnet_grade = N42;
        } else if (magnet_str == "N48") {
            magnet_grade = N48;
        } else if (magnet_str == "N52") {
            magnet_grade = N52;
        } else if (magnet_str == "N55") {
            magnet_grade = N55;
        } else {
            throw std::runtime_error("Unknown magnet grade in JSON!");
        }
        double height = motor_specs["stator_height_m"];
        double diameter = motor_specs["stator_diameter_m"];
        double mass = motor_specs["mass_kg"];
        
        // ... build your motor here ...
        motors[std::stoi(key) - 1] = BLDC_Motor(prop, pos, direction, kv, resistance, no_load_current, max_current_draw, magnet_grade, height, diameter, mass);
    }
    return motors;
}

std::vector<ESC> VehicleFactory::build_escs(const std::string& config_path) {
    std::vector<ESC> escs;
    std::ifstream config_file(config_path);
    nlohmann::json config_data;
    config_file >> config_data;
    escs.resize(config_data.size());

    for (auto& item : config_data.items()) {
        std::string key = item.key();          // "1", "2", etc.
        nlohmann::json esc_specs = item.value();       // The actual block of data
        
        double internal_resistance = esc_specs["internal_resistance_ohms"];
        double max_current = esc_specs["max_current_amps"];
        double max_voltage = esc_specs["max_voltage_volts"];
        double stand_by_current = esc_specs["standby_current_amps"];
        double mass = esc_specs["mass_kg"];
        Eigen::Vector3d pos;
        pos.x() = esc_specs["position"]["x"];
        pos.y() = esc_specs["position"]["y"];
        pos.z() = esc_specs["position"]["z"];

        escs[std::stoi(key) - 1] = ESC(mass, max_current, max_voltage, internal_resistance, pos);
    }
    return escs;
}

BETRotor VehicleFactory::build_prop(const std::string& config_path) {
    std::ifstream config_file(config_path);
    nlohmann::json config_data;
    config_file >> config_data;
    BETRotor prop;
    prop.load_propeller_data(config_data["geometry_filepath"]);
    prop.calculate_coefficients(config_data["aerodynamic_polar_filepath"]);
    return prop;
}

