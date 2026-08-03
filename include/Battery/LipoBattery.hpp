#pragma once

#include <Eigen/Dense>
#include <cstdint>

class LipoBattery {
private:
    double mass;
    // --- Specifications ---
    uint8_t number_of_cells;
    double capacity_Ah;
    double c_rating_continuous; 
    double c_rating_burst;       // The secondary burst rating
    double max_continuous_amps;
    double max_burst_amps;       // The absolute chemical limit

    // --- State Variables ---
    double base_internal_resistance;
    double dynamic_resistance;
    double terminal_voltage;
    
    // FIXED: Added missing state variables required by the cpp file
    double consumed_Ah;
    double resting_voltage;
    double current_draw;

public:
    // FIXED: Added c_burst to the constructor arguments

    LipoBattery(uint8_t number_of_cells, double capacity, double internal_resistance, double c_cont, double c_burst);
    
    void update(double total_current_draw, double dt);
    
    double get_voltage() const { return this->number_of_cells * 3.7; }

    double get_capacity() const { return this->capacity_Ah; }

    double get_current() const { return this->current_draw; }

    double get_mass() const { return this->mass; }

    double get_terminal_voltage() const { return this->terminal_voltage; }
};