#include "Battery/LipoBattery.hpp"
#include <algorithm> // REQUIRED for std::max
#include <cmath>     // REQUIRED for std::pow

LipoBattery::LipoBattery(uint8_t number_of_cells, double capacity, double internal_resistance, double c_cont, double c_burst) 
    : number_of_cells(number_of_cells), 
      capacity_Ah(capacity), 
      c_rating_continuous(c_cont), 
      c_rating_burst(c_burst), 
      base_internal_resistance(internal_resistance), 
      consumed_Ah(0.0),
      current_draw(0.0),
      resting_voltage(number_of_cells * 4.2), // Start at fully charged voltage
      terminal_voltage(number_of_cells * 4.2)
{
    this->max_continuous_amps = this->capacity_Ah * this->c_rating_continuous;
    this->max_burst_amps = this->capacity_Ah * this->c_rating_burst;
    this->dynamic_resistance = this->base_internal_resistance;
}

void LipoBattery::update(double total_current_draw, double dt) {
    // Track current for the getter function
    this->current_draw = total_current_draw;

    // 1. Drain the capacity (Ah)
    this->consumed_Ah += total_current_draw * (dt / 3600.0);

    // 2. Calculate Resting Voltage (Linear discharge curve)
    // (Ensure you clip this so it doesn't go below 3.0V per cell!)
    double cell_v = 4.2 - (0.7 * (this->consumed_Ah / this->capacity_Ah));
    this->resting_voltage = std::max(3.0, cell_v) * this->number_of_cells;

    // 3. The Two-Stage C-Rating Math (Dynamic Resistance)
    if (total_current_draw <= this->max_continuous_amps) {
        
        // ZONE 1 (Safe): The drone is cruising. Resistance is at its lowest.
        this->dynamic_resistance = this->base_internal_resistance;

    } else if (total_current_draw <= this->max_burst_amps) {
        
        // ZONE 2 (Burst): The drone is doing a punch-out. 
        double burst_ratio = (total_current_draw - this->max_continuous_amps) / 
                            (this->max_burst_amps - this->max_continuous_amps);
        
        // Simulate rapid heating: Resistance climbs up to 2x its normal value at the edge of the burst limit.
        this->dynamic_resistance = this->base_internal_resistance * (1.0 + burst_ratio);

    } else {
        
        // ZONE 3 (Death): The drone is pulling an impossible load.
        double overdraw_ratio = total_current_draw / this->max_burst_amps;
        
        // Start at the maximum burst resistance (2x base), and apply an exponential cliff
        double max_burst_res = this->base_internal_resistance * 2.0;
        this->dynamic_resistance = max_burst_res * std::pow(overdraw_ratio, 4.0);
    }

    // 4. Calculate Final Voltage Sag
    this->terminal_voltage = this->resting_voltage - (total_current_draw * this->dynamic_resistance);

    // Hard clip terminal voltage to 0 to prevent physics engine explosions
    if (this->terminal_voltage < 0.0) {
        this->terminal_voltage = 0.0;
    }
}