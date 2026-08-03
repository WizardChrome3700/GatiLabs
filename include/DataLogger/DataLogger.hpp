#pragma once

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include "Vehicles/Quadcopter.hpp"

class DataLogger {
private:
    // --- THE RAM BUFFER ---
    // This holds every single frame of telemetry in memory
    std::vector<std::map<std::string, double>> ram_buffer;

public:
    DataLogger() = default;
    ~DataLogger() = default;

    // 1. Memory Optimization
    void preallocate(size_t expected_frames);

    // 2. The Capture Phase (Runs at 400Hz - Blazing Fast)
    void log_frame(const Quadcopter& quadcopter, double timestamp);

    // 3. The Dump Phase (Runs once at 0Hz - Slow File I/O)
    void export_to_csv(const std::string& filepath);
};