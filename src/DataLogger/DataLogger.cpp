#include "DataLogger.hpp"
#include <memory>
#include <iostream>

// --- PHASE 1: PREALLOCATE ---
void DataLogger::preallocate(size_t expected_frames) {
    // Prevent the vector from dynamically resizing mid-flight
    ram_buffer.reserve(expected_frames);
}

// --- PHASE 2: CAPTURE TO RAM ---
void DataLogger::log_frame(const Quadcopter& quadcopter, double timestamp) {
    // 1. Get the flattened dictionary for this exact frame
    auto current_telemetry = quadcopter.get_telemetry(timestamp);

    // 2. Push it into RAM
    ram_buffer.push_back(current_telemetry);
}

// --- PHASE 3: DUMP TO HARD DRIVE ---
void DataLogger::export_to_csv(const std::string& filepath) {
    if (ram_buffer.empty()) {
        std::cerr << "Warning: RAM buffer is empty. Nothing to export.\n";
        return;
    }

    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file for writing.");
    }

    // 1. Write the Header (Read keys from Frame 0)
    auto& first_frame = ram_buffer[0];
    for (auto it = first_frame.begin(); it != first_frame.end(); ++it) {
        file << it->first;
        if (std::next(it) != first_frame.end()) file << ",";
    }
    file << "\n";

    // 2. Write the Data (Loop through all frames in RAM)
    for (const auto& frame : ram_buffer) {
        for (auto it = frame.begin(); it != frame.end(); ++it) {
            file << it->second;
            if (std::next(it) != frame.end()) file << ",";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Successfully exported " << ram_buffer.size() << " frames to " << filepath << "\n";
}