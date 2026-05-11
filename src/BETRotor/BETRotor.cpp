#include "BETRotor.hpp"
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

bool BETRotor::load_propeller_data(const std::string& filepath) {
    std::ifstream prop_file(filepath);
    if (!prop_file.is_open()) {
        std::cerr << "Error: Could not open file at " << filepath << "\n";
        return false;
    }
    std::string line;
    int col_station = -1;
    int col_chord = -1;
    int col_twist = -1;
    // 2. The Forward Pass
    while (std::getline(prop_file, line)) {
        if (col_station == -1 && 
            line.find("STATION") != std::string::npos &&
            line.find("CHORD") != std::string::npos &&
            line.find("TWIST") != std::string::npos) {
            std::istringstream ss(line);
            std::string word;
            int current_index = 0;
            
            while (ss >> word) {
                if (word == "STATION") col_station = current_index;
                else if (word == "CHORD") col_chord = current_index;
                else if (word == "TWIST") col_twist = current_index;
                current_index++;
            }
            continue;
        }

        if (col_station != -1) {
            std::istringstream ss(line);
            double value;
            std::vector<double> row_data;
            
            while (ss >> value) {
                row_data.push_back(value);
            }

            if (row_data.size() > col_twist) {
                // SUCCESS! We have a row of data. 
                // Extract the exact columns we need:
                double r = row_data[col_station];
                double c = row_data[col_chord];
                double theta = row_data[col_twist];

                // TODO: Save these into your BETRotor class vectors (stations, chords, twists)
                // (Remember to check if 'r' is greater than your root_cutout!)
                this->stations.push_back(r*0.0254);
                this->chords.push_back(c*0.0254);
                this->twists.push_back(theta* (3.14159265359 / 180.0));
            }
        }
        
        if (line.find("RADIUS:") != std::string::npos) {
            std::istringstream ss(line);
            std::string trash;
            ss >> trash >> this->radius; 
            
            this->radius *= 0.0254; 
            continue;
        }
        
        if (line.find("HUBRAD:") != std::string::npos) {
            std::istringstream ss(line);
            std::string trash;
            ss >> trash >> this->root_cutout;
            this->root_cutout *= 0.0254;
            continue;
        }

        if (line.find("BLADES:") != std::string::npos) {
            std::istringstream ss(line);
            std::string trash;
            ss >> trash >> this->num_blades;
            return true;
            break; 
        }
    }
    return false;
}

void BETRotor::print_debug_info() const {
    std::cout << "--- PROPELLER LOADED ---\n";
    
    // Lock the stream into fixed decimal mode with exactly 4 places
    std::cout << std::fixed << std::setprecision(5);
    
    // Now you don't need to repeat setprecision on every line!
    std::cout << "Radius: " << this->radius / 0.0254 << "\n";
    std::cout << "Root Cutout: " << this->root_cutout / 0.0254 << "\n";
    std::cout << "Blades: " << this->num_blades << "\n";
    
    // Temporarily switch back to default for integers so it doesn't print "51.0000 rows"
    std::cout << std::defaultfloat << "Parsed " << this->stations.size() << " data rows.\n\n";

    int print_limit = std::min(5, (int)this->stations.size());
    std::cout << "First " << print_limit << " valid rows (Metric):\n";
    std::cout << "Station      Chord      Twist\n";
    
    // Switch back to fixed mode for the table
    std::cout << std::fixed << std::setprecision(4);
    for(int i = 0; i < print_limit; i++) {
        std::cout << this->stations[i] / 0.0254 << "      " 
                  << this->chords[i] / 0.0254 << "      " 
                  << this->twists[i] / (3.14159265359 / 180.0) << "\n";
    }
}

bool BETRotor::calculate_coefficients(const std::string& filepath) {
    std::vector<double> alpha_data;
    std::vector<double> cl_data;
    std::vector<double> cd_data;
    std::ifstream prop_file(filepath);
    if (!prop_file.is_open()) {
        std::cerr << "Error: Could not open file at " << filepath << "\n";
        return false;
    }
    std::string line;
    uint8_t headerCount = 12;
    while(headerCount-- > 0) {
        if (!std::getline(prop_file, line)) {
            std::cerr << "Error: File ended before expected header lines were read.\n";
        }
    }
    while(std::getline(prop_file, line)) {
        std::istringstream ss(line);
        double alpha, cl, cd;
        if (ss >> alpha >> cl >> cd) {
            alpha_data.push_back(alpha * (3.14159265359 / 180.0)); // Convert to radians
            cl_data.push_back(cl);
            cd_data.push_back(cd);
        } else {
            std::cerr << "Warning: Skipping malformed line: " << line << "\n";
        }
    }
    auto max_cl = std::max_element(cl_data.begin(), cl_data.end());
    auto min_cl = std::min_element(cl_data.begin(), cl_data.end());
    size_t max_cl_index, min_cl_index;
    if(max_cl != cl_data.end()) {
        max_cl_index = std::distance(cl_data.begin(), max_cl);
        this->alpha_stall = alpha_data[max_cl_index];
    }
    min_cl_index = std::distance(cl_data.begin(), min_cl);
    
    // 1. Count how many points are between -5 and +5 degrees
    int N = 0;
    for (double alpha : alpha_data) {
        if (alpha >= -5.0 * (M_PI/180.0) && alpha <= 5.0 * (M_PI/180.0)) N++;
    }

    Eigen::MatrixXd A_linear;
    Eigen::VectorXd b_linear;
    // 2. Resize your dynamic matrices to exactly N
    A_linear.resize(N, 2);
    b_linear.resize(N);

    // 3. Populate them
    int row = 0;
    for (size_t i = 0; i < alpha_data.size(); i++) {
        if (alpha_data[i] >= -5.0 * (M_PI/180.0) && alpha_data[i] <= 5.0 * (M_PI/180.0)) {
            A_linear(row, 0) = alpha_data[i];
            A_linear(row, 1) = 1.0;
            b_linear(row) = cl_data[i];
            row++;
        }
    }
    Eigen::Vector2d x_Cl = A_linear.colPivHouseholderQr().solve(b_linear);
    this->CL_alpha = x_Cl(0);
    row = 0;
    for (size_t i = 0; i < alpha_data.size(); i++) {
        if (alpha_data[i] >= -5.0 * (M_PI/180.0) && alpha_data[i] <= 5.0 * (M_PI/180.0)) {
            A_linear(row, 0) = alpha_data[i] * alpha_data[i];
            A_linear(row, 1) = 1;
            b_linear(row) = cd_data[i];
            row++;
        }
    }
    Eigen::Vector2d x_Cd = A_linear.colPivHouseholderQr().solve(b_linear);
    this->CD_0 = x_Cd(1);
    this->k_CD = x_Cd(0);
    return true;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> BETRotor::calculate_aerodynamics(double omega, const Eigen::Vector3d& body_air_velocity, double& induced_velocity, double thrust, int spin_direction, double air_density) const {
    double total_thrust = 0.0;
    double total_torque = 0.0;
    
    double u = body_air_velocity.x();
    double v = body_air_velocity.y();
    double w = body_air_velocity.z();
    double rho = 1.225; // Air density at sea level

    double v_i; // Induced velocity at the rotor disk
    if (thrust > 0.0) {
        double area = M_PI * this->radius * this->radius;
        double rho = 1.225;
        
        // 1. THE FIX: Check only the drone's freestream speed!
        double freestream_sq = u*u + v*v + w*w; 
        
        if (freestream_sq < 0.1) {
            // Hover condition (drone is practically stationary)
            v_i = std::sqrt(thrust / (2.0 * rho * area));
        } else {
            // Forward flight condition (Glauert equation)
            double denominator_sq = u*u + v*v + (w + induced_velocity)*(w + induced_velocity);
            v_i = thrust / (2.0 * rho * area * std::sqrt(denominator_sq));
        }
    }
    else {
        v_i = 0.0; 
    }
    
    // 2. THE FIX: Apply numerical relaxation to the air inflow too!
    double inflow_relaxation = 0.1;
    induced_velocity = (inflow_relaxation * v_i) + ((1.0 - inflow_relaxation) * induced_velocity);

    // 1. Azimuthal Slicing (e.g., 12 slices = 30 degrees each)
    int num_psi_steps = 12;
    double d_psi = 2.0 * M_PI / num_psi_steps;

    for (int step = 0; step < num_psi_steps; step++) {
        double psi = step * d_psi;
        
        // The unified advancing/retreating calculation
        double v_advance = spin_direction * (u * std::sin(psi) - v * std::cos(psi));

        // 2. Radial Slicing
        for (size_t i = 1; i < this->stations.size(); i++) {
            double r = this->stations[i];
            if (r < this->root_cutout) continue;

            double dr = this->stations[i] - this->stations[i-1];
            double c = this->chords[i];
            double theta = this->twists[i];

            // --- The Core Math ---
            double V_T = (omega * r) - v_advance;
            double V_P = w + v_i; 
            
            // Prevent division by zero if motor is completely stopped
            if (V_T <= 0.01 && V_T >= -0.01) continue; 

            double phi = std::atan2(V_P, V_T);
            double alpha = theta - phi;

            // 1. The Attached (Linear) Airfoil Model
            double Cl_attached = 6.2 * alpha;
            double Cd_attached = 0.008 + 0.1 * (alpha * alpha);

            // 2. The Separated (Flat Plate) Airfoil Model
            // Post-stall, the blade acts like a flat wall. We use standard flat-plate theory.
            double Cd_max = 2.0; 
            double Cl_separated = 0.5 * Cd_max * std::sin(2.0 * alpha);
            double Cd_separated = Cd_max * std::sin(alpha) * std::sin(alpha);

            // 3. The Corrected Blending Function
            double alpha_0 = 0.26; // Stall angle in radians (~15 degrees)
            double M = 50.0;       // Transition sharpness

            double exp_1 = std::exp(-M * (alpha - alpha_0));
            double exp_2 = std::exp(M * (alpha - alpha_0));

            // Calculate sigma (protecting against potential floating point overflow)
            double sigma;
            if (std::isinf(exp_1) || std::isinf(exp_2)) {
                sigma = 1.0; // If exponents blow up, we are deeply stalled
            } else {
                double numerator = 1.0 + exp_1 + exp_2;
                double denominator = (1.0 + exp_1) * (1.0 + exp_2);
                sigma = numerator / denominator;
            }

            // 4. The Final Blended Coefficients
            double C_l = (1.0 - sigma) * Cl_attached + (sigma * Cl_separated);
            double C_d = (1.0 - sigma) * Cd_attached + (sigma * Cd_separated);

            double V_total_sq = (V_T * V_T) + (V_P * V_P);
            double dynamic_pressure = 0.5 * rho * V_total_sq * c * dr;

            double dL = dynamic_pressure * C_l;
            double dD = dynamic_pressure * C_d;

            // Resolve to shaft axes
            double dThrust = dL * std::cos(phi) - dD * std::sin(phi);
            double dTorque = (dL * std::sin(phi) + dD * std::cos(phi)) * r;

            total_thrust += dThrust;
            total_torque += dTorque;
        }
    }

    // 3. Average the circle, then multiply by the number of blades
    double final_thrust = (total_thrust / num_psi_steps) * this->num_blades;
    double final_torque = (total_torque / num_psi_steps) * this->num_blades;

    // 4. Create the final 3D Vectors
    // Thrust points UP (+Z)
    Eigen::Vector3d thrust_vec(0.0, 0.0, final_thrust);
    
    // Torque twists around Z. A CCW motor (+1) creates a CW (-1) drag torque!
    Eigen::Vector3d torque_vec(0.0, 0.0, -spin_direction * final_torque);

    return {thrust_vec, torque_vec};
}

BETRotor::BETRotor() {
    this->radius = 0.0;
    this->root_cutout = 0.0;
    this->num_blades = 0;
    this->stations.clear();
    this->chords.clear();
    this->twists.clear();
}

BETRotor::~BETRotor() {

}