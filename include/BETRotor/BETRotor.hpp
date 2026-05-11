#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

class BETRotor {
protected:
    std::vector<double> stations; // Local radius (r)
    std::vector<double> chords;   // Blade width (c)
    std::vector<double> twists;   // Physical angle (theta) in RADIANS

    double radius;
    double root_cutout;
    int num_blades;

    double CL_alpha;
    double CD_0;
    double k_CD; // Quadratic drag coefficient for separated flow
    double CD_max;
    double alpha_stall; // Stall angle in RADIANS
    double M_stall;

public:

    BETRotor();

    bool load_propeller_data(const std::string& filepath);
    
    void print_debug_info() const;

    ~BETRotor();

    // ... (Aerodynamic functions will go here later) ...

    std::pair<Eigen::Vector3d, Eigen::Vector3d> calculate_aerodynamics(double omega, const Eigen::Vector3d& body_air_velocity, double& induced_velocity, double thrust, int spin_direction, double air_density) const;

    bool calculate_coefficients(const std::string& filepath);
};