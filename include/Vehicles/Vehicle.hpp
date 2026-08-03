#pragma once
#include "RigidBody.hpp"
#include <map>
#include <string>
#include <Eigen/Dense>

class Vehicle
{
private:
    /* data */
protected:
    RigidBody rigid_body;
    Eigen::Vector3d current_force;
    Eigen::Vector3d current_torque;
    void clear_forces();
    void add_force_at_point(Eigen::Vector3d force, Eigen::Vector3d local_position);
public:
    Vehicle(double mass, Eigen::Matrix3d MOI);
    virtual void calculate_forces_torques(double dt) = 0;
    virtual ~Vehicle() = default;
    void update(double dt);
    Eigen::Vector3d get_position() const { return rigid_body.getState().position; }
    virtual std::map<std::string, double> get_telemetry(double timestamp) const = 0;
};