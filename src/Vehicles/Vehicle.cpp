#include "Vehicle.hpp"
#include <Eigen/Dense>

Vehicle::Vehicle(double mass, Eigen::Matrix3d MOI) : rigid_body(mass, MOI) {
    this->current_force = Eigen::Vector3d::Zero();
    this->current_torque = Eigen::Vector3d::Zero();
}

void Vehicle::clear_forces() {
    this->current_force = Eigen::Vector3d::Zero();
    this->current_torque = Eigen::Vector3d::Zero();
}

void Vehicle::add_force_at_point(Eigen::Vector3d force, Eigen::Vector3d local_position) {
    this->current_force += force;
    this->current_torque += local_position.cross(force);
}

void Vehicle::update(double dt) {
    this->clear_forces();
    this->calculate_forces_torques(dt);
    this->rigid_body.setState(this->rigid_body.rk4_step(this->rigid_body.getState(), this->current_force, this->current_torque, dt));
}
