#include "RigidBody.hpp"

RigidBody::RigidBody(double mass, Eigen::Matrix3d MOI_Tensor) {
    this->mass = mass;
    this->MOI_Tensor = MOI_Tensor;
    this->state.position = Eigen::Vector3d::Zero();
    this->state.body_velocity = Eigen::Vector3d::Zero();
    this->state.quat_attitude = Eigen::Quaterniond::Identity();
    this->state.body_rotation_rate = Eigen::Vector3d::Zero();
}

RigidBody::~RigidBody()
{
}

DroneStateDerivative operator* (double constant, DroneStateDerivative state_derivative) {
    DroneStateDerivative state_derivative_scaled;
    state_derivative_scaled.velocity = constant * state_derivative.velocity;
    state_derivative_scaled.body_acceleration = constant * state_derivative.body_acceleration;
    state_derivative_scaled.quat_attitude_derivative = constant * state_derivative.quat_attitude_derivative;
    state_derivative_scaled.body_rate_derivative = constant * state_derivative.body_rate_derivative;
    return state_derivative_scaled;
}

DroneStateDerivative operator+ (DroneStateDerivative state_derivative1, DroneStateDerivative state_derivative2) {
    DroneStateDerivative state_derivative_sum;
    state_derivative_sum.velocity = state_derivative1.velocity + state_derivative2.velocity;
    state_derivative_sum.body_acceleration = state_derivative1.body_acceleration + state_derivative2.body_acceleration;
    state_derivative_sum.quat_attitude_derivative = state_derivative1.quat_attitude_derivative + state_derivative2.quat_attitude_derivative;
    state_derivative_sum.body_rate_derivative = state_derivative1.body_rate_derivative + state_derivative2.body_rate_derivative;
    return state_derivative_sum;
}

DroneState operator+ (DroneState state, DroneStateDerivative state_derivative2) {
    DroneState state_new;
    state_new.position = state.position + state_derivative2.velocity;
    state_new.body_velocity = state.body_velocity + state_derivative2.body_acceleration;
    // Eigen::Vector4d quatAtt = (state.quat_attitude).coeffs() + state_derivative2.quat_attitude_derivative;
    // quatAtt = quatAtt.normalize();
    // state_new.quat_attitude = Eigen::Quaterniond(quatAtt[3], quatAtt[0], quatAtt[1], quatAtt[2]);
    state_new.quat_attitude.coeffs() = (state.quat_attitude.coeffs() + state_derivative2.quat_attitude_derivative).normalized();
    state_new.body_rotation_rate = state.body_rotation_rate + state_derivative2.body_rate_derivative;
    return state_new;
}

DroneStateDerivative RigidBody::drone_state_dynamics(const DroneState& state, Eigen::Vector3d Force, Eigen::Vector3d Torque) {
    DroneStateDerivative state_derivate;

    // 1. Kinematics (World Velocity)
    state_derivate.velocity = state.quat_attitude * state.body_velocity;

    // 2. Gravity
    Eigen::Vector3d gravity_world(0.0, 0.0, -9.81);
    Eigen::Vector3d gravity_body = state.quat_attitude.inverse() * gravity_world;

    // 3. Raw Acceleration (Body Frame)
    Eigen::Vector3d raw_body_accel = (Force/(this->mass)) - state.body_rotation_rate.cross(state.body_velocity) + gravity_body;

    // --- THE HARD GROUND CONSTRAINT (NORMAL FORCE) ---
    // Check if we are physically on or below the ground
    if (state.position.z() <= 0.0) {
        
        // Convert acceleration to World Frame to inspect the Z-axis
        Eigen::Vector3d world_accel = state.quat_attitude * raw_body_accel;

        // If the net acceleration is pushing DOWN into the floor
        if (world_accel.z() < 0.0) {
            
            // The ground pushes back! Cancel the downward Z acceleration
            world_accel.z() = 0.0; 
            
            // Apply static friction so the drone doesn't slide sideways while sitting on the ground
            world_accel.x() = 0.0; 
            world_accel.y() = 0.0;

            // Convert the clamped acceleration back to the Body Frame
            raw_body_accel = state.quat_attitude.inverse() * world_accel;
        }

        // Prevent the RK4 integrator from integrating downward velocity
        if (state_derivate.velocity.z() < 0.0) {
            state_derivate.velocity.z() = 0.0;
        }
    }
    // -------------------------------------------------

    state_derivate.body_acceleration = raw_body_accel;

    // 4. Angular Dynamics
    Eigen::Quaterniond body_rotation_rate_quat(0, state.body_rotation_rate[0], state.body_rotation_rate[1], state.body_rotation_rate[2]);
    state_derivate.quat_attitude_derivative = (1.0/2)*(state.quat_attitude * body_rotation_rate_quat).coeffs();
    state_derivate.body_rate_derivative = ((this->MOI_Tensor).inverse()) * (Torque - state.body_rotation_rate.cross((this->MOI_Tensor) * state.body_rotation_rate));

    return state_derivate;
}

DroneState RigidBody::rk4_step(const DroneState& state, Eigen::Vector3d Force, Eigen::Vector3d Torque, double h) {
    DroneStateDerivative k1 = RigidBody::drone_state_dynamics(state, Force, Torque);
    DroneStateDerivative k2 = RigidBody::drone_state_dynamics(state + (h/2)*k1, Force, Torque);
    DroneStateDerivative k3 = RigidBody::drone_state_dynamics(state + (h/2)*k2, Force, Torque);
    DroneStateDerivative k4 = RigidBody::drone_state_dynamics(state + (h/1)*k3, Force, Torque);
    DroneState state_update = state + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
    return state_update;
}

void RigidBody::setState(const DroneState& state) {
    this->state = state;
}

DroneState RigidBody::getState() const {
    return this->state;
}