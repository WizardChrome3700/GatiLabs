#include <Eigen/Dense>

struct DroneState
{
    Eigen::Vector3d position;
    Eigen::Vector3d body_velocity;
    Eigen::Quaterniond quat_attitude;
    Eigen::Vector3d body_rotation_rate;
};

struct DroneStateDerivative
{
    Eigen::Vector3d velocity;
    Eigen::Vector3d body_acceleration;
    Eigen::Vector4d quat_attitude_derivative;
    Eigen::Vector3d body_rate_derivative;
};

class RigidBody
{
private:
    // 13 states of quadcopter
    DroneState state;
    // drone parameters
    double mass;
    Eigen::Matrix3d MOI_Tensor;
    DroneStateDerivative drone_state_dynamics(const DroneState& state, Eigen::Vector3d Force, Eigen::Vector3d Torque);

public:
    RigidBody(double mass, Eigen::Matrix3d MOI_Tensor);
    ~RigidBody();
    DroneState rk4_step(const DroneState& state, Eigen::Vector3d Force, Eigen::Vector3d Torque, double h);
    void setState(const DroneState& state);
    DroneState getState() const;
};


