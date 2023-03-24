#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

// Vehicle parameters
const double lf = 1.2;  // Distance from CG (center of gravity) to the front axle (m)
const double lr = 1.6;  // Distance from CG to the rear axle (m)
const double m = 1500.0; // Vehicle mass (kg)
const double Iz = 2500.0; // Vehicle yaw moment of inertia (kg*m^2)

// Tire cornering stiffness (N/rad)
const double Cf = 80000.0;
const double Cr = 80000.0;

// Simulation time step (s)
const double dt = 0.01;

// Vehicle state vector [x, y, yaw, vx, vy, yaw_rate]
VectorXd vehicleState(6);

// Update the vehicle state based on the two-track model
void updateVehicleState(double throttle, double steeringAngle) {
    // Extract the current state
    double x = vehicleState(0);
    double y = vehicleState(1);
    double yaw = vehicleState(2);
    double vx = vehicleState(3);
    double vy = vehicleState(4);
    double yaw_rate = vehicleState(5);

    // Calculate the longitudinal and lateral velocities at the front and rear axles
    double vf_lat = vy + lf * yaw_rate;
    double vr_lat = vy - lr * yaw_rate;

    // Calculate the slip angles for front and rear tires
    double alpha_f = std::atan2(vf_lat, vx) - steeringAngle;
    double alpha_r = std::atan2(vr_lat, vx);

    // Calculate the tire lateral forces based on slip angles and cornering stiffness
    double Fyf = -Cf * alpha_f;
    double Fyr = -Cr * alpha_r;

    // Update vehicle state
    double ax = 1/m * (Fyf * std::sin(steeringAngle) + Fyr);
    double ay = 1/m * (Fyf * std::cos(steeringAngle) - Fyr) - vx * yaw_rate;
    double yaw_rate_dot = 1/Iz * (lf * Fyf * std::cos(steeringAngle) - lr * Fyr);

    vehicleState(0) = x + dt * (vx * std::cos(yaw) - vy * std::sin(yaw));
    vehicleState(1) = y + dt * (vx * std::sin(yaw) + vy * std::cos(yaw));
    vehicleState(2) = yaw + dt * yaw_rate;
    vehicleState(3) = vx + dt * ax;
    vehicleState(4) = vy + dt * ay;
    vehicleState(5) = yaw_rate + dt * yaw_rate_dot;
}

int main() {
    // Initialize the vehicle state
    vehicleState << 0, 0, 0, 20, 0, 0;

    // Simulate the vehicle dynamics for 10 seconds with constant throttle and steering angle
    double throttle = 0.2;
    double steeringAngle = 0.1;

    for (int i = 0; i < 1000; ++i) {
        updateVehicleState(throttle, steeringAngle);
        std::cout << "Vehicle state at t = " << (i + 1) * dt << " s: " << vehicleState.transpose() << std::endl;
    }

    return 0;
}
