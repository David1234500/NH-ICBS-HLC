#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <nlopt.hpp>

using Eigen::Vector3d;

// Reeds-Shepp motion types
enum class MotionType { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_BACK };

struct ReedsSheppSegment {
    MotionType motionType;
    double length;
};

double reedsSheppCostFunction(const std::vector<double> &x, std::vector<double> &grad, void *data) {
    const Vector3d *points = reinterpret_cast<Vector3d *>(data);
    Vector3d start = points[0];
    Vector3d goal = points[1];

    double radius = 1.0;

    // Compute the Reeds-Shepp path
    double s = x[0];
    double t = x[1];
    double u = x[2];

    Vector3d p1 = start + Vector3d(cos(start(2)), sin(start(2)), 0) * s;
    Vector3d p2 = goal - Vector3d(cos(goal(2)), sin(goal(2)), 0) * u;
    double angle = atan2(p2(1) - p1(1), p2(0) - p1(0));
    double requiredAngle = angle - p1(2) - t / radius;

    while (requiredAngle > M_PI) requiredAngle -= 2 * M_PI;
    while (requiredAngle < -M_PI) requiredAngle += 2 * M_PI;

    return s * s + t * t + u * u + 1000 * requiredAngle * requiredAngle;
}

std::vector<ReedsSheppSegment> computeReedsSheppPath(const Vector3d &start, const Vector3d &goal, double radius) {
    // Set up the optimization problem
    nlopt::opt opt(nlopt::LN_COBYLA, 3);
    std::vector<double> lb(3, 0);
    opt.set_lower_bounds(lb);
    opt.set_min_objective(reedsSheppCostFunction, const_cast<Vector3d *>(reinterpret_cast<const Vector3d *>(goal.data())));

    Vector3d data[2] = {start, goal};
    opt.set_min_objective(reedsSheppCostFunction, data);

    std::vector<double> x(3, radius);
    double minf;
    nlopt::result result = opt.optimize(x, minf);

    // Extract Reeds-Shepp path from optimization result
    double s = x[0];
    double t = x[1];
    double u = x[2];

    std::vector<ReedsSheppSegment> path;
    path.push_back({MotionType::RS_LEFT, s});
    path.push_back({MotionType::RS_STRAIGHT, radius * t});
    path.push_back({MotionType::RS_RIGHT, u});

    return path;
}

int main() {
    Vector3d start(0, 0, 0);
    Vector3d goal(5, 5, M_PI / 2);
    double turningRadius = 1.0;

    std::vector<ReedsSheppSegment> path = computeReedsSheppPath(start, goal, turningRadius);

    std::cout << "Reeds-Shepp motion primitives:" << std::endl;
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "Segment " << (i + 1) << ": ";
        std::cout << "Length = " << path[i].length << ", ";
        
        std::string motionType;
        switch (path[i].motionType) {
            case MotionType::RS_LEFT:
                motionType = "Left";
                break;
            case MotionType::RS_RIGHT:
                motionType = "Right";
                break;
            case MotionType::RS_STRAIGHT:
                motionType = "Straight";
                break;
            case MotionType::RS_BACK:
                motionType = "Back";
                break;
        }
        std::cout << "Type = " << motionType << std::endl;
    }

    return 0;
}
