#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

using Eigen::Vector2d;

struct Vehicle {
    Vector2d center;
    double length;
    double width;
    double orientation;
};

std::vector<Vector2d> getVehicleCorners(const Vehicle &vehicle) {
    double halfLength = vehicle.length / 2;
    double halfWidth = vehicle.width / 2;

    Vector2d frontRight(halfLength, halfWidth);
    Vector2d frontLeft(halfLength, -halfWidth);
    Vector2d rearRight(-halfLength, halfWidth);
    Vector2d rearLeft(-halfLength, -halfWidth);

    double cosTheta = std::cos(vehicle.orientation);
    double sinTheta = std::sin(vehicle.orientation);

    Eigen::Rotation2D<double> rot(vehicle.orientation);

    frontRight = rot * frontRight + vehicle.center;
    frontLeft = rot * frontLeft + vehicle.center;
    rearRight = rot * rearRight + vehicle.center;
    rearLeft = rot * rearLeft + vehicle.center;

    return {frontRight, frontLeft, rearRight, rearLeft};
}

std::vector<Vector2d> getAxes(const std::vector<Vector2d> &corners) {
    std::vector<Vector2d> axes;
    for (size_t i = 0; i < corners.size(); ++i) {
        Vector2d edge = corners[i] - corners[(i + 1) % corners.size()];
        Vector2d normal(-edge.y(), edge.x());
        axes.push_back(normal.normalized());
    }
    return axes;
}

double projectToAxis(const Vector2d &point, const Vector2d &axis) {
    return point.dot(axis);
}

bool checkOverlap(const std::vector<Vector2d> &corners1, const std::vector<Vector2d> &corners2, const Vector2d &axis) {
    double min1 = projectToAxis(corners1[0], axis);
    double max1 = min1;
    for (const auto &corner : corners1) {
        double projection = projectToAxis(corner, axis);
        min1 = std::min(min1, projection);
        max1 = std::max(max1, projection);
    }

    double min2 = projectToAxis(corners2[0], axis);
    double max2 = min2;
    for (const auto &corner : corners2) {
        double projection = projectToAxis(corner, axis);
        min2 = std::min(min2, projection);
        max2 = std::max(max2, projection);
    }

    return min1 <= max2 && min2 <= max1;
}

bool checkCollision(const Vehicle &v1, const Vehicle &v2) {
    std::vector<Vector2d> corners1 = getVehicleCorners(v1);
    std::vector<Vector2d> corners2 = getVehicleCorners(v2);

    std::vector<Vector2d> axes1 = getAxes(corners1);
    std::vector<Vector2d> axes2 = getAxes(corners2);

    for (const auto &axis : axes1) {
        if (!checkOverlap(corners1, corners2, axis)) {
            return false;
        }
    }

    for (const auto &axis : axes2) {
        if (!checkOverlap(corners1, corners2, axis)) {            
            return false;
        }
    }

    return true;
}

int main() {
    Vehicle vehicle1 = {{0.0, 0.0}, 4.0, 2.0, 0.0};
    Vehicle vehicle2 = {{5.0, 0.0}, 4.0, 2.0, 0.0};

    if (checkCollision(vehicle1, vehicle2)) {
        std::cout << "Collision detected between the two vehicles." << std::endl;
    } else {
        std::cout << "No collision detected between the two vehicles." << std::endl;
    }

    return 0;
}
