#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <nlopt.hpp>


struct Vehicle {
    Vector2d center;
    double length;
    double width;
    double orientation;
};

bool checkOverlap(const Vector2d &circle1, double radius1, const Vector2d &circle2, double radius2) {
    double distanceSquared = (circle1 - circle2).squaredNorm();
    double radiiSum = radius1 + radius2;
    return distanceSquared <= radiiSum * radiiSum;
}

bool checkCollision(const Vehicle &v1, const Vehicle &v2) {
    // Calculate half-diagonal length for each vehicle
    double halfDiagonal1 = 0.5 * std::sqrt(v1.length * v1.length + v1.width * v1.width);
    double halfDiagonal2 = 0.5 * std::sqrt(v2.length * v2.length + v2.width * v2.width);

    // Calculate circles' centers and radii
    Vector2d circle1 = v1.center;
    double radius1 = halfDiagonal1;

    Vector2d circle2 = v2.center;
    double radius2 = halfDiagonal2;

    // Check for overlap between the circles
    return checkOverlap(circle1, radius1, circle2, radius2);
}