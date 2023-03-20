#include <iostream>
#include <nlopt.hpp>



// Define the objective function
double objective(unsigned n, const double* x, double* grad, void* data) {
    double result = x[0]*x[0] + x[1]*x[1];
    if (grad) {
        grad[0] = 2.0*x[0];
        grad[1] = 2.0*x[1];
    }
    return result;
}

// Define the nonlinear constraint function
double constraint(unsigned n, const double* x, double* grad, void* data) {
    double result = x[0] - x[1]*x[1];
    if (grad) {
        grad[0] = 1.0;
        grad[1] = -2.0*x[1];
    }
    return result;
}

int main() {
    // Create an instance of the NLopt optimizer
    nlopt::opt optimizer(nlopt::LD_MMA, 2);

    // Set the objective function
    optimizer.set_min_objective(objective, nullptr);

    // Add the nonlinear constraint function
    double tol = 1e-8;
    optimizer.add_inequality_constraint(constraint, nullptr, tol);

    // Set the initial guess
    std::vector<double> x0 = { 1.0, 1.0 };

    // Set the stopping criteria
    optimizer.set_xtol_rel(1e-6);

    // Optimize the objective function subject to the constraint
    double minf;
    nlopt::result result = optimizer.optimize(x0, minf);

    // Print the results
    std::cout << "Optimization result: " << result << std::endl;
    std::cout << "Minimum objective function value: " << minf << std::endl;
    std::cout << "Optimal solution: (" << x0[0] << ", " << x0[1] << ")" << std::endl;

    return 0;
}