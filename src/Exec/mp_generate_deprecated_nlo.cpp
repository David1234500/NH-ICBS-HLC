#include <iostream>
#include <nlopt.hpp>
#include <chrono>
#include <DynamicsModel/SingleTrackModel.hpp>




struct objective_args_t{
    double mu_pd = 2.0f;
    double mu_hd = 1.0f;
    double mu_vd = 1.0f;
    double mu_vccd = 0.0f;

    double a_v = 20.f;
    double a_p = 10.f;
    double a_h = 0.15f * PI;

    dynamics::data::Pose2D sp = {{0,0},0,0};
    dynamics::data::Pose2D tp = {{30.f,20.f},0,100.f};

    double ts_ms = timestep_ms;
};

enum obj_position_variable_mapping{
    c1_st_a = 0,
    c2_st_a = 1,
    c1_vcc = 2,
    c2_vcc = 3
};



// Define the objective function
double objective(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
    objective_args_t* args = static_cast<objective_args_t*>(f_data);
    
    double v1 =  (0.5f * x[c1_vcc]) + args->sp.vel;
    double v2 = (0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel;
    auto p1 = dynamics::SimpleDynamicsModel::computeNextPose(args->sp,x[c1_st_a], v1, args->ts_ms);
    auto p2 = dynamics::SimpleDynamicsModel::computeNextPose(p1, x[c2_st_a], v2 , args->ts_ms);

    double pd = (args->tp.pos - p2.pos).norm(); 
    double hd =  std::abs(args->tp.h - p2.h);
    double vd =  std::abs(x[c1_vcc] + x[c2_vcc] + args->sp.vel - args->tp.vel);
    double vccd = std::abs(v1) + std::abs(v2);

    double objv = args->mu_hd * hd + args->mu_pd * pd + args->mu_vd * vd + args->mu_vccd * vccd;
    return objv;
}

double constraint_position_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
    objective_args_t* args = static_cast<objective_args_t*>(f_data);
    double v1 =  (0.5f * x[c1_vcc]) + args->sp.vel;
    double v2 = (0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel;
    auto p1 = dynamics::SimpleDynamicsModel::computeNextPose(args->sp,x[c1_st_a], v1, args->ts_ms);
    auto p2 = dynamics::SimpleDynamicsModel::computeNextPose(p1, x[c2_st_a], v2 , args->ts_ms);
    return (args->tp.pos - p2.pos).norm() - args->a_p; 
}

double constraint_heading_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
    objective_args_t* args = static_cast<objective_args_t*>(f_data);
    double v1 =  (0.5f * x[c1_vcc]) + args->sp.vel;
    double v2 = (0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel;
    auto p1 = dynamics::SimpleDynamicsModel::computeNextPose(args->sp,x[c1_st_a], v1, args->ts_ms);
    auto p2 = dynamics::SimpleDynamicsModel::computeNextPose(p1, x[c2_st_a], v2 , args->ts_ms);
    return std::abs(args->tp.h - p2.h) - args->a_h;
}

double constraint_velocity_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
    objective_args_t* args = static_cast<objective_args_t*>(f_data);
    double v1 =  (0.5f * x[c1_vcc]) + args->sp.vel;
    double v2 = (0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel;
    auto p1 = dynamics::SimpleDynamicsModel::computeNextPose(args->sp,x[c1_st_a], v1, args->ts_ms);
    auto p2 = dynamics::SimpleDynamicsModel::computeNextPose(p1, x[c2_st_a], v2 , args->ts_ms);
    return std::abs(x[c1_vcc] + x[c2_vcc] + args->sp.vel - args->tp.vel) - args->a_v;
}



int main() {

    std::cout << "start" << std::endl;
    // Create an instance of the NLopt optimizer
    // nlopt::opt optimizer(nlopt::GN_ISRES, 4); //obj 1.0
    // nlopt::opt optimizer(nlopt::GN_ESCH, 4); //obj 1.1
    // nlopt::opt optimizer(nlopt::GN_AGS, 4); //obj 1.6
    // nlopt::opt optimizer(nlopt::GN_MLSL_LDS, 4); //obj 1.4
    nlopt::opt optimizer(nlopt::GN_ORIG_DIRECT, 4); //obj 0.9
    // nlopt::opt optimizer(nlopt::GN_DIRECT, 4); //obj 0.9
    // nlopt::opt optimizer(nlopt::GN_DIRECT_L_RAND_NOSCAL, 4); //obj 0.7
    

    // Set the objective function
    objective_args_t args;
    optimizer.set_min_objective(objective, static_cast<void*>(&args));

    std::vector<double> lb = {0,0,0.0,0.0};
    optimizer.set_lower_bounds(lb);
    std::vector<double> ub = {2.f*PI,2.f*PI,100.0,100.0};
    optimizer.set_upper_bounds(ub);

    // optimizer.add_inequality_constraint(constraint_heading_delta, static_cast<void*>(&args), 0.01f);
    // optimizer.add_inequality_constraint(constraint_position_delta, static_cast<void*>(&args), 0.01f);
    // optimizer.add_inequality_constraint(constraint_velocity_delta, static_cast<void*>(&args), 0.01f);

    double vhd = (args.sp.vel + args.tp.vel) / 2;
    std::vector<double> x0 = {PI,PI,vhd,0};

    // Set the stopping criteria
    optimizer.set_maxtime(3.f);
    optimizer.set_stopval(1.0f);

    // Optimize the objective function subject to the constraint
    
    double minf;
    auto start = std::chrono::system_clock::now();

    nlopt::result result = optimizer.optimize(x0, minf);

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << " ms \n";
    

    // Print the results
    std::cout << "Optimization result: " << result << std::endl;
    std::cout << "Minimum objective function value: " << minf << std::endl;
    std::cout << "Optimal solution: (" << x0[0] << ", " << x0[1] << ", " << x0[2] << ", " << x0[3] << ")" << std::endl;

    return 0;
}