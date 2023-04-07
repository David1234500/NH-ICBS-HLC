#ifndef MPNLOPTPARAM
#define MPNLOPTPARAM
#include <Config.hpp>
#include <iostream>
#include <nlopt.hpp>
#include <chrono>
#include <DynamicsModel/SingleTrackModel.hpp>

#include <memory>
#include <vector>
#include <algorithm>
#include <random>

std::vector<std::vector<double>> generateRandomSamples(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, int rand_sample_count) {
    std::vector<std::vector<double>> randomSamples;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<double>> dist(lowerBound.size());
    for (int i = 0; i < lowerBound.size(); i++) {
        dist[i] = std::uniform_real_distribution<double>(lowerBound[i], upperBound[i]);
    }

    for (int i = 0; i < rand_sample_count; i++) {
        std::vector<double> sample(lowerBound.size());
        for (int dim = 0; dim < lowerBound.size(); dim++) {
            sample[dim] = dist[dim](gen);
        }
        randomSamples.push_back(sample);
    }

    return randomSamples;
}


class MPNLOptParameters{
public:


    struct mpnl_param_weights_t{
        double mu_l = 0.0f;
        double mu_lth = 0.0f;
        double mu_vd = 0.0f;
        double mu_hth = 0.f;
        double mu_sum_he = 0.f;
        double mu_sum_pe = 0.f;
    };

    struct mpnl_param_args_t{
        mpnl_param_weights_t w;
        double ts_ms = 0.f;
        
        std::vector<double> init_guess = {};
        bool relax_constraints = false;

        double obj_lam_target = 0.f;
        std::vector<double> ub;
        std::vector<double> lb;
        
        double maxt = 0.f;
        double st_val = 0.f;
    };

    enum mpnl_param_obj_pv_map{
        c_lamb = 0,
        c_lamth = 1,
        c_hth = 2,
        c_st_1 = 3,
        c_st_2 = 4,
        c_st_3 = 5,
        c_st_4 = 6,
        c_st_5 = 7,
        c_st_6 = 8,
        c_st_7 = 9,
        c_st_8 = 10,
        c_st_9 = 11,
        c_st_10 = 12,
        c_st_11 = 13,
        c_st_12 = 14,
        c_st_21 = 15,
        c_st_22 = 16,
        c_st_23 = 17,
        c_st_24 = 18,
        c_st_25 = 19,
        c_st_26 = 20,
        c_st_27 = 21,
        c_st_28 = 22,
        c_st_29 = 23,
        c_st_210 = 24,
        c_st_211 = 25,
        c_st_212 = 26,
        c_acc_1 = 27,
        c_acc_2 = 28,
        c_acc_3 = 29,
        c_acc_4 = 30,
        c_acc_5 = 31,
        c_acc_6 = 32,
        c_acc_7 = 33,
        c_acc_8 = 34,
        c_acc_9 = 35,
        c_acc_10 = 36,
        c_acc_11 = 37,
        c_acc_12 = 38
    };


    struct mpnl_param_constraint_args{
        dynamics::data::Pose2D tp = {{0,0},0,0};
        dynamics::data::Pose2D sp = {{0,0},0,0};
        
        std::vector<double> init_guess = {};

        std::vector<float> lam_factor = {-1.f,-1.f};
        
        int32_t st_var_1 = -1;
        int32_t st_var_2 = -1;

        int32_t acc_var_id = -1;
        bool acc = true;
        
        mpnl_param_args_t* args;
    };

    struct mpnl_param_return{
        std::vector<double> result = {};
        double objf_val = 0.f;
        uint64_t rt_ms = 0;
        nlopt::result retcode;
    };  

    struct mpnl_param_obj_args{
        mpnl_param_args_t* model_args;
        std::vector<mpnl_param_constraint_args*> constraint_args;
    };


    static dynamics::data::Pose2D eval_two_acc(std::vector<double> x, float st1, float st2, dynamics::data::Pose2D sp, float ts, int32_t acc_id){
        auto p1 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(sp, st1, x[acc_id], ts);
        auto p2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(p1, st2, x[acc_id], ts);
        return p2;
    }

    static dynamics::data::Pose2D eval_two_steady(std::vector<double> x, float st1, float st2, dynamics::data::Pose2D sp, float ts, int32_t acc_id){
        sp.vel = 2.f * x[acc_id] * (ts / 1000.f);
        auto p1 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(sp, st1, 0, ts);
        auto p2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(p1, st2, 0, ts);
        return p2;
    }


    static void compute_tp(const std::vector <double> &x, MPNLOptParameters::mpnl_param_constraint_args* pc_args){
        if(pc_args->lam_factor[0] > 0.01f){
            pc_args->tp.pos[0] = pc_args->lam_factor[0] * x[c_lamb];
        }
        if(pc_args->lam_factor[1] > 0.01f){
            pc_args->tp.pos[1] = pc_args->lam_factor[1] * x[c_lamb];
        }
    }


    static double objective(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_param_obj_args* args = static_cast<mpnl_param_obj_args*>(f_data);
        
        double cumulative_position_error = 0.f;
        double cumulative_heading_error = 0.f;
        for(auto config: args->constraint_args){
            compute_tp(x,config);

            dynamics::data::Pose2D p2;
            if(config->acc){
                p2 = eval_two_acc(x, x[config->st_var_1], x[config->st_var_2], config->sp, config->args->ts_ms, config->acc_var_id);
            }else{
                p2 = eval_two_steady(x,x[config->st_var_1], x[config->st_var_2], config->sp, config->args->ts_ms, config->acc_var_id);
            }

            cumulative_position_error += (config->tp.pos - p2.pos).norm();

            double hd = std::abs(config->tp.h - p2.h);
            double hd_n = hd <= PI ? hd : 2 * PI - hd; 
            cumulative_heading_error += hd_n;
        }

        double objv = args->model_args->w.mu_hth * std::abs(x[c_hth]) + args->model_args->w.mu_lth * std::abs(x[c_lamth]) + args->model_args->w.mu_sum_pe * cumulative_position_error + args->model_args->w.mu_sum_he * cumulative_heading_error;
        std::cout << "obj: " << std::to_string(objv)  << ", lambda: " << std::to_string( x[c_lamb])  << ", heading thres: " << std::to_string(x[c_hth])  
                  << ", lambda thres: " << std::to_string(std::abs(x[c_lamth])) 
                  << ", sum pos error: " << std::to_string(cumulative_position_error)
                  << ", sum heading error: " << std::to_string(cumulative_heading_error) << std::endl;

        return objv;
    }



    static double constraint_position_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        auto pc_args = static_cast<mpnl_param_constraint_args*>(f_data);
        if(pc_args->lam_factor[0] > 0.01f){
            pc_args->tp.pos[0] = pc_args->lam_factor[0] * x[c_lamb];
        }
        if(pc_args->lam_factor[1] > 0.01f){
            pc_args->tp.pos[1] = pc_args->lam_factor[1] * x[c_lamb];
        }

        dynamics::data::Pose2D p2;
        if(pc_args->acc){
            p2 = eval_two_acc(x,x[pc_args->st_var_1], x[pc_args->st_var_2], pc_args->sp, pc_args->args->ts_ms, pc_args->acc_var_id);
        }else{
            p2 = eval_two_steady(x,x[pc_args->st_var_1], x[pc_args->st_var_2], pc_args->sp, pc_args->args->ts_ms, pc_args->acc_var_id);
        }
        
        // std::cout << "PConstraint: " << std::to_string((pc_args->tp.pos - p2.pos).norm()) << ":" <<  x[c_lamb] << std::endl;
        // std::cout << "p2 " << std::to_string(p2.pos[0]) << ", " << std::to_string (p2.pos[1]) << ", " << std::to_string(p2.h) << ", " << p2.vel << std::endl;
        // std::cout << "tp " << std::to_string(pc_args->tp.pos[0]) << ", " << std::to_string (pc_args->tp.pos[1]) << ", " << std::to_string(pc_args->tp.h) << ", " << pc_args->tp.vel << std::endl;
        // std::cout << "sp " << std::to_string(pc_args->sp.pos[0]) << ", " << std::to_string (pc_args->sp.pos[1]) << ", " << std::to_string(pc_args->sp.h) << ", " << pc_args->sp.vel << std::endl;
        // std::cout << std::endl;

        return (pc_args->tp.pos - p2.pos).norm() - x[c_lamth]; 
    }

    static double constraint_heading_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        auto pc_args = static_cast<mpnl_param_constraint_args*>(f_data);

        dynamics::data::Pose2D p2;
        if(pc_args->acc){
            p2 = eval_two_acc(x,x[pc_args->st_var_1], x[pc_args->st_var_2], pc_args->sp, pc_args->args->ts_ms, pc_args->acc_var_id);
        }else{
            p2 = eval_two_steady(x,x[pc_args->st_var_1], x[pc_args->st_var_2], pc_args->sp, pc_args->args->ts_ms, pc_args->acc_var_id);
        }

        double hd = std::abs(pc_args->tp.h - p2.h);
        double hd_n = hd <= PI ? hd : 2 * PI - hd; 
        return hd_n - x[c_hth];
    }

    MPNLOptParameters(){}

    mpnl_param_obj_args* prepare(mpnl_param_args_t* args,nlopt::algorithm alg=nlopt::GN_ISRES){
        m_optimizer = std::make_shared<nlopt::opt>(alg, 39); 

        Config& config = Config::getInstance();
        args->w.mu_l = config.get<double>({"mpnl_param_weights", "mu_l"});
        args->w.mu_lth = config.get<double>({"mpnl_param_weights", "mu_lth"});
        args->w.mu_vd = config.get<double>({"mpnl_param_weights", "mu_vd"});
        args->w.mu_hth = config.get<double>({"mpnl_param_weights", "mu_hth"});
        args->w.mu_sum_he = config.get<double>({"mpnl_param_weights", "mu_sum_he"});
        args->w.mu_sum_pe = config.get<double>({"mpnl_param_weights", "mu_sum_pe"});

        args->ts_ms = config.get<double>({"mpnl_param_args", "ts_ms"});
        args->obj_lam_target = config.get<double>({"mpnl_param_args", "obj_lam_target"});
        args->ub = config.get<std::vector<double>>({"mpnl_param_args", "ub"});
        args->lb = config.get<std::vector<double>>({"mpnl_param_args", "lb"});
        args->maxt = config.get<double>({"mpnl_param_args", "maxt"});
        args->st_val = config.get<double>({"mpnl_param_args", "st_val"});
        args->relax_constraints = config.get<bool>({"mpnl_param_args", "relax_constraints"});
        
        std::vector<double>* lower = new std::vector<double>(c_acc_12 + 1);
        lower->at(c_lamb) = args->lb[0];
        lower->at(c_lamth) = args->lb[1];
        lower->at(c_hth) = args->lb[2];
        for(uint32_t i = c_st_1; i <= c_st_212; i ++){
            lower->at(i) = args->lb[3];
        }
        for(uint32_t i = c_acc_1; i <= c_acc_12; i ++){
            lower->at(i) = args->lb[4];
        }
        m_optimizer->set_lower_bounds(*lower);
         
        std::cout << "Real Lower: [";
        for(auto val: *lower){
            std::cout << std::to_string(val) << ", ";
        }
        std::cout <<  "]" << std::endl;

        std::vector<double>* upper = new std::vector<double>(c_acc_12 + 1);
        upper->at(c_lamb) = args->ub[0];
        upper->at(c_lamth) = args->ub[1];
        upper->at(c_hth) = args->ub[2];
        for(uint32_t i = c_st_1; i <= c_st_7; i ++){
            upper->at(i) = args->ub[3];
        }
        for(uint32_t i = c_acc_1; i <= c_acc_12; i ++){
            upper->at(i) = args->ub[4];
        }
        m_optimizer->set_upper_bounds(*upper);

        std::cout << "Real Upper: [";
        for(auto val: *upper){
            std::cout << std::to_string(val) << ", ";
        }
        std::cout <<  "]" << std::endl;


        float hsteps =  Config::getInstance().get<float>({"map","angle_steps"});
        float dstep =  Config::getInstance().get<float>({"disc","dstep"});
        float hstep =  Config::getInstance().get<float>({"disc","hstep"});

        std::vector<mpnl_param_constraint_args*> configurations;

        // PS -> P1 H0
        // mpnl_param_constraint_args* c0_args = new mpnl_param_constraint_args;
        // c0_args->args = args;
        // c0_args->sp = {{0, 0}, 0, 0};
        // c0_args->tp = {{0, 0}, 0, 0};
        // c0_args->st_var_1 = c_st_1;
        // c0_args->st_var_1 = c_st_21;
        // c0_args->acc_var_id = c_acc_1;
        // c0_args->lam_factor = {1.f,0.f};
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c0_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c0_args), 0.01f);
        // }
        // configurations.push_back(c0_args);

        // PS -> P2 H1
        mpnl_param_constraint_args* c1_args = new mpnl_param_constraint_args;
        c1_args->args = args;
        c1_args->sp = {{0, 0}, hstep, 0};
        c1_args->tp = {{0, 0}, hstep, 0};
        c1_args->st_var_1 = c_st_2;
        c1_args->st_var_2 = c_st_22;
        c1_args->acc_var_id = c_acc_2;
        c1_args->lam_factor = {1.f,1.f};
        if(!args->relax_constraints){
            m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c1_args), 0.01f);
            m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c1_args), 0.01f);
        }
        configurations.push_back(c1_args);

        // // PS -> P1 H1 with ST1
        // mpnl_param_constraint_args* c2_args = new mpnl_param_constraint_args;
        // c2_args->args = args;
        // c2_args->sp = {{0, 0}, hstep, 0};
        // c2_args->tp = {{0, 0}, 7 * hstep, 0};
        // c2_args->st_var_id = c_st_1;
        // c2_args->acc_var_id = c_acc_3;
        // c2_args->lam_factor = {1.f,0.f};
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c2_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c2_args), 0.01f);
        // }
        // configurations.push_back(c2_args);

        // // PS -> P2 H0 with ST2
        // mpnl_param_constraint_args* c3_args = new mpnl_param_constraint_args;
        // c3_args->args = args;
        // c3_args->sp = {{0, 0}, 0, 0};
        // c3_args->tp = {{0, 0}, 7 * hstep, 0};
        // c3_args->st_var_id = c_st_2;
        // c3_args->acc_var_id = c_acc_4;
        // c3_args->lam_factor = {1.f,1.f};
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c3_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c3_args), 0.01f);
        // }
        // configurations.push_back(c3_args);

        // // PS -> P3 H2 
        // mpnl_param_constraint_args* c4_args = new mpnl_param_constraint_args;
        // c4_args->args = args;
        // c4_args->sp = {{0, 0}, 2 * hstep, 0};
        // c4_args->tp = {{0, 0}, 2 * hstep, 0};
        // c4_args->st_var_id = -1;
        // c4_args->acc_var_id = c_acc_5;
        // c4_args->lam_factor = {0.f,1.f};
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c4_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c4_args), 0.01f);
        // }
        // configurations.push_back(c4_args);

        // // PS -> P3 H1 with ST3
        // mpnl_param_constraint_args* c5_args = new mpnl_param_constraint_args;
        // c5_args->args = args;
        // c5_args->sp = {{0, 0},  hstep, 0};
        // c5_args->tp = {{0, 0}, 3 * hstep, 0};
        // c5_args->st_var_id = c_st_3;
        // c5_args->acc_var_id = c_acc_6;
        // c5_args->lam_factor = {0.f,1.f};
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c5_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c5_args), 0.01f);
        // }
        // configurations.push_back(c5_args);

        // // PS -> P4 H0
        // mpnl_param_constraint_args* c6_args = new mpnl_param_constraint_args;
        // c6_args->args = args;
        // c6_args->sp = {{0, 0}, 0, 0};
        // c6_args->tp = {{0, 0}, 0, 0};
        // c6_args->st_var_id = -1;
        // c6_args->acc_var_id = c_acc_7;
        // c6_args->lam_factor = {2.f,0.f};
        // c6_args->acc = false;
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c6_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c6_args), 0.01f);
        // }
        // configurations.push_back(c6_args);

        // // PS -> P5 H0 H1 with ST4
        // mpnl_param_constraint_args* c7_args = new mpnl_param_constraint_args;
        // c7_args->args = args;
        // c7_args->sp = {{0, 0}, 0, 0};
        // c7_args->tp = {{0, 0}, hstep, 0};
        // c7_args->st_var_id = c_st_4;
        // c7_args->acc_var_id = c_acc_8;
        // c7_args->lam_factor = {2.f,1.f};
        // c7_args->acc = false;
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c7_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c7_args), 0.01f);
        // }
        // configurations.push_back(c7_args);

        // // PS -> P5 H1 H1 with ST5
        // mpnl_param_constraint_args* c8_args = new mpnl_param_constraint_args;
        // c8_args->args = args;
        // c8_args->sp = {{0, 0}, hstep, 0};
        // c8_args->tp = {{0, 0}, hstep, 0};
        // c8_args->st_var_id = c_st_5;
        // c8_args->acc_var_id = c_acc_9;
        // c8_args->lam_factor = {2.f,1.f};
        // c8_args->acc = false;
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c8_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c8_args), 0.01f);
        // }
        // configurations.push_back(c8_args);

        // // PS -> P5 H1 H0 with ST6
        // mpnl_param_constraint_args* c9_args = new mpnl_param_constraint_args;
        // c9_args->args = args;
        // c9_args->sp = {{0, 0}, hstep, 0};
        // c9_args->tp = {{0, 0}, 0, 0};
        // c9_args->st_var_id = c_st_6;
        // c9_args->acc_var_id = c_acc_10;
        // c9_args->lam_factor = {2.f,1.f};
        // c9_args->acc = false;
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c9_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c9_args), 0.01f);
        // }
        // configurations.push_back(c9_args);

        // // PS -> P6 H1 H1 
        // mpnl_param_constraint_args* c10_args = new mpnl_param_constraint_args;
        // c10_args->args = args;
        // c10_args->sp = {{0, 0}, hstep, 0};
        // c10_args->tp = {{0, 0}, hstep, 0};
        // c10_args->st_var_id = c_st_6;
        // c10_args->acc_var_id = c_acc_11;
        // c10_args->lam_factor = {2.f,2.f};
        // c10_args->acc = false;
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c10_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c10_args), 0.01f);
        // }
        // configurations.push_back(c10_args);

        // // PS -> P6 H0 H1 with ST7
        // mpnl_param_constraint_args* c11_args = new mpnl_param_constraint_args;
        // c11_args->args = args;
        // c11_args->sp = {{0, 0}, 0, 0};
        // c11_args->tp = {{0, 0}, hstep, 0};
        // c11_args->st_var_id = c_st_7;
        // c11_args->acc_var_id = c_acc_12;
        // c11_args->lam_factor = {2.f,2.f};
        // c11_args->acc = false;
        // if(!args->relax_constraints){
        //     m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c11_args), 0.01f);
        //     m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c11_args), 0.01f);
        // }
        // configurations.push_back(c11_args);

        // Set the stopping criteria
        m_result = *upper;
        m_optimizer->set_maxtime(args->maxt);
        m_optimizer->set_stopval(args->st_val);
        
        auto ob_args = new mpnl_param_obj_args;
        ob_args->model_args = args;
        ob_args->constraint_args = configurations;
        m_optimizer->set_min_objective(objective, static_cast<void*>(ob_args));
        return ob_args;
    }

    mpnl_param_return optimize(){
        if(m_optimizer == nullptr){
            return mpnl_param_return();
        }

        mpnl_param_return retval;
        
        auto start = std::chrono::system_clock::now();
        retval.retcode = m_optimizer->optimize(m_result, retval.objf_val);
        auto end = std::chrono::system_clock::now();
        retval.rt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        retval.result = m_result;

        std::cout <<  "retcode:" << retval.retcode << std::endl;
        std::cout <<  "rt_ms:" << retval.rt_ms << std::endl;
        std::cout <<  "objf:" << retval.objf_val << std::endl;
        std::cout <<  "result vector [";
        for(auto val: retval.result){
            std::cout << std::to_string(val) << ", ";
        }
        std::cout <<  "]" << std::endl;;

        return retval;
    }

    void visualize_results(mpnl_param_return* result, mpnl_param_obj_args* obj_args, int32_t id, bool success){
        float hsteps =  Config::getInstance().get<float>({"map","angle_steps"});
        float dstep =  Config::getInstance().get<float>({"disc","dstep"});
        float hstep =  Config::getInstance().get<float>({"disc","hstep"});
        float timestep_ms =  Config::getInstance().get<float>({"timestep_ms"});

        json res;

        json map;
        for(uint32_t x = 0; x < 3; x++){
            for(uint32_t y = 0; y < 3; y++){
                json node;
                node["x"] = x * result->result[c_lamb];
                node["y"] = y * result->result[c_lamb];
                map.push_back(node);
            }
        }
        res["map"] = map;
        
        for(auto config: obj_args->constraint_args){
            compute_tp(result->result,config);

            dynamics::data::Pose2D p2;
            if(config->acc){
                p2 = eval_two_acc(result->result, result->result[config->st_var_1],result->result[config->st_var_2], config->sp, config->args->ts_ms, config->acc_var_id);
            }else{
                p2 = eval_two_steady(result->result,result->result[config->st_var_1],result->result[config->st_var_2], config->sp, config->args->ts_ms, config->acc_var_id);
            }

            double pos = (config->tp.pos - p2.pos).norm();
            double hd = std::abs(config->tp.h - p2.h);
            double hd_n = hd <= PI ? hd : 2 * PI - hd; 

            json edge;
            edge["ID"] = "pos_e: " + std::to_string(pos) + " head_e: " + std::to_string(hd_n);
            edge["edge"] = predict(config->sp, result->result[ config->acc_var_id], result->result[ config->acc_var_id],result->result[config->st_var_1],result->result[config->st_var_2]);
            res["edges"].push_back(edge);
        }

        std::ofstream o("mp_param_res" + std::to_string(id) + "S" +  std::to_string(success) + ".json");
        o << res.dump() << std::endl;
        o.close();
    }

    json predict(dynamics::data::Pose2D veh_pose, float acc1, float acc2, float st_angle1, float st_angl2){
        float hsteps =  Config::getInstance().get<float>({"map","angle_steps"});
        float dstep =  Config::getInstance().get<float>({"disc","dstep"});
        float timestep_ms =  Config::getInstance().get<float>({"timestep_ms"});
        float hstep =  Config::getInstance().get<float>({"disc","hstep"});

        json jedge;

        dynamics::data::Pose2D next_pose;
        for(float ts = 0; ts <= timestep_ms; ts += 50.f){
            next_pose = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(veh_pose, st_angle1, acc1, ts);
            json point;
            point["x"] = next_pose.pos[0];
            point["y"] = next_pose.pos[1];
            jedge["curve"].push_back(point);

        }
    
        for(float ts = 0; ts <= timestep_ms; ts += 50.f){
            auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(next_pose, st_angl2, acc2, ts);
            json point;
            point["x"] = next_pose2.pos[0];
            point["y"] = next_pose2.pos[1];
            jedge["curve"].push_back(point);
        }
        return  jedge;
    }


private:
    std::shared_ptr<nlopt::opt> m_optimizer = nullptr;
    std::vector<double> m_result;
};


#endif