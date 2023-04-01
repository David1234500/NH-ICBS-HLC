#include <iostream>
#include <nlopt.hpp>
#include <chrono>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <memory>

class MPNLOptSingle{
public:


    struct mpnl_weights_t{
        double mu_pd = 0.f;
        double mu_hd = 0.f;
        double mu_vd = 0.f;
        double mu_accd = 0.f;
        double mu_sccd = 0.f;
    };

    struct mpnl_args_t{
        dynamics::data::Pose2D sp = {{0,0},0,0};
        dynamics::data::Pose2D tp = {{0,0},0,0};
        mpnl_weights_t w;
        double ts_ms = 0.f;
        std::vector<double> ub;
        std::vector<double> lb;
        double maxt = 0.f;
        double st_val = 0.f;

        double a_v = 0.f;
        double a_p = 0.f;
        double a_h = 0.f;
        double a_scc = 0.f;
    };

    enum mpnl_obj_pv_map{
        c1_st_a = 0,
        c2_st_a = 1,
        c1_acc = 2,
        c2_acc = 3
    };

    struct mpnl_return{
        std::vector<double> result = {};
        double objf_val = 0.f;
        uint64_t rt_ms = 0;
        nlopt::result retcode;
    };  


    static dynamics::data::Pose2D eval_two(std::vector<double> x, mpnl_args_t* args){
        auto p1 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(args->sp,x[c1_st_a], x[c1_acc], args->ts_ms);
        auto p2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(p1,      x[c2_st_a], x[c2_acc], args->ts_ms);
        return p2;
    }


    static double objective(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        
        auto p2 = eval_two(x,args);

        double hd = std::abs(args->tp.h - p2.h);
        double hd_n = hd <= PI ? hd : 2 * PI - hd;

        double pd =  (args->tp.pos - p2.pos).norm(); 
        
        double vd =  std::abs(p2.vel - args->tp.vel);
        double accd = std::abs(x[c1_acc]) + std::abs(x[c2_acc]);
        double sccd = std::abs(x[c1_st_a]) + std::abs(x[c2_st_a]);

        double objv = args->w.mu_hd * hd_n + args->w.mu_pd * pd + args->w.mu_vd * vd + args->w.mu_accd * accd + args->w.mu_sccd * sccd;
        return objv;
    }

    static double constraint_position_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        auto p2 = eval_two(x,args);
        return (args->tp.pos - p2.pos).norm() - args->a_p; 
    }

    static double constraint_heading_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        auto p2 = eval_two(x,args);
        double hd = std::abs(args->tp.h - p2.h);
        double hd_n = hd <= PI ? hd : 2 * PI - hd; 
        return hd_n - args->a_h;
    }

   static double constraint_velocity_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        auto p2 = eval_two(x,args);
        return std::abs(p2.vel - args->tp.vel) - args->a_v;
    }

    static double constraint_steering_change(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
       
        return std::abs(x[c1_st_a]) + std::abs(x[c2_st_a]) - args->a_scc;
    }

    


    MPNLOptSingle(){}


    void prepare(mpnl_args_t* args, bool enforce_constraints=false, bool enforce_steering_change_limit=false,nlopt::algorithm alg=nlopt::GN_ISRES){
        m_optimizer = std::make_shared<nlopt::opt>(alg, 4); 
        m_optimizer->set_min_objective(objective, static_cast<void*>(args));

        Config& config = Config::getInstance();

        args->w.mu_pd = config.get<double>({"mpnl_weights", "mu_pd"});
        args->w.mu_hd = config.get<double>({"mpnl_weights", "mu_hd"});
        args->w.mu_vd = config.get<double>({"mpnl_weights", "mu_vd"});
        args->w.mu_accd = config.get<double>({"mpnl_weights", "mu_vccd"});
        args->w.mu_sccd = config.get<double>({"mpnl_weights", "mu_sccd"});
        
        args->ts_ms = config.get<double>({"mpnl_args", "ts_ms"});
        
        if(args->sp.vel < 0.f || args->tp.vel < 0.f){
            args->ub = config.get<std::vector<double>>({"mpnl_args", "ubr"});
            args->lb = config.get<std::vector<double>>({"mpnl_args", "lbr"});
        }else{
            args->ub = config.get<std::vector<double>>({"mpnl_args", "ubf"});
            args->lb = config.get<std::vector<double>>({"mpnl_args", "lbf"});
        }
        
        args->maxt = config.get<double>({"mpnl_args", "maxt"});
        args->st_val = config.get<double>({"mpnl_args", "st_val"});
        args->a_v = config.get<double>({"mpnl_args", "a_v"});
        args->a_p = config.get<double>({"mpnl_args", "a_p"});
        args->a_h = config.get<double>({"mpnl_args", "a_h"});
        args->a_scc = config.get<double>({"mpnl_args", "a_scc"});

        m_optimizer->set_lower_bounds(args->lb);
        m_optimizer->set_upper_bounds(args->ub);

        if(enforce_constraints){
            m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(args), 0.01f);
            m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(args), 0.01f);
            m_optimizer->add_inequality_constraint(constraint_velocity_delta, static_cast<void*>(args), 0.01f);
            if(enforce_steering_change_limit){
                m_optimizer->add_inequality_constraint(constraint_steering_change, static_cast<void*>(args), 0.01f);
            }
        }

        m_result = {0,0,0,0};

        // Set the stopping criteria
        m_optimizer->set_maxtime(args->maxt);
        m_optimizer->set_stopval(args->st_val);
    }

    mpnl_return optimize(){
        if(m_optimizer == nullptr){
            return mpnl_return();
        }

        mpnl_return retval;
        
        auto start = std::chrono::system_clock::now();
        retval.retcode = m_optimizer->optimize(m_result, retval.objf_val);
        auto end = std::chrono::system_clock::now();
        retval.rt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        retval.result = m_result;

        return retval;
    }


private:
    std::shared_ptr<nlopt::opt> m_optimizer = nullptr;
    std::vector<double> m_result;
};

