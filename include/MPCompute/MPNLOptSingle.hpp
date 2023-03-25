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
        double mu_vccd = 0.f;
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
    };

    enum mpnl_obj_pv_map{
        c1_st_a = 0,
        c2_st_a = 1,
        c1_vcc = 2,
        c2_vcc = 3
    };

    struct mpnl_return{
        std::vector<double> result = {};
        double objf_val = 0.f;
        uint64_t rt_ms = 0;
        nlopt::result retcode;
    };  


    static dynamics::data::Pose2D eval_two(std::vector<double> x, mpnl_args_t* args){
        double v1 = (0.5f * x[c1_vcc]) + args->sp.vel;
        double v2 = (0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel;
        auto p1 = dynamics::SimpleDynamicsModel::computeNextPose(args->sp,x[c1_st_a], v1, args->ts_ms);
        auto p2 = dynamics::SimpleDynamicsModel::computeNextPose(p1, x[c2_st_a], v2 , args->ts_ms);
        return p2;
    }


    static double objective(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        
        auto p2 = eval_two(x,args);

        double hd = std::abs(args->tp.h - p2.h);
        double hd_n = hd <= PI ? hd : 2 * PI - hd;

        double pd =  (args->tp.pos - p2.pos).norm(); 
        
        double vd =  std::abs((0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel - args->tp.vel);
        double vccd = std::abs(x[c1_vcc]) + std::abs(x[c2_vcc]);

        double objv = args->w.mu_hd * hd_n + args->w.mu_pd * pd + args->w.mu_vd * vd + args->w.mu_vccd * vccd;
        
        // std::cout << "obj nadj: " << std::to_string(objv)  <<" [" << std::to_string(pd) << ", " << std::to_string(hd_n) << ", " << std::to_string(vd) << ", " << std::to_string(vccd) << "]" << std::endl;
        // std::cout << "obj  adj: " << std::to_string(objv)  <<" [" << std::to_string( args->w.mu_pd * pd) << ", " << std::to_string(args->w.mu_hd * hd_n) << ", " << std::to_string(args->w.mu_vd * vd) << ", " << std::to_string(args->w.mu_vccd * vccd) << "]" << std::endl;
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
        return hd_n- args->a_h;
    }

    static double constraint_velocity_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        auto p2 = eval_two(x,args);
        return std::abs((0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel - args->tp.vel) - args->a_v;
    }


    MPNLOptSingle(){}


    void prepare(mpnl_args_t* args, bool enforce_constraints=false, bool complete_eval=false,nlopt::algorithm alg=nlopt::GN_ISRES){
        m_optimizer = std::make_shared<nlopt::opt>(alg, 4); 
        m_optimizer->set_min_objective(objective, static_cast<void*>(&args));

        Config& config = Config::getInstance("path/to/your/config.json");

        args->w.mu_pd = config.get<double>({"mpnl_weights", "mu_pd"});
        args->w.mu_hd = config.get<double>({"mpnl_weights", "mu_hd"});
        args->w.mu_vd = config.get<double>({"mpnl_weights", "mu_vd"});
        args->w.mu_vccd = config.get<double>({"mpnl_weights", "mu_vccd"});
        
        args->ts_ms = config.get<double>({"mpnl_args", "ts_ms"});
        args->ub = config.get<std::vector<double>>({"mpnl_args", "ub"});
        args->lb = config.get<std::vector<double>>({"mpnl_args", "lb"});
        args->maxt = config.get<double>({"mpnl_args", "maxt"});
        args->st_val = config.get<double>({"mpnl_args", "st_val"});
        args->a_v = config.get<double>({"mpnl_args", "a_v"});
        args->a_p = config.get<double>({"mpnl_args", "a_p"});
        args->a_h = config.get<double>({"mpnl_args", "a_h"});

        m_optimizer->set_lower_bounds(args->lb);
        m_optimizer->set_upper_bounds(args->ub);

        if(enforce_constraints){
            m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(args), 0.01f);
            m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(args), 0.01f);
            m_optimizer->add_inequality_constraint(constraint_velocity_delta, static_cast<void*>(args), 0.01f);
        }

        double vhd = (args->sp.vel + args->tp.vel) / 4;
        m_result = {0,0,vhd,vhd};

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

