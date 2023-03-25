#include <iostream>
#include <nlopt.hpp>
#include <chrono>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <memory>

class MPNLOptSingleDualStage{
public:


    struct mpnl_weights_t{
        double mu_pd = 1.0f;
        double mu_hd = 10.0f;
        double mu_vd = 0.5f;
        double mu_vccd = 0.01f;

        double a_v = 20.f;
        double a_p = 2.f;
        double a_h = 0.05f * PI;
    };

    struct mpnl_args_t{
        dynamics::data::Pose2D sp = {{0,0},0,0};
        dynamics::data::Pose2D tp = {{0,0},0,0};
        
        mpnl_weights_t w;
        double ts_ms = 500.f;
        
        std::vector<double> ub = { PI / 7.f, PI / 7.f, 200.0, 200.0};
        std::vector<double> lb = {-PI / 7.f,-PI / 7.f,0.f,0.0};
        double maxt = 2.f;
        double st_val = 5.f;
    };

    

    enum mpnl_obj_pv_map{
        c1_st_a = 0,
        c2_st_a = 1,
        c1_vcc = 2,
        c2_vcc = 3
    };

    struct mpnl_return{
        std::vector<double> result_prim = {};
        std::vector<double> result_sec = {};
        
        double objf_val_prim = 0.f;
        double objf_val_sec = 0.f;

        uint64_t rt_ms_prim = 0;
        uint64_t rt_ms_sec = 0;

        nlopt::result retcode_prim;
        nlopt::result retcode_sec;
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
        return objv;
    }

    static double constraint_position_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        auto p2 = eval_two(x,args);
        return (args->tp.pos - p2.pos).norm() - args->w.a_p; 
    }

    static double constraint_heading_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        auto p2 = eval_two(x,args);
        double hd = std::abs(args->tp.h - p2.h);
        double hd_n = hd <= PI ? hd : 2 * PI - hd; 
        return hd_n- args->w.a_h;
    }

    static double constraint_velocity_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_args_t* args = static_cast<mpnl_args_t*>(f_data);
        auto p2 = eval_two(x,args);
        return std::abs((0.5f * (x[c1_vcc] + x[c2_vcc])) + args->sp.vel - args->tp.vel) - args->w.a_v;
    }


    MPNLOptSingleDualStage(){}

    void prepare(mpnl_args_t args, bool enforce_constraints=false, bool complete_eval=false,nlopt::algorithm alg=nlopt::GN_ISRES){
        m_optimizer_prim = std::make_shared<nlopt::opt>(alg, 4); 
        m_optimizer_prim->set_min_objective(objective, static_cast<void*>(&args));

        m_optimizer_prim->set_lower_bounds(args.lb);
        m_optimizer_prim->set_upper_bounds(args.ub);

        if(enforce_constraints){
            m_optimizer_prim->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(&args), 0.01f);
            m_optimizer_prim->add_inequality_constraint(constraint_position_delta, static_cast<void*>(&args), 0.01f);
            m_optimizer_prim->add_inequality_constraint(constraint_velocity_delta, static_cast<void*>(&args), 0.01f);
        }

        m_optimizer_prim->set_maxtime(args.maxt);
        m_optimizer_prim->set_stopval(args.st_val);

        double vhd = (args.sp.vel + args.tp.vel) / 4;
        m_result = {0,0,vhd,vhd};

        m_optimizer_sec = std::make_shared<nlopt::opt>(nlopt::LN_COBYLA, 4); 
        m_optimizer_sec->set_min_objective(objective, static_cast<void*>(&args));

        m_optimizer_sec->set_lower_bounds(args.lb);
        m_optimizer_sec->set_upper_bounds(args.ub);

        if(enforce_constraints){
            m_optimizer_sec->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(&args), 0.01f);
            m_optimizer_sec->add_inequality_constraint(constraint_position_delta, static_cast<void*>(&args), 0.01f);
            m_optimizer_sec->add_inequality_constraint(constraint_velocity_delta, static_cast<void*>(&args), 0.01f);
        }

        m_optimizer_sec->set_maxtime(args.maxt);
        m_optimizer_sec->set_stopval(args.st_val);
    }

    mpnl_return optimize(){
        if(m_optimizer_prim == nullptr || m_optimizer_sec == nullptr){
            return mpnl_return();
        }

        mpnl_return retval;

        auto start = std::chrono::system_clock::now();
        retval.retcode_prim = m_optimizer_prim->optimize(m_result, retval.objf_val_prim);
        auto end = std::chrono::system_clock::now();
        retval.rt_ms_prim = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        retval.result_prim = m_result;

        start = std::chrono::system_clock::now();
        retval.retcode_sec = m_optimizer_sec->optimize(m_result, retval.objf_val_sec);
        end = std::chrono::system_clock::now();
        retval.rt_ms_sec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        
        retval.result_sec = m_result;

        return retval;
    }


private:
    std::shared_ptr<nlopt::opt> m_optimizer_prim = nullptr;
    std::shared_ptr<nlopt::opt> m_optimizer_sec = nullptr;
    std::vector<double> m_result;
};

