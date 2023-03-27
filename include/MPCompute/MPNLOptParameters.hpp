#ifndef MPNLOPTPARAM
#define MPNLOPTPARAM
#include <Config.hpp>
#include <iostream>
#include <nlopt.hpp>
#include <chrono>
#include <DynamicsModel/SingleTrackModel.hpp>

#include <memory>


class MPNLOptParameters{
public:


    struct mpnl_param_weights_t{
        double mu_l = 0.0f;
        double mu_lth = 0.0f;
        double mu_vd = 0.0f;
        double mu_hth = 0.f;
    };

    struct mpnl_param_args_t{
        mpnl_param_weights_t w;
        double ts_ms = 0.f;
        
        double obj_lam_target = 0.f;
        std::vector<double> ub;
        std::vector<double> lb;
        
        double maxt = 0.f;
        double st_val = 0.f;
    };

    enum mpnl_param_obj_pv_map{
        c_lamb = 0,
        c_acc_1 = 1,
        c_acc_2 = 2,
        c_lamth = 3,
        c_st_1 = 4,
        c_st_2 = 5,
        c_st_3 = 6,
        c_st_4 = 7,
        c_st_5 = 8,
        c_st_6 = 9,
        c_st_7 = 10,
        c_hth = 11
    };

    struct mpnl_param_constraint_args{
        dynamics::data::Pose2D tp = {{0,0},0,0};
        dynamics::data::Pose2D sp = {{0,0},0,0};
        std::vector<float> lam_factor = {-1.f,-1.f};
        int32_t depents_on_st = -1;
        bool accel = true;
        mpnl_param_args_t* args;
    };

    struct mpnl_param_return{
        std::vector<double> result = {};
        double objf_val = 0.f;
        uint64_t rt_ms = 0;
        nlopt::result retcode;
    };  


    static dynamics::data::Pose2D eval_two_acc(std::vector<double> x, float st, dynamics::data::Pose2D sp, float ts){
        auto p1 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(sp, st, x[c_acc_1], ts);
        auto p2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(p1, st, x[c_acc_2], ts);
        return p2;
    }

    static dynamics::data::Pose2D eval_two_steady(std::vector<double> x, float st, dynamics::data::Pose2D sp, float ts){
        auto p1 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(sp, st, 0, ts);
        auto p2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(p1, st, 0, ts);
        return p2;
    }


    static double objective(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        mpnl_param_args_t* args = static_cast<mpnl_param_args_t*>(f_data);
        
        double objv = args->w.mu_l * (x[c_lamb] - args->obj_lam_target) - std::abs(x[c_st_1])  - std::abs(x[c_st_2]) - std::abs(x[c_st_3]) + args->w.mu_hth * std::abs(x[c_hth]) - args->w.mu_lth * std::abs(x[c_lamth]);

        std::cout << "obj: " << std::to_string(objv)  <<" [" << std::to_string( x[c_lamb] - args->obj_lam_target)  << ", " << std::to_string(- std::abs(x[c_st_1]) - std::abs(x[c_st_2] - std::abs(x[c_st_3]))) << ", " << std::to_string(std::abs(x[c_hth])) << ", " <<std::to_string(std::abs(x[c_lamth])) << "]" << std::endl;
        return objv;
    }



    static double constraint_position_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        static auto pc_args = static_cast<mpnl_param_constraint_args*>(f_data);
        if(pc_args->lam_factor[0] > 0.01f){
            pc_args->tp.pos[0] = pc_args->lam_factor[0] * x[c_lamb];
        }
        if(pc_args->lam_factor[1] > 0.01f){
            pc_args->tp.pos[1] = pc_args->lam_factor[1] * x[c_lamb];
        }
        
        float st = 0.f;
        if(pc_args->depents_on_st > 0){
            st = x[pc_args->depents_on_st];
        }

        dynamics::data::Pose2D p2;
        if(pc_args->accel){
            p2 = eval_two_acc(x,st,pc_args->sp, pc_args->args->ts_ms);
        }else{
            pc_args->sp.vel = (x[c_acc_1] + x[c_acc_2]) * (pc_args->args->ts_ms / 1000.f);
            p2 = eval_two_steady(x,st,pc_args->sp, pc_args->args->ts_ms);
        }
        
        std::cout << "PConstraint: " << std::to_string((pc_args->tp.pos - p2.pos).norm()) << ":" << (pc_args->tp.pos - p2.pos).norm() - x[c_lamth] << std::endl;
        std::cout << "tp " << std::to_string(p2.pos[0]) << ", " << std::to_string (p2.pos[1]) << ", " << std::to_string(p2.h) << ", " << p2.vel << std::endl;
        std::cout << "p2 " << std::to_string(pc_args->tp.pos[0]) << ", " << std::to_string (pc_args->tp.pos[1]) << ", " << std::to_string(pc_args->tp.h) << ", " << pc_args->tp.vel << std::endl;
        
        return (pc_args->tp.pos - p2.pos).norm() - x[c_lamth]; 
    }

    static double constraint_heading_delta(const std::vector <double> &x, std::vector<double> &grad, void* f_data){
        static auto pc_args = static_cast<mpnl_param_constraint_args*>(f_data);
        float st = 0.f;
        if(pc_args->depents_on_st > 0){
            st = x[pc_args->depents_on_st];
        }
        
        dynamics::data::Pose2D p2;
        if(pc_args->accel){
            p2 = eval_two_acc(x, st, pc_args->sp, pc_args->args->ts_ms);
        }else{
            pc_args->sp.vel = (x[c_acc_1] + x[c_acc_2]) * (pc_args->args->ts_ms / 1000.f);
            p2 = eval_two_steady(x,st,pc_args->sp, pc_args->args->ts_ms);
        }

        double hd = std::abs(pc_args->tp.h - p2.h);
        double hd_n = hd <= PI ? hd : 2 * PI - hd; 
        return hd_n - x[c_hth];
    }

    MPNLOptParameters(){}

    void prepare(mpnl_param_args_t* args, bool enforce_constraints=false, bool complete_eval=false,nlopt::algorithm alg=nlopt::GN_ISRES){
        m_optimizer = std::make_shared<nlopt::opt>(alg, 9); 
        m_optimizer->set_min_objective(objective, static_cast<void*>(args));

        Config& config = Config::getInstance();
        args->w.mu_l = config.get<double>({"mpnl_param_weights", "mu_l"});
        args->w.mu_lth = config.get<double>({"mpnl_param_weights", "mu_lth"});
        args->w.mu_vd = config.get<double>({"mpnl_param_weights", "mu_vd"});
        args->w.mu_hth = config.get<double>({"mpnl_param_weights", "mu_hth"});

        args->ts_ms = config.get<double>({"mpnl_param_args", "ts_ms"});
        args->obj_lam_target = config.get<double>({"mpnl_param_args", "obj_lam_target"});
        args->ub = config.get<std::vector<double>>({"mpnl_param_args", "ub"});
        args->lb = config.get<std::vector<double>>({"mpnl_param_args", "lb"});
        args->maxt = config.get<double>({"mpnl_param_args", "maxt"});
        args->st_val = config.get<double>({"mpnl_param_args", "st_val"});

        m_optimizer->set_lower_bounds(args->lb);
        m_optimizer->set_upper_bounds(args->ub); 

        float hsteps =  Config::getInstance().get<float>({"map","angle_steps"});
        float dstep =  Config::getInstance().get<float>({"disc","dstep"});
        float hstep =  Config::getInstance().get<float>({"disc","hstep"});

        // PS -> P1 H0
        mpnl_param_constraint_args* c0_args = new mpnl_param_constraint_args;
        c0_args->args = args;
        c0_args->sp = {{0, 0}, 0, 0};
        c0_args->tp = {{0, 0}, 0, 0};
        c0_args->depents_on_st = -1;
        c0_args->lam_factor = {1.f,0.f};
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c0_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c0_args), 0.01f);

        // PS -> P2 H1
        mpnl_param_constraint_args* c1_args = new mpnl_param_constraint_args;
        c1_args->args = args;
        c1_args->sp = {{0, 0}, hstep, 0};
        c1_args->tp = {{0, 0}, hstep, 0};
        c1_args->depents_on_st = -1;
        c1_args->lam_factor = {1.f,1.f};
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c1_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c1_args), 0.01f);

        // PS -> P1 H1 with ST1
        mpnl_param_constraint_args* c2_args = new mpnl_param_constraint_args;
        c2_args->args = args;
        c2_args->sp = {{0, 0}, hstep, 0};
        c2_args->tp = {{0, 0}, 7 * hstep, 0};
        c2_args->depents_on_st = c_st_1;
        c2_args->lam_factor = {1.f,0.f};
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c2_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c2_args), 0.01f);

        // PS -> P2 H0 with ST2
        mpnl_param_constraint_args* c3_args = new mpnl_param_constraint_args;
        c3_args->args = args;
        c3_args->sp = {{0, 0}, 0, 0};
        c3_args->tp = {{0, 0}, 7 * hstep, 0};
        c3_args->depents_on_st = c_st_2;
        c3_args->lam_factor = {1.f,1.f};
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c3_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c3_args), 0.01f);

        // PS -> P3 H2 
        mpnl_param_constraint_args* c4_args = new mpnl_param_constraint_args;
        c4_args->args = args;
        c4_args->sp = {{0, 0}, 2 * hstep, 0};
        c4_args->tp = {{0, 0}, 2 * hstep, 0};
        c4_args->depents_on_st = -1;
        c4_args->lam_factor = {0.f,1.f};
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c4_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c4_args), 0.01f);

        // PS -> P3 H1 with ST3
        mpnl_param_constraint_args* c5_args = new mpnl_param_constraint_args;
        c5_args->args = args;
        c5_args->sp = {{0, 0},  hstep, 0};
        c5_args->tp = {{0, 0}, 3 * hstep, 0};
        c5_args->depents_on_st = c_st_3;
        c5_args->lam_factor = {0.f,1.f};
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c5_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c5_args), 0.01f);

        // PS -> P4 H0
        mpnl_param_constraint_args* c6_args = new mpnl_param_constraint_args;
        c6_args->args = args;
        c6_args->sp = {{0, 0}, 0, 0};
        c6_args->tp = {{0, 0}, 0, 0};
        c6_args->depents_on_st = -1;
        c6_args->lam_factor = {2.f,0.f};
        c6_args->accel = false;
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c6_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c6_args), 0.01f);

        // PS -> P5 H0 H1 with ST4
        mpnl_param_constraint_args* c7_args = new mpnl_param_constraint_args;
        c7_args->args = args;
        c7_args->sp = {{0, 0}, 0, 0};
        c7_args->tp = {{0, 0}, hstep, 0};
        c7_args->depents_on_st = c_st_4;
        c7_args->lam_factor = {2.f,1.f};
        c7_args->accel = false;
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c7_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c7_args), 0.01f);


        // PS -> P5 H1 H1 with ST5
        mpnl_param_constraint_args* c8_args = new mpnl_param_constraint_args;
        c8_args->args = args;
        c8_args->sp = {{0, 0}, hstep, 0};
        c8_args->tp = {{0, 0}, hstep, 0};
        c8_args->depents_on_st = c_st_5;
        c8_args->lam_factor = {2.f,1.f};
        c8_args->accel = false;
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c8_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c8_args), 0.01f);

        // PS -> P5 H1 H0 with ST6
        mpnl_param_constraint_args* c9_args = new mpnl_param_constraint_args;
        c9_args->args = args;
        c9_args->sp = {{0, 0}, hstep, 0};
        c9_args->tp = {{0, 0}, 0, 0};
        c9_args->depents_on_st = c_st_6;
        c9_args->lam_factor = {2.f,1.f};
        c9_args->accel = false;
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c9_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c9_args), 0.01f);

        // PS -> P6 H1 H1 
        mpnl_param_constraint_args* c10_args = new mpnl_param_constraint_args;
        c10_args->args = args;
        c10_args->sp = {{0, 0}, hstep, 0};
        c10_args->tp = {{0, 0}, hstep, 0};
        c10_args->depents_on_st = c_st_6;
        c10_args->lam_factor = {2.f,2.f};
        c10_args->accel = false;
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c10_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c10_args), 0.01f);

        // PS -> P6 H0 H1 with ST7
        mpnl_param_constraint_args* c11_args = new mpnl_param_constraint_args;
        c11_args->args = args;
        c11_args->sp = {{0, 0}, 0, 0};
        c11_args->tp = {{0, 0}, hstep, 0};
        c11_args->depents_on_st = c_st_7;
        c11_args->lam_factor = {2.f,2.f};
        c11_args->accel = false;
        m_optimizer->add_inequality_constraint(constraint_position_delta, static_cast<void*>(c11_args), 0.01f);
        m_optimizer->add_inequality_constraint(constraint_heading_delta, static_cast<void*>(c11_args), 0.01f);

        // Set the stopping criteria
        m_result = args->lb;
        m_optimizer->set_maxtime(args->maxt);
        m_optimizer->set_stopval(args->st_val);
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

        std::cout <<  "result vector [";
        for(auto val: retval.result){
            std::cout << std::to_string(val) << ", ";
        }
        std::cout <<  "]" << std::endl;;

        return retval;
    }

    void visualize_results(mpnl_param_return* result){
        float hsteps =  Config::getInstance().get<float>({"map","angle_steps"});
        float dstep =  Config::getInstance().get<float>({"disc","dstep"});
        float hstep =  Config::getInstance().get<float>({"disc","hstep"});

        json res;

        json edge_0;
        edge_0["ID"] = "PS -> P1 H0";
        dynamics::data::Pose2D sp0 = {{0, 0}, 0, 0};
        edge_0["edge"] = predict(sp0, result->result[c_acc_1], result->result[c_acc_2], 0);
        res.push_back(edge_0);

        json edge_1;
        edge_1["ID"] = "PS -> P2 H1";
        dynamics::data::Pose2D sp1 = {{0, 0}, hstep, 0};
        edge_1["edge"] = predict(sp1, result->result[c_acc_1], result->result[c_acc_2], 0);
        res.push_back(edge_1);

        json edge_2;
        edge_2["ID"] = "PS -> P1 H1 with ST1";
        dynamics::data::Pose2D sp2 = {{0, 0}, hstep, 0};
        edge_2["edge"] = predict(sp2, result->result[c_acc_1], result->result[c_acc_2], result->result[c_st_1]);
        res.push_back(edge_2);

        json edge_3;
        edge_3["ID"] = "PS -> P2 H0 with ST2";
        dynamics::data::Pose2D sp3 = {{0, 0}, 0, 0};
        edge_3["edge"] = predict(sp3, result->result[c_acc_1], result->result[c_acc_2], result->result[c_st_2]);
        res.push_back(edge_3);

        json edge_4;
        edge_4["ID"] = "PS -> P3 H2";
        dynamics::data::Pose2D sp4 = {{0, 0}, 2 * hstep, 0};
        edge_4["edge"] = predict(sp4, result->result[c_acc_1], result->result[c_acc_2], 0);
        res.push_back(edge_4);

        json edge_5;
        edge_5["ID"] = "PS -> P3 H1 with ST3";
        dynamics::data::Pose2D sp5 = {{0, 0}, hstep, 0};
        edge_5["edge"] = predict(sp5, result->result[c_acc_1], result->result[c_acc_2], result->result[c_st_3]);
        res.push_back(edge_5);

        json edge_6;
        edge_6["ID"] = "PS -> P4 H0";
        dynamics::data::Pose2D sp6 = {{0, 0}, 0, 0};
        edge_6["edge"] = predict(sp6, result->result[c_acc_1], result->result[c_acc_2], 0);
        res.push_back(edge_6);

        json edge_7;
        edge_7["ID"] = "PS -> P5 H0 H1 with ST4";
        dynamics::data::Pose2D sp7 = {{0, 0}, 0, 0};
        edge_7["edge"] = predict(sp7, result->result[c_acc_1], result->result[c_acc_2], result->result[c_st_4]);
        res.push_back(edge_7);

        json edge_8;
        edge_8["ID"] = "PS -> P5 H1 H1 with ST5";
        dynamics::data::Pose2D sp8 = {{0, 0}, hstep, 0};
        edge_8["edge"] = predict(sp8, result->result[c_acc_1], result->result[c_acc_2], result->result[c_st_5]);
        res.push_back(edge_8);
        
        std::ofstream o("mp_param_res.json");
        o << res << std::endl;
        o.close();
    }

    json predict(dynamics::data::Pose2D veh_pose, float acc1, float acc2, float st_angle){
        float hsteps =  Config::getInstance().get<float>({"map","angle_steps"});
        float timestep_ms =  Config::getInstance().get<float>({"timestep_ms"});
        float dstep =  Config::getInstance().get<float>({"disc","dstep"});
        float hstep =  Config::getInstance().get<float>({"disc","hstep"});

        json jedge;

        dynamics::data::Pose2D next_pose;
        for(float ts = 0; ts <= timestep_ms; ts += 50.f){
            next_pose = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(veh_pose, st_angle, acc1, ts);
            json point;
            point["x"] = next_pose.pos[0];
            point["y"] = next_pose.pos[1];
            jedge["curve"].push_back(point);

        }
    
        for(float ts = 0; ts <= timestep_ms; ts += 50.f){
            auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(next_pose, st_angle, acc2, ts);
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