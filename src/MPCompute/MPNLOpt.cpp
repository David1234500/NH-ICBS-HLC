#include <MPCompute/MPNLOpt.hpp>
#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>
#include <MPCompute/MPNLOptSingle.hpp>

#include <vector>
#include <set>
#include <Eigen/Dense>

MPNLOpt::MPNLOpt(){
    
}


void MPNLOpt::workerThreadMPEdges(uint32_t index){
    int32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().getXstep();
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});
    bool mpnl_debug = Config::getInstance().get<bool>({"mpnl_debug"});
    
    double solvetime = 0.f;
    while(!m_terminateMPThreads){
        
        bool hasTask = false;
        MPTask threadTask;
       
        m_mpTaskMutex.lock();
        if(!m_mpTaskQueue.empty()){
            threadTask = *m_mpTaskQueue.begin();
            m_mpTaskQueue.erase(m_mpTaskQueue.begin());
            rlog("workerMPEdges", LOG_INFO, std::to_string(m_mpTaskQueue.size()) + " Working now on task: " + std::to_string(threadTask.cai) +":"+ std::to_string(threadTask.csi) +" -> "+ std::to_string(threadTask.txi) +":"+ std::to_string(threadTask.tyi) + ":"+ std::to_string(threadTask.tsi));
            m_solvetimes[index] = solvetime;
            hasTask = true;
        }else{
            hasTask = false;
        }
        if(m_mpTaskQueue.size() % 10 == 0){
            double avg = 0.f;
            for(auto t: m_solvetimes){
                avg += (1.f/30.f) * t.second;
            }
            rlog("workerMPEdges", LOG_INFO, "Average solve time: " + std::to_string(avg) + "[ms] ESTTC: " + std::to_string(((avg * m_mpTaskQueue.size())/60000.f) / 15.f) + "[m]" );
        }

        m_mpTaskMutex.unlock();

        if(hasTask){

            int start_angle = (threadTask.cai - threadTask.mhd + map_size_angle) % map_size_angle;
            int end_angle = (threadTask.cai + threadTask.mhd + map_size_angle) % map_size_angle;
            for (int tsh = start_angle; tsh != end_angle; tsh = (tsh + 1) % map_size_angle) {
            
                //Compute best fit settings set to get from the current to the target location
                dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi, tsh, threadTask.tsi};
                
                MPNLOptSingle::mpnl_args_t args;
                args.sp = {{0.f,0.f}, api * static_cast<float>(threadTask.cai), vlevels[threadTask.csi] * dynamics::SimpleDynamicsModel::velocity_limit()};
                args.tp = {{(dpc*threadTask.txi), (dpc*threadTask.tyi)},  api * static_cast<float>(tsh), vlevels[threadTask.tsi] * dynamics::SimpleDynamicsModel::velocity_limit()};

                MPNLOptSingle nl_stm_opt;
                nl_stm_opt.prepare(&args, true, false);
                
                auto res = nl_stm_opt.optimize(); 
                solvetime = 0.1 * static_cast<double>(res.rt_ms) + 0.9 * solvetime;

                auto p1 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(args.sp, res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a], res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc], timestep_ms);
                auto pr = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(p1,      res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a], res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc], timestep_ms);
                
                if(mpnl_debug){
                    std::cout << "src" << std::to_string(0) << ", " << std::to_string (0) << ", " << std::to_string(api * static_cast<float>(threadTask.cai)) << ", " << vlevels[threadTask.csi] * dynamics::SimpleDynamicsModel::velocity_limit() << std::endl;
                    std::cout << "targ" << std::to_string(dpc*threadTask.txi) << ", " << std::to_string (dpc*threadTask.tyi) << ", " << std::to_string(api * static_cast<float>(tsh)) << ", " << vlevels[threadTask.tsi] * dynamics::SimpleDynamicsModel::velocity_limit() << std::endl;
                    std::cout << "obj " << std::to_string(res.objf_val) << std::endl;
                    std::cout << "retcode " << std::to_string(res.retcode) << std::endl;
                    std::cout << "rt " << std::to_string(res.rt_ms) << std::endl;
                    std::cout << "c_st_a " << std::to_string(res.result[0]) << ", "<< std::to_string(res.result[1]) << std::endl;
                    std::cout << "c_vcc " << std::to_string(res.result[2]) << ", "<< std::to_string(res.result[3]) << std::endl;
                    std::cout << "res_pose ((" << std::to_string(pr.pos[0]) << ", "<< std::to_string(pr.pos[1]) << "), " + std::to_string(pr.h) +", " + std::to_string(pr.vel) + ")"<< std::endl;
                    std::cout << std::endl;
                }

                if(res.retcode <= 0 || res.retcode >= 4){
                    continue;
                }

                // Q1 MP
                dynamics::data::Pose2DWithError epose;
                epose = pr;
                epose = target_pose_by_index;

                epose.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                double v_end_1 = args.sp.vel + res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc] * (timestep_ms / 1000.f);
                double v_end_2 = v_end_1 + res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc] * (timestep_ms / 1000.f);

                epose.s_v = (args.sp.vel + v_end_1) / 2;
                epose.s_v_2 = (v_end_1 + v_end_2) / 2;

                epose.bi_pose = target_pose_by_index;
                MotionPrimitive mp_q1 = {epose, target_pose_by_index};

                rlog("workerMPEdges", LOG_INFO,"Added transition with " + std::to_string(res.retcode) +  ":" + std::to_string(res.objf_val) +": [" + std::to_string(threadTask.cai) +":"+ std::to_string(threadTask.csi) 
                + "] ->  [" + std::to_string(res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a]) + ", " + std::to_string(res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a])
                + ", " + std::to_string(res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc])  + ", " + std::to_string(res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc]) + "] -> "
                + "[" + std::to_string(epose.pos[0])  + ", " + std::to_string(epose.pos[1]) + ", " + std::to_string(epose.h) + ", " + std::to_string(epose.vel)+ "]");


                // Q2 MP
                double h_q2 = pr.h + 0.5 * PI;
                Eigen::Rotation2Df rot_q2(0.5 * PI);
                dynamics::data::Position2Df pd_q2 = rot_q2 * pr.pos;
                uint32_t hi_q2 = (tsh + (map_size_angle/4)) % map_size_angle;
                
                dynamics::data::Pose2D p_q2 = {pd_q2, h_q2, pr.vel};
                dynamics::data::PoseByIndex pi_q2 = {round(pd_q2[0] / dpc),round(pd_q2[1] / dpc), hi_q2,  threadTask.tsi};

                dynamics::data::Pose2DWithError epose_q2;
                epose_q2 = p_q2;
                epose_q2.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose_q2.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                epose_q2.s_v = (args.sp.vel + v_end_1) / 2;
                epose_q2.s_v_2 = (v_end_1 + v_end_2) / 2;
                epose_q2.bi_pose = pi_q2;
                
                MotionPrimitive mp_q2 = {epose_q2, pi_q2};
                
                // Q3 MP
                double h_q3 = pr.h + PI;
                Eigen::Rotation2Df rot_q3(PI);
                dynamics::data::Position2Df pd_q3 = rot_q3 * pr.pos;
                uint32_t hi_q3 = (tsh + (map_size_angle/2)) % map_size_angle;
                
                dynamics::data::Pose2D p_q3 = {pd_q3, h_q3, pr.vel};
                dynamics::data::PoseByIndex pi_q3 = {round(pd_q3[0] / dpc),round(pd_q3[1] / dpc), hi_q3,  threadTask.tsi};

                dynamics::data::Pose2DWithError epose_q3;
                epose_q3 = p_q3;
                epose_q3.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose_q3.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                epose_q3.s_v = (args.sp.vel + v_end_1) / 2;
                epose_q3.s_v_2 = (v_end_1 + v_end_2) / 2;
                epose_q3.bi_pose = pi_q3;
                MotionPrimitive mp_q3 = {epose_q3, pi_q3};

                // Q4 MP
                double h_q4 = pr.h + 1.5f * PI;
                Eigen::Rotation2Df rot_q4(1.5f * PI);
                dynamics::data::Position2Df pd_q4 = rot_q4 * pr.pos;
                uint32_t hi_q4 = (tsh + 3 * (map_size_angle/4)) % map_size_angle;
                
                dynamics::data::Pose2D p_q4 = {pd_q4, h_q4, pr.vel};
                dynamics::data::PoseByIndex pi_q4 = {round(pd_q4[0] / dpc),round(pd_q4[1] / dpc), hi_q4,  threadTask.tsi};

                dynamics::data::Pose2DWithError epose_q4;
                epose_q4 = p_q4;
                epose_q4.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose_q4.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                epose_q4.s_v = (args.sp.vel + v_end_1) / 2;
                epose_q4.s_v_2 = (v_end_1 + v_end_2) / 2;
                epose_q4.bi_pose = pi_q4;
                MotionPrimitive mp_q4 = {epose_q4, pi_q4};

                m_mpTaskMutex.lock();
                m_mpmap[threadTask.cai                       ][threadTask.csi].push_back(mp_q1);
                m_mpmap[threadTask.cai +   (map_size_angle/4)][threadTask.csi].push_back(mp_q2);
                m_mpmap[threadTask.cai +   (map_size_angle/2)][threadTask.csi].push_back(mp_q3);
                m_mpmap[threadTask.cai + 3*(map_size_angle/4)][threadTask.csi].push_back(mp_q4);  
                m_mpTaskMutex.unlock();
                
            }
    
        }else{
            return;
        }
        hasTask = false;
    }
}
