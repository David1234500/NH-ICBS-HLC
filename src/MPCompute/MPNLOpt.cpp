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
    double solvetime = 0.f;
    while(!m_terminateMPThreads){
        
        bool hasTask = false;
        MPTask threadTask;
       
        
        m_mpTaskMutex.lock();
        if(!m_mpTaskQueue.empty()){
            threadTask = *m_mpTaskQueue.begin();
            m_mpTaskQueue.erase(m_mpTaskQueue.begin());
            rlog("workerMPEdges", LOG_INFO, std::to_string(m_mpTaskQueue.size()) + " Working now on task: " + std::to_string(threadTask.txi) +":"+ std::to_string(threadTask.tyi) + ":"+ std::to_string(threadTask.cai) +":"+ std::to_string(threadTask.csi));
            m_solvetimes[index] = solvetime;
            hasTask = true;
        }else{
            hasTask = false;
        }
        if(m_mpTaskQueue.size() % 10 == 0){
            double avg = 0.f;
            for(auto t: m_solvetimes){
                avg += (1.f/32.f) * t.second;
            }
            rlog("workerMPEdges", LOG_INFO, "Average solve time: " + std::to_string(avg) + "[ms] ESTTC: " + std::to_string((avg * m_mpTaskQueue.size())/60000.f) + "[m]" );
        }

        m_mpTaskMutex.unlock();

        if(hasTask){
           
            int32_t h_sw_beg = ((threadTask.cai - (map_size_angle / 4)) + map_size_angle) % map_size_angle;
            int32_t h_sw_end = ((threadTask.cai + (map_size_angle / 4)) + map_size_angle) % map_size_angle;

            while(h_sw_beg != h_sw_end){
            
                //Compute best fit settings set to get from the current to the target location
                dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi, h_sw_beg, threadTask.tsi};
                
                MPNLOptSingle::mpnl_args_t args;
                args.sp = {{0.f,0.f}, api * static_cast<float>(threadTask.cai), m_speedsFactor[threadTask.csi] * dynamics::SimpleDynamicsModel::velocity_limit()};
                args.tp = {{(xpc*threadTask.txi), (ypc*threadTask.tyi)},  api * static_cast<float>(h_sw_beg), m_speedsFactor[threadTask.tsi] * dynamics::SimpleDynamicsModel::velocity_limit()};

                MPNLOptSingle nl_stm_opt;
                nl_stm_opt.prepare(args, true, false, nlopt::GN_ISRES);
                auto res = nl_stm_opt.optimize(); 
                solvetime = 0.1 * static_cast<double>(res.rt_ms) + 0.9 * solvetime;
                if(res.retcode <= 0 || res.retcode >= 4){
                    h_sw_beg = (h_sw_beg + 1) % map_size_angle;
                    continue;
                }

                auto p_r = MPNLOptSingle::eval_two(res.result, &args);
                
                // Q1 MP
                dynamics::data::Pose2DWithError epose;
                epose = p_r;
                epose = target_pose_by_index;

                epose.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                epose.s_v = args.sp.vel + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_vcc];
                epose.s_v_2 = epose.s_v + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_vcc];
                epose.bi_pose = target_pose_by_index;
                MotionPrimitive mp_q1 = {epose, target_pose_by_index};

                rlog("workerMPEdges", LOG_INFO,"Added transition with " + std::to_string(res.retcode) +  ":" + std::to_string(res.objf_val) +" to " + std::to_string(epose.pos[0]) +":"+ std::to_string(epose.pos[1]) 
                + " -> s " + std::to_string(epose.s_a) +"_"+ std::to_string(epose.s_v) +  " st " + std::to_string(res.rt_ms));

                // Q2 MP
                double h_q2 = p_r.h + 0.5 * PI;
                Eigen::Rotation2Df rot_q2(0.5 * PI);
                dynamics::data::Position2Df pd_q2 = rot_q2 * p_r.pos;
                uint32_t hi_q2 = h_sw_beg + (map_size_angle/4);
                
                dynamics::data::Pose2D p_q2 = {pd_q2, h_q2, p_r.vel};
                dynamics::data::PoseByIndex pi_q2 = {round(pd_q2[0] / xpc),round(pd_q2[1] / ypc), hi_q2,  threadTask.tsi};

                dynamics::data::Pose2DWithError epose_q2;
                epose_q2 = p_q2;
                epose_q2.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose_q2.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                epose_q2.s_v = args.sp.vel + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_vcc];
                epose_q2.s_v_2 = epose.s_v + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_vcc];
                epose_q2.bi_pose = pi_q2;
                
                MotionPrimitive mp_q2 = {epose_q2, pi_q2};
                
                // Q3 MP
                double h_q3 = p_r.h + PI;
                Eigen::Rotation2Df rot_q3(PI);
                dynamics::data::Position2Df pd_q3 = rot_q3 * p_r.pos;
                uint32_t hi_q3 = h_sw_beg + (map_size_angle/2);
                
                dynamics::data::Pose2D p_q3 = {pd_q3, h_q3, p_r.vel};
                dynamics::data::PoseByIndex pi_q3 = {round(pd_q3[0] / xpc),round(pd_q3[1] / ypc), hi_q3,  threadTask.tsi};

                dynamics::data::Pose2DWithError epose_q3;
                epose_q3 = p_q3;
                epose_q3.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose_q3.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                epose_q3.s_v = args.sp.vel + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_vcc];
                epose_q3.s_v_2 = epose.s_v + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_vcc];
                epose_q3.bi_pose = pi_q3;
                MotionPrimitive mp_q3 = {epose_q3, pi_q3};

                // Q4 MP
                double h_q4 = p_r.h + 1.5f * PI;
                Eigen::Rotation2Df rot_q4(1.5f * PI);
                dynamics::data::Position2Df pd_q4 = rot_q4 * p_r.pos;
                uint32_t hi_q4 = h_sw_beg + 3.f * (map_size_angle/4.f);
                
                dynamics::data::Pose2D p_q4 = {pd_q4, h_q4, p_r.vel};
                dynamics::data::PoseByIndex pi_q4 = {round(pd_q4[0] / xpc),round(pd_q4[1] / ypc), hi_q4,  threadTask.tsi};

                dynamics::data::Pose2DWithError epose_q4;
                epose_q4 = p_q4;
                epose_q4.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose_q4.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                epose_q4.s_v = args.sp.vel + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_vcc];
                epose_q4.s_v_2 = epose.s_v + 0.5f * res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_vcc];
                epose_q4.bi_pose = pi_q4;
                MotionPrimitive mp_q4 = {epose_q4, pi_q4};

                m_mpTaskMutex.lock();
                m_mpmap[threadTask.cai                       ][threadTask.csi].push_back(mp_q1);
                m_mpmap[threadTask.cai +   (map_size_angle/4)][threadTask.csi].push_back(mp_q2);
                m_mpmap[threadTask.cai +   (map_size_angle/2)][threadTask.csi].push_back(mp_q3);
                m_mpmap[threadTask.cai + 3*(map_size_angle/4)][threadTask.csi].push_back(mp_q4);  
                m_mpTaskMutex.unlock();
                h_sw_beg = (h_sw_beg + 1) % map_size_angle;

                // m_terminateMPThreads = true;
                // break;
            }
    
        }else{
            return;
        }
        hasTask = false;
    }
}
