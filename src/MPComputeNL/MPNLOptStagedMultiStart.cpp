#include <MPComputeNL/MPNLOptSingle.hpp>
#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>
#include <MPComputeNL/MPNLOptStagedMultiStart.hpp>

#include <vector>
#include <set>
#include <random>
#include <Eigen/Dense>

MPNLOptStagedMultiStart::MPNLOptStagedMultiStart(){
    
}


std::vector<std::vector<double>> generateUniformSamples(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, int uni_sample_count) {
    std::vector<std::vector<double>> uniformSamples;

    double stepSize[4];
    for (int i = 0; i < 4; i++) {
        stepSize[i] = (upperBound[i] - lowerBound[i]) / (uni_sample_count - 1);
    }

    for (int i = 0; i < uni_sample_count; i++) {
        
        std::vector<double> sample = {
            lowerBound[0] + i * stepSize[0],
            lowerBound[1] + i * stepSize[1],
            lowerBound[2] + i * stepSize[2],
            lowerBound[3] + i * stepSize[3]
        };
        uniformSamples.push_back(sample);      
    }

    return uniformSamples;
}

std::vector<std::vector<double>> generateRandomSamples(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, int rand_sample_count) {
    std::vector<std::vector<double>> randomSamples;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist[4];
    for (int i = 0; i < 4; i++) {
        dist[i] = std::uniform_real_distribution<double>(lowerBound[i], upperBound[i]);
    }

    for (int i = 0; i < rand_sample_count; i++) {
        std::vector<double> sample = {
            dist[0](gen),
            dist[1](gen),
            dist[2](gen),
            dist[3](gen)
        };
        randomSamples.push_back(sample);
    }

    return randomSamples;
}

void MPNLOptStagedMultiStart::workerThreadMPEdges(uint32_t index){
    auto& config = Config::getInstance();
    int32_t timestep_ms = config.get<uint32_t>({"timestep_ms"});
    float dpc = config.get<float>({"disc","dstep"});
    float api = config.get<float>({"disc","hstep"});
    int32_t zero_velocity_level = config.get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = config.get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = config.get<int32_t>({"map","angle_steps"});
    int32_t x_steps = config.getXstep();
    int32_t worker_count = config.get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = config.get<std::vector<float>>({"velocity","vlevels"});
    bool mpnl_debug = config.get<bool>({"mpnl_debug"});
    int32_t uni_sample_count = config.get<int32_t>({"mpnl_multi_start_staged","uni_sample_count"});
    int32_t rand_sample_count = config.get<int32_t>({"mpnl_multi_start_staged","rand_sample_count"});
    
    double solvetime = 0.f;
    int32_t completedTasks = 0;
    double totalTime = 0.0;
    std::chrono::steady_clock::time_point taskStartTime;
    std::chrono::steady_clock::time_point taskEndTime;

    while(!m_terminateMPThreads){
        
        bool hasTask = false;
        MPTask threadTask;
       
        m_mpTaskMutex.lock();
        if(!m_mpTaskQueue.empty()){
            taskStartTime = std::chrono::steady_clock::now();
            threadTask = *m_mpTaskQueue.begin();
            m_mpTaskQueue.erase(m_mpTaskQueue.begin());
            rlog("workerMPEdges", LOG_INFO, std::to_string(m_mpTaskQueue.size()) + " Working now on task: " + std::to_string(threadTask.cai) +":"+ std::to_string(threadTask.csi) +" -> "+ std::to_string(threadTask.txi) +":"+ std::to_string(threadTask.tyi) + ":"+ std::to_string(threadTask.tsi));
            hasTask = true;
        }else{
            hasTask = false;
        }

        m_mpTaskMutex.unlock();

        if(hasTask){

            int start_angle = (threadTask.cai - threadTask.mhd + map_size_angle) % map_size_angle;
            int end_angle = (threadTask.cai + threadTask.mhd + map_size_angle) % map_size_angle;
            for (int tsh = start_angle; tsh != end_angle; tsh = (tsh + 1) % map_size_angle) {
            
                //Compute best fit settings set to get from the current to the target location
                dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi, tsh, threadTask.tsi};
                
                bool found_primitive = false;
                
                

                MPNLOptSingle::mpnl_args_t args;
                args.sp = {{0.f,0.f}, api * static_cast<float>(threadTask.cai), vlevels[threadTask.csi] * dynamics::SimpleDynamicsModel::velocity_limit()};
                args.tp = {{(dpc*threadTask.txi), (dpc*threadTask.tyi)},  api * static_cast<float>(tsh), vlevels[threadTask.tsi] * dynamics::SimpleDynamicsModel::velocity_limit()};

                if(args.sp.vel < 0.f || args.tp.vel < 0.f){
                    args.ub = config.get<std::vector<double>>({"mpnl_args", "ubr"});
                    args.lb = config.get<std::vector<double>>({"mpnl_args", "lbr"});
                }else{
                    args.ub = config.get<std::vector<double>>({"mpnl_args", "ubf"});
                    args.lb = config.get<std::vector<double>>({"mpnl_args", "lbf"});
                }

                auto uni_samples = generateUniformSamples(args.lb, args.ub, uni_sample_count);
                auto rand_samples = generateRandomSamples(args.lb, args.ub, rand_sample_count);
                std::vector<std::vector<double>> samples;
                samples.insert(samples.end(), uni_samples.begin(), uni_samples.end());
                samples.insert(samples.end(), rand_samples.begin(), rand_samples.end());

                // rlog("workerMPEdges", LOG_INFO, std::to_string(m_mpTaskQueue.size()) + "Task: " + std::to_string(threadTask.cai) +":"+ std::to_string(threadTask.csi) +" -> "+ std::to_string(threadTask.txi) +":"+ std::to_string(threadTask.tyi) + ":"+ std::to_string(threadTask.tsi) + ":" + std::to_string(samples.size()));
                MPNLOptSingle::mpnl_return res;
                double best_obj_val = 10.0;

                for(auto sample: samples){

                    args.init_guess = sample;
                    
                    MPNLOptSingle nl_stm_opt_isres;
                    nl_stm_opt_isres.prepare(&args, true, false);
                    
                    auto isres_res = nl_stm_opt_isres.optimize(); 

                    if(isres_res.retcode > 0 && isres_res.retcode < 4){
                        found_primitive = true;
                        if(isres_res.objf_val < best_obj_val){
                            std::cout << "isres best obj " << std::to_string(res.objf_val) << std::endl;
                            best_obj_val = isres_res.objf_val;
                            res = isres_res;
                        }
                    }

                    MPNLOptSingle nl_stm_opt_direct;
                    nl_stm_opt_direct.prepare(&args, false, false, nlopt::GN_DIRECT);
                    
                    auto direct_res = nl_stm_opt_direct.optimize(); 

                    if(direct_res.retcode > 0 && direct_res.retcode < 4){
                        found_primitive = true;
                        if(direct_res.objf_val < best_obj_val){
                            std::cout << "direct best obj " << std::to_string(res.objf_val) << " : " << res.ceval << std::endl;
                            best_obj_val = direct_res.objf_val;
                            res = direct_res;
                        }
                    }

                    MPNLOptSingle nl_stm_opt_crs2;
                    nl_stm_opt_crs2.prepare(&args, false, false, nlopt::GN_CRS2_LM);
                    
                    auto crs2_res = nl_stm_opt_crs2.optimize(); 

                    if(crs2_res.retcode > 0 && crs2_res.retcode < 4){
                        found_primitive = true;
                        if(crs2_res.objf_val < best_obj_val){
                            std::cout << "crs2 best obj " << std::to_string(res.objf_val) << std::endl;
                            best_obj_val = crs2_res.objf_val;
                            res = crs2_res;
                        }
                    }
                }

                if(!found_primitive){
                    continue;
                }

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

                // Q1 MP
                dynamics::data::Pose2DWithError epose;
                epose = pr;
                epose = target_pose_by_index;

                epose.s_a = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_st_a];
                epose.s_a_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_st_a];
                double v_end_1 = args.sp.vel + res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc] * (timestep_ms / 1000.f);
                double v_end_2 = v_end_1 + res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc] * (timestep_ms / 1000.f);
                epose.s_acc = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc];
                epose.s_acc_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc];
                epose.is_acc_based = true;

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
                epose_q2.s_acc = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc];
                epose_q2.s_acc_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc];
                epose_q2.is_acc_based = true;

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
                epose_q3.s_acc = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc];
                epose_q3.s_acc_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc];
                epose_q3.is_acc_based = true;
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
                epose_q4.s_acc = res.result[MPNLOptSingle::mpnl_obj_pv_map::c1_acc];
                epose_q4.s_acc_2 = res.result[MPNLOptSingle::mpnl_obj_pv_map::c2_acc];
                epose_q4.is_acc_based = true;
                MotionPrimitive mp_q4 = {epose_q4, pi_q4};

                m_mpTaskMutex.lock();
                m_mpmap[threadTask.cai                       ][threadTask.csi].push_back(mp_q1);
                m_mpmap[threadTask.cai +   (map_size_angle/4)][threadTask.csi].push_back(mp_q2);
                m_mpmap[threadTask.cai +   (map_size_angle/2)][threadTask.csi].push_back(mp_q3);
                m_mpmap[threadTask.cai + 3*(map_size_angle/4)][threadTask.csi].push_back(mp_q4);  
                m_mpTaskMutex.unlock();
                
            }
    
            taskEndTime = std::chrono::steady_clock::now();
            double taskDuration = std::chrono::duration_cast<std::chrono::milliseconds>(taskEndTime - taskStartTime).count();
            completedTasks++;
            totalTime += taskDuration;

            double avg = totalTime / completedTasks;
            int32_t remainingTasks = m_mpTaskQueue.size();
            double estc = (remainingTasks / worker_count) * avg / 60000; // Convert ms to minutes

           
            rlog("workerMPEdges", LOG_INFO, "Average solve time: " + std::to_string(avg) + "[ms] ESTC: " + std::to_string(estc) + "[m]");
            

        }else{
            return;
        }
        hasTask = false;
    }
}
