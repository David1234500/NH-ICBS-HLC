#include <MPCompute/MPBruteforce.hpp>
#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>

MPBruteforce::MPBruteforce(){

}

void MPBruteforce::workerThreadMPEdges(uint32_t index){
    int32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float xpc = Config::getInstance().get<float>({"disc","xstep"});
    float ypc = Config::getInstance().get<float>({"disc","ystep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});

    float fit_quality_angle = Config::getInstance().get<int>({"MPComputeBruteForce","fit_quality_angle"});
    float fit_quality_position = Config::getInstance().get<int>({"MPComputeBruteForce","fit_quality_position"});
    float fit_allowed_speed_difference = Config::getInstance().get<int>({"MPComputeBruteForce","fit_allowed_speed_difference"});

    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().get<int32_t>({"map","x_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});


    while(!m_terminateMPThreads){
        
        bool hasTask = false;
        MPTask threadTask;

        m_mpTaskMutex.lock();
        if(!m_mpTaskQueue.empty()){
            threadTask = *m_mpTaskQueue.begin();
            m_mpTaskQueue.erase(m_mpTaskQueue.begin());
            
            if(((threadTask.txi + threadTask.tyi) % 10) == 0){
                rlog("workerMPEdges", LOG_INFO," Working now on task: " + std::to_string(threadTask.txi) +":"+ std::to_string(threadTask.tyi) + ":"+ std::to_string(threadTask.cai) +":"+ std::to_string(threadTask.csi));
            }
            
            hasTask = true;
        }else{
            hasTask = false;
        }
        m_mpTaskMutex.unlock();

        if(hasTask){
            
            dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * static_cast<float>(threadTask.cai), vlevels[threadTask.csi] * dynamics::SimpleDynamicsModel::velocity_limit()};
           
                for(int32_t a = 0; a < map_size_angle; a ++){
                
                    //Compute best fit settings set to get from the current to the target location
                    dynamics::data::PoseByIndex tp_bi = {threadTask.txi,threadTask.tyi, a, threadTask.tsi};
                    dynamics::data::Pose2D target_pose = {{ (xpc*threadTask.txi), (ypc*threadTask.tyi)},  api * static_cast<float>(a), vlevels[threadTask.tsi] * dynamics::SimpleDynamicsModel::velocity_limit()};

                    double speed_delta = fit_allowed_speed_difference;
                    auto epose = dynamics::SimpleDynamicsModel::forceBestFit(veh_pose, tp_bi, target_pose, threadTask.tstep, speed_delta);
                    
                    //Discard if error greater than half of the angular resolution 
                    if(epose.a_error > fit_quality_angle){
                        continue;
                    }
                    if(epose.p_error > fit_quality_position){
                        continue;
                    }
                    
                    rlog("workerMPEdges", LOG_INFO,"Added edge to " + std::to_string(epose.pos[0]) +":"+ std::to_string(epose.pos[1]) 
                    + " -> s" + std::to_string(epose.s_a) +"_"+ std::to_string(epose.s_v) + "e" + std::to_string(epose.p_error) +  "p" + std::to_string(a) +"_"+ std::to_string(threadTask.tsi));

                    addSymetricMPs(epose.s_a,epose.s_a_2,epose.s_v, epose.s_v_2, tp_bi, a, threadTask.cai, threadTask.csi, threadTask.tsi);
            }
    
        }else{
            return;
        }
        hasTask = false;
    }
}


