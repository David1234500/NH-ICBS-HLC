#include <MPCompute/MPBruteforce.hpp>
#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>

MPBruteforce::MPBruteforce(){

}

void MPBruteforce::workerThreadMPEdges(uint32_t index){
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
            
            dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * static_cast<float>(threadTask.cai), m_speedsFactor[threadTask.csi] * dynamics::SimpleDynamicsModel::velocity_limit()};
           
                for(int32_t a = 0; a < map_size_angle; a ++){
                
                    //Compute best fit settings set to get from the current to the target location
                    dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi, a, threadTask.tsi};
                    dynamics::data::Pose2D target_pose = {{-(m_mpMapCarOffset * xpc) + (xpc*threadTask.txi), -(m_mpMapCarOffset * ypc) + (ypc*threadTask.tyi)},  api * static_cast<float>(a), m_speedsFactor[threadTask.tsi] * dynamics::SimpleDynamicsModel::velocity_limit()};

                    double speed_delta = state_change_fit_allowed_speed_difference;
                    auto epose = dynamics::SimpleDynamicsModel::forceBestFit(veh_pose, target_pose_by_index, target_pose, threadTask.tstep, speed_delta);
                    
                    //Discard if error greater than half of the angular resolution 
                    if(epose.a_error > state_change_fit_quality_angle){
                        continue;
                    }
                    if(epose.p_error > state_change_fit_quality_position){
                        continue;
                    }

                    // Found a link to the neighbor, add an edge for this one
                    MotionPrimitive nt_edge = {epose, target_pose_by_index};
                    
                    rlog("workerMPEdges", LOG_INFO,"Added edge to " + std::to_string(epose.pos[0]) +":"+ std::to_string(epose.pos[1]) 
                    + " -> s" + std::to_string(epose.s_a) +"_"+ std::to_string(epose.s_v) + "e" + std::to_string(epose.p_error) +  "p" + std::to_string(a) +"_"+ std::to_string(threadTask.tsi));
                    
                    // add new edge to the edgelist
                    m_mpTaskMutex.lock();
                    m_mpmap[threadTask.cai][threadTask.csi].push_back(nt_edge);  
                    m_mpTaskMutex.unlock();
                
            }
    
        }else{
            return;
        }
        hasTask = false;
    }
}


