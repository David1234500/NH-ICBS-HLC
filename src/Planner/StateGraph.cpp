#include <Planner/StateGraph.hpp>
#include <iostream>

StateGraph::StateGraph(){}

void StateGraph::computeEdges(){
    
    //For each physical location and angle option in the grid
    for(uint32_t i = 1; i < map_size_x - 1; i ++){
        std::cout << " col  " << i << std::endl;
        for(uint32_t j = 1; j < map_size_y - 1; j ++){
            for(uint32_t k = 0; k < map_size_angle; k ++){
              
                // For each other cell and angle option close by
                for(int8_t x = -1; x < 2; x ++){
                    for(int8_t y = -1; y < 2; y ++){
                        for(uint32_t a = 0; a < map_size_angle; a ++){
                            
                            auto grid_pose = gridToPosition(i,j,k);
                            auto target_pose = gridToPosition(i+x,j+y,k+a);

                            float angle = 0;
                            float speed = 0;
                            if(dynamics::SimpleDynamicsModel::computeBestFit(grid_pose, target_pose, angle, speed, 100)){
                                Edge edge;
                                m_adjacency_matrix[i][j][k][i+x][j+y][k+a] = edge;
                            }
                        }
                    }
                }
            
            }
        }
    }
}

dynamics::data::Pose2D StateGraph::gridToPosition(uint32_t x, uint32_t y, uint32_t angle){
dynamics::data::Pose2D pose;

pose.h = api * (float)angle;
pose.vel = 0;
pose.pos = {xpc * (float)x,  ypc * (float)y};

return pose;
}
