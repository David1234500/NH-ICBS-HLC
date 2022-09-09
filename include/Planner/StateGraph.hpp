#ifndef STATEG_GRAPH_HPP
#define STATEG_GRAPH_HPP

#include <Config.hpp>
#include <Planner/ProxyGraph.hpp>

struct StateNode{
    uint64_t uiid;
};

class StateGraph{

public:
StateGraph(){
    
    uint64_t index = 0;
    for(uint32_t x = 0; x < map_size_x; x ++){
        for(uint32_t y = 0; y < map_size_y; y ++){
            for(uint32_t a = 0; a < map_size_angle; a ++){
                for(uint32_t s = 0; s < map_size_speed; s ++){
                    m_stateMap[x][y][a][s].uiid = index;
                    index += 1;
                }
            }
        }
    }

}

StateNode m_stateMap[map_size_x][map_size_y][map_size_angle][map_size_speed];

inline StateNode operator [](dynamics::data::PoseByIndex global_index){
    return m_stateMap[global_index.x][global_index.y][global_index.a][global_index.s];
}

};


#endif