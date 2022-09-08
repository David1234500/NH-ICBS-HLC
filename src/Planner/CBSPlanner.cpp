#include <Planner/CBSPlanner.hpp>


void CBSPlanner::insertByOrder(IndexNode node, std::vector<IndexNode>& openSet, std::map<IndexNode,float>& fScore){

    // Implement binary search here

}

void CBSPlanner::astar(IndexNode start, IndexNode target, StateGraph& graph){
    std::vector<IndexNode> openSet;
    std::map<IndexNode,IndexNode> cameFrom;

    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known
    std::map<IndexNode,float> m_gScore;

    //For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how cheap a path could be from start to finish if it goes through n.
    std::map<IndexNode,float> fScore;

    while(!openSet.empty()){
        auto current = openSet.front();
       
        if(current == target){
            // success
            return;
        }

        openSet.erase(openSet.begin());
        for(auto neighbors: graph.m_proxyEdgeList[current.a]){
            if(){
                
            }
            float tentative_score = m_gScore[]]



        }
    }
}
