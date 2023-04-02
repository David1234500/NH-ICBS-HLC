#ifndef MPNLOpt_HPP
#define MPNLOpt_HPP

#include <MPCompute/MPCompute.hpp>

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Config.hpp>

#include <vector>
#include <map>
#include <mutex>


class MPNLOptStagedMultiStart: public MPCompute{
public:

MPNLOptStagedMultiStart();

std::map<uint32_t, double> m_solvetimes;
void workerThreadMPEdges(uint32_t index) override;

};


#endif