#ifndef MPBruteforce_HPP
#define MPBruteforce_HPP

#include <MPCompute/MPCompute.hpp>
#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Config.hpp>

#include <vector>
#include <map>
#include <mutex>


class MPBruteforce: public MPCompute{
public:

MPBruteforce();

void workerThreadMPEdges(uint32_t index) override;

};


#endif