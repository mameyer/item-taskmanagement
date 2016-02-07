#ifndef HELPER_HPP
#define HELPER_HPP

#include <rtt/types/TypekitRepository.hpp>
#include <rtt/types/TypeInfoRepository.hpp>
#include <rtt/types/TypeInfo.hpp>
#include <rtt/typelib/TypelibMarshallerBase.hpp>

#include <base/Trajectory.hpp>
#include <vector>
#include <map>

class Helper {
public:
    static void writeTrajectory(const std::vector< base::Trajectory > &trajectory, const std::string& fullPath);
    static void readTrajectory(std::vector< base::Trajectory > &trajectory, const std::string& fullPath);
    static void readTrajectories(std::map< std::string, std::vector< base::Trajectory > > &trajectories, const std::string& dir);
    static void updateConfig(const std::map< std::string, double > &confVals, const std::string &controllerConfigName);
};

struct FollowerControllerRating {
    double angleError;
    double distError;
    bool reachedEnd;
    unsigned int numMotionCommands;
    double angleErrorSum;
    double distErrorSum;
    bool stabilityFailed;
    std::string path;
};

#endif