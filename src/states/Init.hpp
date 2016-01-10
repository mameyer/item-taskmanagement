#ifndef INIT_H
#define INIT_H

#include <state_machine/State.hpp>
#include <state_machine/Config.hpp>
#include <orocos_cpp/ConfigurationHelper.hpp>
#include <orocos_cpp/TransformerHelper.hpp>
#include <orocos_cpp/LoggingHelper.hpp>
#include "../Eo2Robot.hpp"
#include "Forward.hpp"

class TaskWithConfig
{
public:
    RTT::TaskContext *task;
    std::vector<std::string> config;
};

class Init : public state_machine::State {
public:
    Init(bool logTasks);
    void enter(const State *lastState);
    void exit();
    virtual void executeFunction();
    bool isInitialized();

public:
    void updateConfig(RTT::TaskContext *task, const std::vector<std::string> &configs);
    void updateConfig(RTT::TaskContext *task, const std::string &config, const std::string &config2);   
    void updateConfig(RTT::TaskContext *task, const std::string& config, const std::string& config2, const std::string& config3);
    
    std::vector<TaskWithConfig> getAllTasks() {
        return allTasks;
    }
 
protected:
    trajectory_follower::proxies::Task *trajectoryFollowerTask;
    motion_planning_libraries::proxies::Task *plannerTask;
    graph_slam::proxies::VelodyneSLAM *velodyneSlamTask;
    traversability::proxies::Simple *traversabilityTask;
    trajectory_follower::proxies::TurnVelocityToSteerAngleTask *motionCommandConverterTask;
    localization::proxies::VelodyneInMLS *localizationTask;
    localization::proxies::PoseProvider *poseProviderTask;
    
    virtual bool setup();
    bool configure();
    virtual bool connect();
    bool start();
    
    void registerWithConfig(RTT::TaskContext *task, const std::vector<std::string> &configs);
    void registerWithConfig(RTT::TaskContext *task, const std::string &config = "default");
    void registerWithConfig(RTT::TaskContext *task, const std::string &config, const std::string &config2);
    void registerWithConfig(RTT::TaskContext *task, const std::string &config, const std::string &config2, const std::string &config3);

    std::vector<TaskWithConfig> allTasks;
    
    bool initialized;
    orocos_cpp::ConfigurationHelper confHelper;
    Eo2Robot robot;
    orocos_cpp::TransformerHelper* trHelper;
    bool doLog;    
};

#endif // INIT_H