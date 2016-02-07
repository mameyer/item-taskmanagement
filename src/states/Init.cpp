#include "Init.hpp"
#include <rtt/transports/corba/TaskContextServer.hpp>

#include <orocos_cpp/ConfigurationHelper.hpp>
#include <orocos_cpp/Bundle.hpp>

#include <trajectory_follower/proxies/Task.hpp>
#include <motion_planning_libraries/proxies/Task.hpp>
#include <graph_slam/proxies/VelodyneSLAM.hpp>
#include <traversability/proxies/Simple.hpp>
#include <trajectory_follower/proxies/TurnVelocityToSteerAngleTask.hpp>
#include <localization/proxies/VelodyneInMLS.hpp>
//#include <localization/proxies/PoseProvider.hpp>
#include <joint_dispatcher/proxies/Task.hpp>
#include <odometry/proxies/Skid.hpp>

Init::Init(bool logTasks) : State("Init"), initialized(false), doLog(logTasks) {
    trHelper = new orocos_cpp::TransformerHelper(robot);
    std::cout << "Init::Init.." << std::endl;
}

bool Init::isInitialized()
{
    return initialized;
}

void Init::enter(const state_machine::State* lastState)
{
    msg << "Entering state Init" << std::endl;
}

void Init::updateConfig(RTT::TaskContext* task, const std::vector< std::string >& configs)
{
    for(TaskWithConfig &t: allTasks)
    {
        if(t.task == task)
        {
            t.config = configs;
            return;
        }
    }

    throw std::runtime_error("Init::updateConfig : No task " + task->getName() + " registered");
}

void Init::updateConfig(RTT::TaskContext* task, const std::string& config, const std::string& config2)
{
    std::vector< std::string > configs;
    configs.push_back(config);
    configs.push_back(config2);

    updateConfig(task, configs);
}

void Init::updateConfig(RTT::TaskContext* task, const std::string& config, const std::string& config2, const std::string& config3)
{
    std::vector< std::string > configs;
    configs.push_back(config);
    configs.push_back(config2);
    configs.push_back(config3);

    updateConfig(task, configs);
}

void Init::registerWithConfig(RTT::TaskContext* task, const std::vector< std::string >& configs)
{
    TaskWithConfig t;
    t.task = task;
    t.config = configs;

    allTasks.push_back(t);
}

void Init::registerWithConfig(RTT::TaskContext* task, const std::string& config)
{
    std::vector< std::string > configs;
    configs.push_back(config);

    registerWithConfig(task, configs);
}

void Init::registerWithConfig(RTT::TaskContext* task, const std::string& config, const std::string& config2)
{
    std::vector< std::string > configs;
    configs.push_back(config);
    configs.push_back(config2);

    registerWithConfig(task, configs);
}

void Init::registerWithConfig(RTT::TaskContext* task, const std::string& config, const std::string& config2, const std::string& config3)
{
    std::vector< std::string > configs;
    configs.push_back(config);
    configs.push_back(config2);
    configs.push_back(config3);

    registerWithConfig(task, configs);
}

void Init::executeFunction()
{
    orocos_cpp::LoggingHelper lHelper;

    //lHelper.logTasks(conf.loggingEnabledTaskMap);

    std::cout << "Init::executeFunction.." << std::endl;

    setup();
    configure();
    connect();
    start();

    if(doLog)
    {
        std::cout << "Activating logger" << std::endl;
        std::vector<std::string> excludes;

        lHelper.logTasks(excludes);
    }

    initialized = true;
}

bool Init::connect()
{
    //eo2DispatcherTask->synchronize();
    
    //velodyneSlamTask->pose_samples.connectTo(trajectoryFollowerTask->robot_pose);
    poseProviderTask->pose_samples.connectTo(trajectoryFollowerTask->robot_pose);
    trajectoryFollowerTask->motion_command.connectTo(motionCommandConverterTask->motion_command_in);
    velodyneSlamTask->pose_provider_update.connectTo(poseProviderTask->pose_provider_update);
    
    //auto dispatcherMotionStatusOut = new OutputProxyPort<base::samples::Joints>(eo2DispatcherTask->getPort("motion_status"));
    //dispatcherMotionStatusOut->connectTo(odometryTask->actuator_samples, RTT::ConnPolicy::buffer(200));
    
    //odometryTask->odometry_samples.connectTo(velodyneSlamTask->odometry_samples, RTT::ConnPolicy::buffer(200));
    //odometryTask->odometry_samples.connectTo(poseProviderTask->odometry_samples);
    
    return true;
}


bool Init::setup()
{
    trajectoryFollowerTask = new trajectory_follower::proxies::Task("follower");
    registerWithConfig(trajectoryFollowerTask);

    velodyneSlamTask = new graph_slam::proxies::VelodyneSLAM("slam");
    registerWithConfig(velodyneSlamTask, "default");

    motionCommandConverterTask = new trajectory_follower::proxies::TurnVelocityToSteerAngleTask("motion_command_converter");
    registerWithConfig(motionCommandConverterTask);
    
    poseProviderTask = new localization::proxies::PoseProvider("pose_provider");
    registerWithConfig(poseProviderTask, "default");
    
    //eo2DispatcherTask = new joint_dispatcher::proxies::Task("eo2_dispatcher");
    //registerWithConfig(eo2DispatcherTask, "default", "odometry");
    
    //odometryTask = new odometry::proxies::Skid("odometry");
    //registerWithConfig(odometryTask);

    return true;
}

bool Init::configure()
{
    for(TaskWithConfig &t: allTasks)
    {
        confHelper.applyConfig(t.task, t.config);

        if(!trHelper->configureTransformer(t.task))
        {
            throw std::runtime_error("Init::Failed to configure transformer for task " + t.task->getName());
        }

        std::cout << "Init::Configuring " << t.task->getName() << std::endl;
        if(!t.task->configure())
        {
            std::string config = "[";
            for(auto conf: t.config)
            {
                config += conf + ", ";
            }
            config += "]";
            throw std::runtime_error("Init::Failed to configure task " + t.task->getName() + " with configuration " + config);
        }

        std::cout << "Init::Configured " << t.task->getName() << std::endl;

    }
    return true;
}

bool Init::start()
{
    for(TaskWithConfig &t: allTasks)
    {
        if(!t.task->start())
        {
            throw std::runtime_error("Init::Failed to start task " + t.task->getName());
        }
        std::cout << "Init::Started " << t.task->getName() << std::endl;

    }

    return true;
}

void Init::exit()
{
    msg << "Leaving init state ...\n";
};

bool Init::restart()
{
    for(TaskWithConfig &t: allTasks)
    {
        if(!t.task->stop())
        {
            throw std::runtime_error("Init::Failed to start task " + t.task->getName());
        }
        std::cout << "Init::Stopped " << t.task->getName() << std::endl;
	
	if(!t.task->start())
        {
            throw std::runtime_error("Init::Failed to start task " + t.task->getName());
        }
        std::cout << "Init::Started " << t.task->getName() << std::endl;

    }

    return true;
}