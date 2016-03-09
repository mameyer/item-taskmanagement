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
#include <joint_dispatcher/proxies/Task.hpp>
#include <odometry/proxies/Skid.hpp>
#include <mars/proxies/Task.hpp>
#include <orocos_cpp/Bundle.hpp>
#include <mars/tasks/MarsControl.hpp>
#include <mars/proxies/IMU.hpp>
#include <mars/proxies/RotatingLaserRangeFinder.hpp>
#include <mars/proxies/Joints.hpp>
#include <drive_mode_controller/proxies/Task.hpp>

Init::Init(bool logTasks, bool active) : State("Init"), initialized(false), doLog(logTasks), active(active) {
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

void Init::updateConfig(RTT::TaskContext* task, const orocos_cpp::Configuration& conf)
{
    confHelper.applyConfig(task, conf);
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
    
    std::cout << "InitSimulation::executeFunction.." << std::endl;
    mars::proxies::Task *marsSimulationTask = new mars::proxies::Task("mars_simulation", false);
    confHelper.applyConfig(marsSimulationTask, "default");

    std::string dataDir = orocos_cpp::Bundle::getInstance().getDataDirectory();
    const char* autoproj_root = std::getenv("AUTOPROJ_CURRENT_ROOT");
    if(autoproj_root == NULL) {
        std::cerr << "Env AUTOPROJ_CURRENT_ROOT not available, return" << std::endl;
        return;
    }

    std::string autoproj_root_str(autoproj_root);
    try
    {
        marsSimulationTask->configure();
        marsSimulationTask->start();
        marsSimulationTask->loadScene(autoproj_root_str + "/models/robots/eo2/smurf/eo2.smurf");
        marsSimulationTask->loadScene(autoproj_root_str + "/models/terrains/spacebot_cup_building.smurfs");
    } catch(std::runtime_error& e) {
        std::cerr << "marsSimulationTask: " << e.what() << std::endl;
    }

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
    poseProviderTask->pose_samples.connectTo(trajectoryFollowerTask->robot_pose);
    //trajectoryFollowerTask->motion2D.connectTo(motionCommandConverterTask->motion_command_in);
    trajectoryFollowerTask->motion_command.connectTo(motionCommandConverterTask->motion_command_in);
    velodyneSlamTask->pose_provider_update.connectTo(poseProviderTask->pose_provider_update);
    
    if (active) {
	velodyneSlamTask->envire_map.connectTo(traversabilityTask->mls_map);
	poseProviderTask->pose_samples.connectTo(plannerTask->start_pose_samples);
	plannerTask->trajectory.connectTo(trajectoryFollowerTask->trajectory);
	traversabilityTask->traversability_map.connectTo(plannerTask->traversability_map);
    }
    
    driveModeControllerTask->actuator_mov_cmds_out.connectTo(jointsTask->command);
    //motionCommandConverterTask->motion2D.connectTo(driveModeControllerTask->motion2D);
    motionCommandConverterTask->motion_command.connectTo(driveModeControllerTask->motion_command);
    velodyneTask->pointcloud.connectTo(velodyneSlamTask->simulated_pointcloud);
    perfectOdometryTask->pose_samples.connectTo(poseProviderTask->odometry_samples);
    perfectOdometryTask->pose_samples.connectTo(velodyneSlamTask->odometry_samples);
    
    return true;
}

bool Init::disconnect()
{
    trajectoryFollowerTask->robot_pose.disconnect();
    motionCommandConverterTask->motion_command_in.disconnect();
    poseProviderTask->pose_provider_update.disconnect();
    
    if (active) {
	traversabilityTask->mls_map.disconnect();
	plannerTask->start_pose_samples.disconnect();
	trajectoryFollowerTask->trajectory.disconnect();
	plannerTask->traversability_map.disconnect();
    }
    
    jointsTask->command.disconnect();
    driveModeControllerTask->motion_command.disconnect();
    velodyneSlamTask->simulated_pointcloud.disconnect();
    poseProviderTask->odometry_samples.disconnect();
    velodyneSlamTask->odometry_samples.disconnect();

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
    
    if (active) {
	plannerTask = new motion_planning_libraries::proxies::Task("planner",false);
	registerWithConfig(plannerTask);
	
	traversabilityTask = new traversability::proxies::Simple("traversability");
	registerWithConfig(traversabilityTask);
    }

    velodyneTask = new mars::proxies::RotatingLaserRangeFinder("velodyne", false);
    registerWithConfig(velodyneTask, "default");

    driveModeControllerTask = new drive_mode_controller::proxies::Task("drive_mode_controller");
    registerWithConfig(driveModeControllerTask, "default");

    perfectOdometryTask = new mars::proxies::IMU("perfect_odometry");
    registerWithConfig(perfectOdometryTask, "default");

    jointsTask = new mars::proxies::Joints("joints");
    registerWithConfig(jointsTask, "default");

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

bool Init::stop()
{
    for(TaskWithConfig &t: allTasks)
    {
        if(!t.task->stop())
        {
            throw std::runtime_error("Init::Failed to stop task " + t.task->getName());
        }
        std::cout << "Init::Stopped " << t.task->getName() << std::endl;
    }
    
    return true;
}

bool Init::restart()
{
    disconnect();
    mars::proxies::Task *marsSimulationTask = new mars::proxies::Task("mars_simulation", false);
    marsSimulationTask->stop();
    
    mars::Positions pos;
    pos.nodename = "root";
    pos.posx = 0.;
    pos.posy = 0.;
    pos.posz = 0.05;
    pos.rotx = 0;
    pos.roty = 0;
    pos.rotz = 0;
    
    marsSimulationTask->setPosition(pos);
    marsSimulationTask->start();
    usleep(50000);
    
    /*velodyneSlamTask = new graph_slam::proxies::VelodyneSLAM("slam");
    velodyneSlamTask->stop();
    confHelper.applyConfig(velodyneSlamTask, "default");
    
    if(!trHelper->configureTransformer(velodyneSlamTask))
    {
	throw std::runtime_error("Init::Failed to configure transformer for task " + velodyneSlamTask->getName());
    }
    
    velodyneSlamTask->configure();*/
    connect();
    //velodyneSlamTask->start();
    usleep(100000);
}