#include "InitSimulation.hpp"
#include <mars/proxies/Task.hpp>
#include <orocos_cpp/Bundle.hpp>

#include <mars/proxies/IMU.hpp>
#include <mars/proxies/RotatingLaserRangeFinder.hpp>
#include <mars/proxies/Joints.hpp>
#include <drive_mode_controller/proxies/Task.hpp>
#include <graph_slam/proxies/VelodyneSLAM.hpp>
#include <trajectory_follower/proxies/Task.hpp>
#include <traversability/proxies/Simple.hpp>
#include <motion_planning_libraries/proxies/Task.hpp>
#include <trajectory_follower/proxies/TurnVelocityToSteerAngleTask.hpp>

InitSimulation::InitSimulation(bool logTasks): Init(logTasks)
{
}

bool InitSimulation::setup()
{
    Init::setup();

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

bool InitSimulation::connect()
{
    Init::connect();

    driveModeControllerTask->actuator_mov_cmds_out.connectTo(jointsTask->command);
    motionCommandConverterTask->motion_command.connectTo(driveModeControllerTask->motion_command);
    velodyneTask->pointcloud.connectTo(velodyneSlamTask->simulated_pointcloud, RTT::ConnPolicy::buffer(50));
    perfectOdometryTask->pose_samples.connectTo(velodyneSlamTask->odometry_samples);

    return true;
}

void InitSimulation::executeFunction()
{
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

    Init::executeFunction();
}