#include "states/DummyState.hpp"
#include "states/GenerateMap.hpp"

#include "CommonStartup.hpp"
#include "Helper.hpp"

#include <orocos_cpp/Bundle.hpp>
#include <state_machine/StateMachine.hpp>
#include <motion_planning_libraries/proxies/Task.hpp>

#include <base/samples/RigidBodyState.hpp>
#include <base/Angle.hpp>
#include <trajectory_follower/proxies/Task.hpp>
#include <base/Trajectory.hpp>

class DriveLog: public state_machine::State
{
public:
    DriveLog(State* success, State* failue);
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
    inline void setLogDir(const std::string &logDir) {
        this->logDir = logDir;
    };

private:
    bool wasFollowing;
    std::map< std::string, std::vector< base::Trajectory > > trajectories;
    state_machine::Config *config;
    trajectory_follower::proxies::Task *trajectoryFollower;
    RTT::InputPort< int > *trajectoryFollowerStateReader;
    RTT::OutputPort< std::vector< base::Trajectory > > *trajectoryWriter;
    localization::proxies::PoseProvider *poseProviderTask;
    RTT::InputPort< base::samples::RigidBodyState > *poseReader;
    std::string logDir;
    std::map< std::string, std::vector< base::Trajectory > >::iterator trajectoriesIter;
};

DriveLog::DriveLog(state_machine::State* success, state_machine::State* failure)
    : State("DriveLog", success, failure)
{
    trajectoryFollower = new trajectory_follower::proxies::Task("follower");
    poseProviderTask = new localization::proxies::PoseProvider("pose_provider");
    trajectoryFollowerStateReader = &(trajectoryFollower->state.getReader());
    trajectoryWriter = &(trajectoryFollower->trajectory.getWriter());
    poseReader = &(poseProviderTask->pose_samples.getReader());
    wasFollowing = false;

    const char* autoproj_root = std::getenv("AUTOPROJ_CURRENT_ROOT");
    if(autoproj_root == NULL) {
        std::runtime_error("Env AUTOPROJ_CURRENT_ROOT not available, return");
    }

    std::string autoproj_root_str(autoproj_root);
    logDir = autoproj_root_str + std::string("/bundles/eo2/logs");
    
    std::cout << "Helper::readTrajectories: " << logDir << std::endl;
    Helper::readTrajectories(trajectories, logDir);
}

void DriveLog::enter(const state_machine::State* lastState)
{
    if (trajectories.empty())
        throw std::runtime_error("DriveLog::enter no trajectories set..");

    trajectoriesIter = trajectories.begin();
    trajectoryWriter->write(trajectoriesIter->second);
    
    std::map< std::string, double > configValues;
    configValues["l1"] = 2.;
    Helper::updateConfig(configValues, "noOrientationControllerConfig");
    
    configValues.clear();
    configValues["K0"] = 2.;
    Helper::updateConfig(configValues, "chainedControllerConfig");
}

void DriveLog::executeFunction()
{
    base::samples::RigidBodyState currentPose;
    if (poseReader->readNewest(currentPose) == RTT::NewData) {
        if (currentPose.getPitch() < base::Angle::fromDeg(-45).getRad()
                || currentPose.getPitch() > base::Angle::fromDeg(45).getRad()
                || currentPose.getRoll() < base::Angle::fromDeg(-45).getRad()
                || currentPose.getRoll() > base::Angle::fromDeg(45).getRad()) {
            fail();
        }
    }

    int trajectoryFollowerState;
    if (trajectoryFollowerStateReader->readNewest(trajectoryFollowerState) == RTT::NewData) {
	if (trajectoryFollowerState == trajectory_follower::Task_STATES::Task_STABILITY_FAILED)
	    fail();
	
        if (!wasFollowing && trajectoryFollowerState == trajectory_follower::Task_STATES::Task_FOLLOWING_TRAJECTORY)
            wasFollowing = true;

        if (wasFollowing && trajectoryFollowerState == trajectory_follower::Task_STATES::Task_FINISHED_TRAJECTORIES) {
            wasFollowing = false;
            finish();
        }
    }
}

void DriveLog::exit()
{

}

int main(int argc, char** argv)
{
    bool gotLogDir = false;

    int logStart = 0;
    for (int i = 0; i<argc; i++) {
        if(strcmp(argv[i], "--logpath") == 0) {
            logStart = i + 1;
            gotLogDir = true;
            break;
        }
    }

    if (gotLogDir) {
	if (logStart > argc) {
	    std::cout << "Usage ./Cmd --logpath <path>" << std::endl;
	    return -1;
	}
    }

    CommonStartup csu;
    csu.start(argc, argv);
    std::cout << "csu.start.." << std::endl;

    DummyState *dummy = new DummyState();
    DriveLog *drive = new DriveLog(dummy, dummy);
    GenerateMap* gm = new GenerateMap(drive);

    if (gotLogDir)
	drive->setLogDir(std::string(argv[logStart]));

    csu.init->addEdge("Initialized", gm, [&] () {
        return csu.init->isInitialized();
    });

    std::cout << "csu.runLoop.." << std::endl;
    csu.runLoop([&] () {
        if (&csu.stateMachine->getCurrentState() == dummy) {
            csu.init->restart();
            csu.stateMachine->start(drive);
        }
    });

    return 0;
}