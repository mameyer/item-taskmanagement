#include "states/DummyState.hpp"
#include "states/GenerateMap.hpp"

#include "CommonStartup.hpp"

#include <orocos_cpp/Bundle.hpp>
#include <state_machine/StateMachine.hpp>
#include <motion_planning_libraries/proxies/Task.hpp>

#include <base/samples/RigidBodyState.hpp>
#include <trajectory_follower/proxies/Task.hpp>

class Drive: public state_machine::State
{
public:
    Drive(State* success, State* failue);
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
    inline void setGoals(std::vector<base::samples::RigidBodyState> &goals) {
        this->goals = goals;
    };

private:
    bool wasFollowing;
    std::vector<base::samples::RigidBodyState> goals;
    state_machine::Config *config;
    motion_planning_libraries::proxies::Task *globalPlanner;
    trajectory_follower::proxies::Task *trajectoryFollower;
    RTT::OutputPort< base::samples::RigidBodyState > *goalPoseWriter;
    RTT::InputPort< int > *trajectoryFollowerStateReader;
    RTT::InputPort< int > *globalPlannerStateReader;
    GenerateMap genMap;
};

Drive::Drive(state_machine::State* success, state_machine::State* failure)
    : State("Drive", success, failure)
    , genMap(this)
{
    registerSubState(&genMap);
    globalPlanner = new motion_planning_libraries::proxies::Task("planner", false);
    trajectoryFollower = new trajectory_follower::proxies::Task("follower");
    goalPoseWriter = &(globalPlanner->goal_pose_samples.getWriter());
    trajectoryFollowerStateReader = &(trajectoryFollower->state.getReader());
    globalPlannerStateReader = &(globalPlanner->state.getReader());
    wasFollowing = false;
}

void Drive::enter(const state_machine::State* lastState)
{
    if (goals.empty())
	throw std::runtime_error("Drive::enter no goal set..");
    
    goalPoseWriter->write(goals.front());
    goals.erase(goals.begin());
}

void Drive::executeFunction()
{
    int globalPlannerState;
    if (globalPlannerStateReader->read(globalPlannerState) == RTT::NewData) {
	
	if (globalPlannerState == motion_planning_libraries::Task_STATES::Task_MISSING_START_GOAL_TRAV
	    || globalPlannerState == motion_planning_libraries::Task_STATES::Task_MISSING_TRAV)
	    executeSubState(genMap);
    }
    
    int trajectoryFollowerState;
    if (trajectoryFollowerStateReader->readNewest(trajectoryFollowerState) == RTT::NewData) {
        if (!wasFollowing && trajectoryFollowerState == trajectory_follower::Task_STATES::Task_FOLLOWING_TRAJECTORY)
            wasFollowing = true;
	
	if (wasFollowing && trajectoryFollowerState == trajectory_follower::Task_STATES::Task_FINISHED_TRAJECTORIES) {
	    wasFollowing = false;
	    
	    if (goals.empty()) {
		finish();
	    } else {
		goalPoseWriter->write(goals.front());
		goals.erase(goals.begin());
	    }
	}
    }
}

void Drive::exit()
{

}

int main(int argc, char** argv)
{
    bool gotPos = false;

    int posStart = 0;
    for (int i = 0; i<argc; i++) {
        if(strcmp(argv[i], "--pos") == 0) {
            posStart = i + 1;
            gotPos = true;
            break;
        }
    }

    if(!gotPos || ( posStart + 3 > argc))
    {
        std::cout << "Usage ./Cmd --pos <x> <y> <thetha>" << std::endl;
        return -1;
    }

    base::samples::RigidBodyState goalPose;
    goalPose.position.x() = boost::lexical_cast<double>(argv[posStart + 0]);
    goalPose.position.y() = boost::lexical_cast<double>(argv[posStart + 1]);
    goalPose.position.z() = 0.;
    double theta = boost::lexical_cast<double>(argv[posStart + 2]);
    goalPose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));

    CommonStartup csu;
    csu.start(argc, argv);
    std::cout << "csu.start.." << std::endl;

    DummyState *dummy = new DummyState();
    Drive *drive = new Drive(dummy, dummy);
    GenerateMap* gm = new GenerateMap(drive);

    std::vector< base::samples::RigidBodyState > goals;
    goals.push_back(goalPose);
    base::samples::RigidBodyState home;
    home.position = Eigen::Vector3d(0., 0., 0.);
    home.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ()));
    goals.push_back(home);
    drive->setGoals(goals);

    csu.init->addEdge("Initialized", gm, [&] () {
        return csu.init->isInitialized();
    });

    std::cout << "csu.runLoop.." << std::endl;
    csu.runLoop();

    return 0;
}
