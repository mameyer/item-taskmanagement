#include "states/DummyState.hpp"
#include "states/GenerateMap.hpp"

#include "CommonStartup.hpp"

#include <orocos_cpp/Bundle.hpp>
#include <state_machine/StateMachine.hpp>
#include <motion_planning_libraries/proxies/Task.hpp>

#include <base/samples/RigidBodyState.hpp>

class Drive: public state_machine::State
{
public:
    Drive(State* success, State* failue);
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
    inline void setGoal(base::samples::RigidBodyState &goal) { this->goal = goal; };

private:
    base::samples::RigidBodyState goal;
    state_machine::Config *config;
    motion_planning_libraries::proxies::Task *globalPlanner;
    RTT::OutputPort< base::samples::RigidBodyState > *goalPoseWriter;
};

Drive::Drive(state_machine::State* success, state_machine::State* failure)
    : State("Drive", success, failure)
{
    globalPlanner = new motion_planning_libraries::proxies::Task("global_planner", false);
    goalPoseWriter = &(globalPlanner->goal_pose_samples.getWriter());
}

void Drive::enter(const state_machine::State* lastState)
{
    
}

void Drive::executeFunction()
{
    goalPoseWriter->write(goal);
}

void Drive::exit()
{

}

int main(int argc, char** argv)
{
    bool gotPos = false;
    
    int posStart = 0;
    for (int i = 0; i<argc; i++){
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
    
    csu.init->addEdge("Initialized", gm, [&] () {
        return csu.init->isInitialized();
    });

    std::cout << "csu.runLoop.." << std::endl;
    csu.runLoop();
  
    return 0;
}