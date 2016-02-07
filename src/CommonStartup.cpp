#include "CommonStartup.hpp"
#include <orocos_cpp/Spawner.hpp>
#include <orocos_cpp/Bundle.hpp>
#include "states/InitSimulation.hpp"

#include <QApplication>
#include <state_machine/StateMachineWidget.hpp>
#include <state_machine/StateMachine.hpp>

using namespace orocos_cpp;

void CommonStartup::start(int argc, char** argv)
{
    planningActive = false;
    bool loggingActive = false;

    for (int i = 0; i<argc; i++) {
        if(strcmp(argv[i], "sim") == 0) {
            planningActive=true;
            std::cout << "Simulation enabled" << std::endl;
        }

        if(strcmp(argv[i], "log") == 0) {
            loggingActive=true;
            std::cout << "Logging enabled" << std::endl;
        }
    }

    if(planningActive) {
        static int argcp = 0;
        static char** argvp = nullptr;
        app = new QApplication(argcp, argvp);

        widget = new StateMachineWidget();
        widget->show();
        widget->resize(800,600);
    }

    Spawner &spawner(Spawner::getInstace());
    config = &(state_machine::Config::getConfig(Bundle::getInstance().getConfigurationDirectory() + "../taskmanagement.yml"));

    std::cout << "config loaded.." << std::endl;
    stateMachine = &state_machine::StateMachine::getInstance();

    spawner.spawnDeployment("item_follower");
    spawner.spawnDeployment("item_graphslam");

    spawner.spawnDeployment("eo2_sim");
    //spawner.spawnTask("joint_dispatcher::Task", "eo2_dispatcher");
    //spawner.spawnTask("odometry::Skid", "odometry");
    spawner.waitUntilAllReady(base::Time::fromSeconds(15));
    
    if (planningActive) {
	spawner.spawnDeployment("item_planner");
    }
    
    init = new InitSimulation(loggingActive, planningActive);

    stateMachine->start(init);
}

void CommonStartup::runLoop(std::function<void()> loopCallback)
{
    state_machine::serialization::StateMachine smDump(*stateMachine);

    if(planningActive)
    {
        widget->update(smDump);
        widget->repaint();
        app->processEvents();
    }

    base::Time stateMachineLastSend = base::Time::now();

    stateMachine->setExecuteCallback([&]()
    {
        base::Time curTime = base::Time::now();

        //Events for state_machine visualisation + state_machine
        std::vector<state_machine::serialization::Event> newEvents = stateMachine->getNewEvents();
        for(auto e: newEvents)
        {
            if(planningActive)
            {
                //update widget
                widget->update(e);
                widget->repaint();
            }
        }
        //Send SM every 60 seconds
        if((curTime - stateMachineLastSend) > base::Time::fromSeconds(60)) {
            state_machine::serialization::StateMachine smDump(*stateMachine);
            stateMachineLastSend = base::Time::now();
        }

        loopCallback();

        //collect all debug messages
        std::string debugMsgs = stateMachine->getDebugStream().str();
        stateMachine->getDebugStream().str(std::string());

        if(!debugMsgs.empty())
        {
            //write them to proxy and the console
            std::cout << debugMsgs;
        }

        if(planningActive)
        {
            app->processEvents();
        }
    });

    while (!stateMachine->execute())
    {
    }
}