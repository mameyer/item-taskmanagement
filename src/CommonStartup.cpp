#include "CommonStartup.hpp"
#include <orocos_cpp/Spawner.hpp>
#include <orocos_cpp/Bundle.hpp>
#include "states/InitSimulation.hpp"

//#include <qt4/QtGui/QApplication>
//#include <state_machine/StateMachineWidget.hpp>
#include <state_machine/StateMachine.hpp>

using namespace orocos_cpp;

void CommonStartup::start(int argc, char** argv)
{
    simulationActive = false;
    bool loggingActive = false;

    for (int i = 0; i<argc; i++) {
        if(strcmp(argv[i], "sim") == 0) {
            simulationActive=true;
            std::cout << "Simulation enabled" << std::endl;
        }
        
        if(strcmp(argv[i], "log") == 0) {
            loggingActive=true;
            std::cout << "Logging enabled" << std::endl;
        }
    }

    if(simulationActive) {
        static int argcp = 0;
        static char** argvp = nullptr;
        //app = new QApplication(argcp, argvp);

        /*widget = new StateMachineWidget();
        widget->show();
        widget->resize(800,600);*/
    }

    Spawner &spawner(Spawner::getInstace());
    config = &(state_machine::Config::getConfig(Bundle::getInstance().getConfigurationDirectory() + "../taskmanagement.yml"));
    
    std::cout << "config loaded.." << std::endl;
    stateMachine = &state_machine::StateMachine::getInstance();

    spawner.spawnDeployment("item_planner");
    std::cout << "spawnDeployment: item_planner" << std::endl;
    spawner.spawnDeployment("item_follower");
    std::cout << "spawnDeployment: item_follower" << std::endl;
    spawner.spawnDeployment("item_graphslam");
    std::cout << "spawnDeployment: item_graphslam" << std::endl;

    if (simulationActive) {
	spawner.spawnDeployment("eo2_sim");
	std::cout << "spawnDeployment: eo2_sim" << std::endl;
        spawner.waitUntilAllReady(base::Time::fromSeconds(15));
	std::cout << "spawnDeployment: waitUntilAllReady" << std::endl;
        init = new InitSimulation(loggingActive);
    } else {

    }

    std::cout << "stateMachine->start.." << std::endl;
    stateMachine->start(init);
}

void CommonStartup::runLoop(std::function<void()> loopCallback)
{
    state_machine::serialization::StateMachine smDump(*stateMachine);

    if(simulationActive)
    {
        /*widget->update(smDump);
        widget->repaint();
        app->processEvents();*/
    }

    base::Time stateMachineLastSend = base::Time::now();

    stateMachine->setExecuteCallback([&]()
    {
        base::Time curTime = base::Time::now();

        //Events for state_machine visualisation + state_machine
        std::vector<state_machine::serialization::Event> newEvents = stateMachine->getNewEvents();
        for(auto e: newEvents)
        {
            if(simulationActive)
            {
                //update widget
                /*widget->update(e);
                widget->repaint();*/
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

        if(simulationActive)
        {
            //app->processEvents();
        }
    });

    while (!stateMachine->execute())
    {
    }
}