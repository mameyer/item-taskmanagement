#pragma once

#include "states/Init.hpp"

#include <state_machine/State.hpp>
#include <state_machine/StateMachine.hpp>

//class QApplication;

//class StateMachineWidget;

class CommonStartup
{
public:
    state_machine::StateMachine* stateMachine;
    //StateMachineWidget *widget;
    Init *init;
    bool simulationActive;
    state_machine::Config *config;
    //QApplication *app;
    void start(int argc, char** argv);
    
    void runLoop(std::function<void()> loopCallback = [](){});
    
private:
    std::string debugMsgs;
    
};