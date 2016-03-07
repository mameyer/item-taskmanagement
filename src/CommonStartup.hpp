#pragma once

#include "states/Init.hpp"

#include <state_machine/State.hpp>
#include <state_machine/StateMachine.hpp>

class QApplication;
class StateMachineWidget;

class CommonStartup
{
public:
    state_machine::StateMachine* stateMachine;
    Init *init;
    bool planningActive;
    state_machine::Config *config;
    void start(int argc, char** argv);
    
    void runLoop(std::function<void()> loopCallback = [](){});
    
private:
    std::string debugMsgs;
    
};