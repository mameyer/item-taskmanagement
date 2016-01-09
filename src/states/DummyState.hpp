#ifndef DUMMYSTATE_H
#define DUMMYSTATE_H

#include <state_machine/State.hpp>

class DummyState : public state_machine::State
{
public:
    DummyState() : State("DummyState") {};
    virtual ~DummyState() {};
    virtual void enter(const State* lastState) {};
    virtual void executeFunction() {};
    virtual void exit() {};
};

#endif // DUMMYSTATE_H
