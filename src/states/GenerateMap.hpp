#ifndef GENERATEMAP_H
#define GENERATEMAP_H

#include <state_machine/State.hpp>
#include "Forward.hpp"
#include <envire/core/EventTypes.hpp>
#include <rtt/Port.hpp>
#include <rtt/extras/ReadOnlyPointer.hpp>

extern template class RTT::InputPort<RTT::extras::ReadOnlyPointer<std::vector<envire::BinaryEvent>>>; 

class GenerateMap : public state_machine::State
{
public:
    GenerateMap(State *successState);

    graph_slam::proxies::VelodyneSLAM *slam;
    RTT::InputPort<RTT::extras::ReadOnlyPointer<std::vector<envire::BinaryEvent>>> *mapReader;
    
    virtual void enter(const State *lastState1);
    virtual void executeFunction();
    virtual void exit();
    
protected:
    bool trigger;
    bool genMap();
};

#endif // GENERATEMAP_H