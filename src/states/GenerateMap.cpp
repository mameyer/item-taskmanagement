#include "GenerateMap.hpp"
#include <state_machine/Config.hpp>
#include <boost/lexical_cast.hpp>
#include <envire/Core.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <graph_slam/proxies/VelodyneSLAM.hpp>

GenerateMap::GenerateMap(State* successState): State("GenerateMap", successState)
{
    slam = new graph_slam::proxies::VelodyneSLAM("slam");
    mapReader = &(slam->envire_map.getReader());
}

bool GenerateMap::genMap()
{
    //generate new map
    int ret = slam->generateMap();

    return ret == 0;
}


void GenerateMap::enter(const State *lastState)
{
    mapReader->clear();

    trigger = true;
}

void GenerateMap::executeFunction()
{
    //Regenerate map if it is empty (should only happen on startup)
    RTT::extras::ReadOnlyPointer<std::vector<envire::BinaryEvent>> map;
    envire::Environment env;

    if(trigger)
    {
	if(genMap())
	  trigger = false;
    }

    bool gotData = false;
    while(mapReader->read(map) == RTT::NewData)
    {
        const std::vector<envire::BinaryEvent>* event = map.get();
        env.applyEvents(*event);
        gotData = true;
    }
    
    if(gotData)
    {
        std::vector<envire::MLSGrid*> maps = env.getItems<envire::MLSGrid>();
        for (envire::MLSGrid* singleMap : maps) 
        {
            if (!singleMap->empty()) {
                finish();
                return;
            }
        }
        
        msg << "Map was empty, regenerating map" << std::endl;
        trigger = true;
    }
    
}

void GenerateMap::exit()
{

}