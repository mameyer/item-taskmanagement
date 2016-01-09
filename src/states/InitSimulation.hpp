#ifndef INITSIMULATION_H
#define INITSIMULATION_H
#include "Init.hpp"

class InitSimulation : public Init
{
public:
    InitSimulation(bool logTasks);
private:
    virtual void executeFunction();
    virtual bool setup();
    virtual bool connect();

    mars::proxies::RotatingLaserRangeFinder *velodyneTask;
    drive_mode_controller::proxies::Task *driveModeControllerTask;
    mars::proxies::IMU *perfectOdometryTask;
    mars::proxies::Joints *jointsTask;
};

#endif // INITSIMULATION_H