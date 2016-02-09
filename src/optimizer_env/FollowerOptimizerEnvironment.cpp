#include "FollowerOptimizerEnvironment.hpp"

#include <iostream>

namespace bolero {

namespace follower_optimizer_environment {

FollowerOptimizerEnvironment::FollowerOptimizerEnvironment(lib_manager::LibManager *theManager)
    : bolero::Environment(theManager, "follower_environment", 1)
{
    
}

FollowerOptimizerEnvironment::~FollowerOptimizerEnvironment()
{

}

void FollowerOptimizerEnvironment::reset()
{
    errors.clear();
    runs = 0;
}

void FollowerOptimizerEnvironment::init()
{
    runs = 0;
}

void FollowerOptimizerEnvironment::setInputs(const double* values, int numInputs)
{
    // the values from the optimizat are in the range from 0 to 1
    // you should scale it to your search space
    functions.scaleToSearchSpace(values, curParams);
}

void FollowerOptimizerEnvironment::getOutputs(double* values, int numOutputs) const
{

}

void FollowerOptimizerEnvironment::stepAction()
{
    error = 0.2*functions.angleError() + 0.3*functions.distanceError() + 0.5*functions.stability();
    if (functions.stabilityFailed() || functions.maxDistanceErrReached())
	error += 1.;
    std::cout << "[" << runs++ << "] stepAction error: " << error << std::endl;
    errors.push_back(error);
}

bool FollowerOptimizerEnvironment::isEvaluationDone() const
{
    return true;
}

int FollowerOptimizerEnvironment::getFeedback(double* feedback) const
{
    feedback[0] = error;
    return 1;
}

void FollowerOptimizerEnvironment::setFunctions(const OptimizerFunctions& functions)
{
    this->functions = functions;
}

}

}

DESTROY_LIB(bolero::follower_optimizer_environment::FollowerOptimizerEnvironment);
CREATE_LIB(bolero::follower_optimizer_environment::FollowerOptimizerEnvironment);