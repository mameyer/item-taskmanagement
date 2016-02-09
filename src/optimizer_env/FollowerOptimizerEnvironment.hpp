#pragma once

#include <bolero/Environment.h>
#include <functional>
#include <vector>

namespace bolero {
namespace follower_optimizer_environment {

struct OptimizerFunctions {
    std::function< double() > angleError, distanceError, stability;
    std::function< bool() > stabilityFailed, maxDistanceErrReached;
    std::function< void(const double* values, std::vector< double > &params) > scaleToSearchSpace;
};

class FollowerOptimizerEnvironment : public Environment
{

public:
    FollowerOptimizerEnvironment(lib_manager::LibManager *theManager);
    virtual ~FollowerOptimizerEnvironment();

    CREATE_MODULE_INFO();

    virtual void init();
    virtual void reset();
    virtual void setFunctions(const OptimizerFunctions& functions);

    /**
    * These functions are used for the controller interfacing a
    * behavior to an environment
    */
    virtual int getNumInputs() const {
	return numInputs;
    }
    
    virtual int getNumOutputs() const {
	return 0;
    }
    
    virtual void getOutputs(double *values, int numOutputs) const;
    virtual void setInputs(const double *values, int numInputs);
    virtual void stepAction();
    virtual bool isEvaluationDone() const;
    virtual int getFeedback(double *feedback) const;
    
    bool isBehaviorLearningDone() const {
        return false;
    }
    
    void setNumInputs(int numInputs) {
	this->numInputs = numInputs;
    }
    
    std::vector< double > getErrors() {
	return errors;
    }
    
    std::vector< double > getCurParams() {
	return curParams;
    }
    
private:
    // add whatever you need
    std::vector< double > curParams;
    double error;
    int runs;
    OptimizerFunctions functions;
    int numInputs;
    std::vector<double > errors;

}; // end of class definition ExampleEnvironment

} // end of namespace follower_environment

} // end of namespace bolero