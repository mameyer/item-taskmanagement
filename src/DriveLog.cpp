#include "states/DummyState.hpp"
#include "states/GenerateMap.hpp"

#include "CommonStartup.hpp"
#include <orocos_cpp/Configuration.hpp>
#include <orocos_cpp/ConfigurationHelper.hpp>

#include <orocos_cpp/Bundle.hpp>
#include <state_machine/StateMachine.hpp>
#include <motion_planning_libraries/proxies/Task.hpp>

#include <base/samples/RigidBodyState.hpp>
#include <base/Angle.hpp>
#include <trajectory_follower/proxies/Task.hpp>
#include <base/Trajectory.hpp>

#include <orocos_cpp/Bundle.hpp>
#include <orocos_cpp/YAMLConfiguration.hpp>

#include <rtt/types/TypekitRepository.hpp>
#include <rtt/types/TypeInfoRepository.hpp>
#include <rtt/types/TypeInfo.hpp>
#include <rtt/typelib/TypelibMarshallerBase.hpp>

#include <base/Trajectory.hpp>
#include <vector>
#include <map>
#include <stdexcept>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <ostream>
#include <boost/filesystem.hpp>

#include <base/Time.hpp>
#include <trajectory_follower/TrajectoryFollowerTypes.hpp>

#include <functional>
#include <algorithm>
#include <random>

#include <base/Float.hpp>

// optimizer
#include <bolero/Environment.h>
#include <bolero/bl_loader/BLLoader.h>
#include <bolero/Optimizer.h>
#include "optimizer_env/FollowerOptimizerEnvironment.hpp"
#include <configmaps/ConfigData.h>


class MyOstream
{
public:
    MyOstream(const std::string &fullPath) : myFstream(fullPath) {};
    // for regular output of variables and stuff
    template<typename T> MyOstream& operator<<(const T& msg)
    {
        std::cout << msg;
        myFstream << msg;
        return *this;
    }
    // for manipulators like std::endl
    typedef std::ostream& (*stream_function)(std::ostream&);
    MyOstream& operator<<(stream_function func)
    {
        func(std::cout);
        func(myFstream);
        return *this;
    }
private:
    std::ofstream myFstream;
};

class FollowerControllerRating {
public:
    base::Time startTime, endTime;
    double angleError;
    double distanceError;
    bool reachedEnd;
    unsigned int numMotionCommands;
    double angleErrorRelative;
    double distanceErrorRelative;
    bool stabilityFailed;
    bool maxDistanceErrorReached;
    double stability;
    trajectory_follower::ControllerType controllerType;
    std::string conf;
    double error;

    FollowerControllerRating()
        : angleError(std::numeric_limits< double >::max()),
          distanceError(std::numeric_limits< double >::max()),
          angleErrorRelative(std::numeric_limits< double >::max()),
          distanceErrorRelative(std::numeric_limits< double >::max()),
          stability(std::numeric_limits< double >::max()),
          error(std::numeric_limits< double >::max())
    {
        reachedEnd = false;
        numMotionCommands = 0;
        stabilityFailed = false;
        maxDistanceErrorReached = false;
        conf = "";
    }

    friend std::ostream& operator<<(std::ostream& os, const FollowerControllerRating& fcr);
};

std::ostream& operator<<(std::ostream& os, const FollowerControllerRating& fcr)
{
    os << "aglErr: " << std::to_string(fcr.angleError) << "[" << std::to_string(fcr.angleErrorRelative) << "]" << ", distErr: " << std::to_string(fcr.distanceError) << "[" << std::to_string(fcr.distanceErrorRelative) << "] -- " << "stab -> " << std::to_string(fcr.stability);
    return os;
}

class DriveLog: public state_machine::State
{
public:
    DriveLog(State* success, State* failue);
    ~DriveLog();
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
    inline void setLogDir(const std::string &logDir) {
        this->logDir = logDir;
    };
    Init *init;
    bool isEvaluationDone() {
        return evaluationDone;
    };

private:
    const double maxK0NoOrientation = 0.3;
    const double maxK0Chained = 0.05;
    const double maxK2 = 5.;
    const double maxK3 = 5.;
    const double maxL1 = 0.3;

    const double maxDistanceError = 1.5;
    const unsigned int maxSeconds = 120;
    const unsigned int debugRatingCnt = 10;

    std::shared_ptr< MyOstream > stream;
    bool wasFollowing;
    std::map< std::string, std::vector< base::Trajectory > > trajectories;
    state_machine::Config *config;
    trajectory_follower::proxies::Task *trajectoryFollower;
    RTT::InputPort< int > *trajectoryFollowerStateReader;
    RTT::OutputPort< std::vector< base::Trajectory > > *trajectoryWriter;
    localization::proxies::PoseProvider *poseProviderTask;
    RTT::InputPort< base::samples::RigidBodyState > *poseReader;
    std::string logDir;
    std::map< std::string, std::vector< base::Trajectory > >::iterator trajectoriesIter;
    std::map< std::string, double > configValues;
    orocos_cpp::ConfigurationHelper confHelper;
    bool updateConfig(const std::map< std::string, double > &confVals, trajectory_follower::ControllerType controllerType, RTT::TaskContext *task);
    void readTrajectory(std::vector< base::Trajectory > &trajectory, const std::string& fullPath);
    void readTrajectories(std::map< std::string, std::vector< base::Trajectory > > &trajectories, const std::string& dir);
    std::map< std::string, std::vector< FollowerControllerRating > > rating;
    RTT::InputPort< trajectory_follower::FollowerData > *followerDataReader;
    double distanceErrSum, angleErrSum, stabilitySum;
    std::string confValsStr;
    double lastRot;
    trajectory_follower::ControllerType currentTrajectoryControllerType;
    bool badConditions;

    // optimizer
    bolero::Optimizer *blOptimizer;
    bolero::bl_loader::BLLoader *blLoader;
    bolero::follower_optimizer_environment::FollowerOptimizerEnvironment *environment;
    double* feedbacks;
    int num_feedbacks;
    double feedback;
    std::string confFile;
    std::string strEnvironment;
    std::string strBLOptimizer;
    int maxEvaluations;
    int numInputs;
    int numOutputs;
    double *inputs;
    double *outputs;
    int evaluationCount;
    bool evaluationDone;
    configmaps::ConfigMap map;
    bool failedLastTime;
    bolero::follower_optimizer_environment::OptimizerFunctions opFuns;
};

DriveLog::DriveLog(state_machine::State* success, state_machine::State* failure)
    : State("DriveLog", success, failure)
{
    trajectoryFollower = new trajectory_follower::proxies::Task("follower");
    poseProviderTask = new localization::proxies::PoseProvider("pose_provider");
    trajectoryFollowerStateReader = &(trajectoryFollower->state.getReader());
    trajectoryWriter = &(trajectoryFollower->trajectory.getWriter());
    poseReader = &(poseProviderTask->pose_samples.getReader());
    followerDataReader = &(trajectoryFollower->follower_data.getReader(RTT::ConnPolicy::buffer(500)));
    wasFollowing = false;
    failedLastTime = false;

    const char* autoproj_root = std::getenv("AUTOPROJ_CURRENT_ROOT");
    if (autoproj_root == NULL)
        std::runtime_error("Env AUTOPROJ_CURRENT_ROOT not available, return");

    std::string autoproj_root_str(autoproj_root);
    logDir = autoproj_root_str + std::string("/bundles/eo2/logs");

    std::cout << "Helper::readTrajectories: " << logDir << std::endl;
    readTrajectories(trajectories, logDir);

    if (trajectories.empty())
        throw std::runtime_error("DriveLog::enter no trajectories set..");

    trajectoriesIter = trajectories.begin();

    map = configmaps::ConfigMap::fromYamlFile(std::string("../../data/learning_config.yml"));
    std::string strCotrollerType = map["ControllerType"][0].getString();
    std::string loggerPath = logDir + std::string("/");
    
    if (strCotrollerType == "CONTROLLER_NO_ORIENTATION") {
	currentTrajectoryControllerType = trajectory_follower::ControllerType::CONTROLLER_NO_ORIENTATION;
	loggerPath += std::string("CONTROLLER_NO_ORIENTATION__");
    } else if (strCotrollerType == "CONTROLLER_CHAINED") {
	currentTrajectoryControllerType = trajectory_follower::ControllerType::CONTROLLER_CHAINED;
	loggerPath += std::string("CONTROLLER_CHAINED__");
    } else {
	std::runtime_error("wrong initialization for currentTrajectoryControllerType");
    }

    loggerPath += base::Time::now().toString() + std::string(".rating");

    stream = std::shared_ptr< MyOstream >(new MyOstream(loggerPath));
    *stream.get() << "[[" << base::Time::now().toString() << "]] started rating: " << loggerPath << std::endl;

    
    // optimizer
    evaluationDone = false;

    blLoader = new bolero::bl_loader::BLLoader();
    blLoader->loadConfigFile(std::string("../../data/learning_libraries.txt"));
    blLoader->dumpTo(std::string("../../data/libs_info.xml"));
   
    strEnvironment = map["Environment"][0].getString();
    *stream.get() << "optimizer | Environment: " << strEnvironment << std::endl;
    strBLOptimizer = map["Optimizer"][0].getString();
    *stream.get() << "optimizer | Optimizer: " << strBLOptimizer << std::endl;
    maxEvaluations = map["MaxEvaluations"][0].getInt();
    *stream.get() << "optimizer | MaxEvaluations: " << maxEvaluations << std::endl;    

    feedbacks = new double[maxEvaluations];
    num_feedbacks = 0;

    environment = dynamic_cast< bolero::follower_optimizer_environment::FollowerOptimizerEnvironment* >(blLoader->acquireEnvironment(strEnvironment));
    blOptimizer = blLoader->acquireOptimizer(strBLOptimizer);

    assert(environment);
    assert(blOptimizer);

    environment->init();

    switch (currentTrajectoryControllerType) {
    case trajectory_follower::ControllerType::CONTROLLER_NO_ORIENTATION:
        environment->setNumInputs(2);
        break;
    case trajectory_follower::ControllerType::CONTROLLER_CHAINED:
        environment->setNumInputs(3);
        break;
    default:
        std::runtime_error("wrong initialization for currentTrajectoryControllerType");
        break;
    }


    // optimizer functions
    opFuns.angleError = [&]() {
        return this->rating[trajectoriesIter->first].back().angleError;
    };
    opFuns.distanceError = [&]() {
        return this->rating[trajectoriesIter->first].back().distanceError;
    };
    opFuns.stability = [&]() {
        return this->rating[trajectoriesIter->first].back().stability;
    };
    opFuns.scaleToSearchSpace = [&](const double* values, std::vector< double > &params) {
        params.clear();
        switch (currentTrajectoryControllerType) {
        case trajectory_follower::ControllerType::CONTROLLER_NO_ORIENTATION:
            params.resize(2);
            params[0] = maxK0NoOrientation*values[0];
            params[1] = maxL1*values[1];
            break;
        case trajectory_follower::ControllerType::CONTROLLER_CHAINED:
            params.resize(3);
            params[0] = maxK0Chained*values[0];
            params[1] = maxK2*values[1];
            params[2] = maxK3*values[2];
            break;
        default:
            std::runtime_error("wrong initialization for currentTrajectoryControllerType");
            break;
        }
    };
    opFuns.maxDistanceErrReached = [&]() {
        return this->rating[trajectoriesIter->first].back().maxDistanceErrorReached;
    };
    opFuns.stabilityFailed = [&]() {
        return this->rating[trajectoriesIter->first].back().stabilityFailed;
    };

    environment->setFunctions(opFuns);

    numInputs = environment->getNumOutputs();
    numOutputs = environment->getNumInputs();

    inputs = new double[numInputs];
    outputs = new double[numOutputs];

    blOptimizer->init(numOutputs);

    evaluationCount = 0;
}

void DriveLog::enter(const state_machine::State* lastState)
{
    // generate controller config using optimizer
    if (!failedLastTime) {
        blOptimizer->getNextParameters(outputs, numOutputs);
        environment->setInputs(outputs, numOutputs);
    } else {
        failedLastTime = false;
    }

    switch (currentTrajectoryControllerType) {
    case trajectory_follower::ControllerType::CONTROLLER_NO_ORIENTATION:
        configValues["l1"] = environment->getCurParams()[0];
        configValues["K0"] = environment->getCurParams()[1];
        break;
    case trajectory_follower::ControllerType::CONTROLLER_CHAINED:
        configValues["K0"] = environment->getCurParams()[0];
        configValues["K2"] = environment->getCurParams()[1];
        configValues["K3"] = environment->getCurParams()[2];
        break;
    default:
        std::runtime_error("wrong initialization for currentTrajectoryControllerType");
        break;
    }


    // stop follower and apply new config
    trajectoryFollower->stop();
    if (updateConfig(configValues, currentTrajectoryControllerType, trajectoryFollower)) {
        trajectoryFollower->configure();
        usleep(100);
    }

    if (!rating[trajectoriesIter->first].empty()) {
        std::sort(rating[trajectoriesIter->first].begin(), rating[trajectoriesIter->first].end(),
                  [](const FollowerControllerRating & a, const FollowerControllerRating & b) -> bool
        {
            return a.error < b.error;
        });

        unsigned int upper = std::min(debugRatingCnt, static_cast<unsigned int>(rating[trajectoriesIter->first].size()));
        *stream.get() << std::endl;
        for (unsigned int i=0; i<upper; i++) {
            if ((rating[trajectoriesIter->first].at(i).angleError == std::numeric_limits< double >::max())
                    || (rating[trajectoriesIter->first].at(i).distanceError == std::numeric_limits< double >::max())
                    || (rating[trajectoriesIter->first].at(i).stability == std::numeric_limits< double >::max())
		    || rating[trajectoriesIter->first].at(i).maxDistanceErrorReached
		    || rating[trajectoriesIter->first].at(i).stabilityFailed)
                break;

            *stream.get() << "best stability[" << i << "]: " << rating[trajectoriesIter->first].at(i) << " | " << rating[trajectoriesIter->first].at(i).conf << std::endl;
        }
    }

    FollowerControllerRating currentRating;
    rating[trajectoriesIter->first].push_back(currentRating);

    confValsStr.clear();
    for (auto iter: configValues)
        confValsStr += std::string("{") + iter.first + std::string(": ") + std::to_string(iter.second) + std::string("}, ");
    if (confValsStr.size() > 2) confValsStr.erase(confValsStr.end()-2, confValsStr.end());

    rating[trajectoriesIter->first].back().controllerType = currentTrajectoryControllerType;
    rating[trajectoriesIter->first].back().conf = confValsStr;

    distanceErrSum = 0;
    angleErrSum = 0;
    lastRot = 0;
    stabilitySum = 0;

    usleep(1000);
    base::samples::RigidBodyState currentPose;
    while (poseReader->readNewest(currentPose) == RTT::NoData) {
        usleep(100);
    }

    if (currentPose.getPitch() < base::Angle::fromDeg(-45).getRad()
            || currentPose.getPitch() > base::Angle::fromDeg(45).getRad()
            || currentPose.getRoll() < base::Angle::fromDeg(-45).getRad()
            || currentPose.getRoll() > base::Angle::fromDeg(45).getRad()) {
        *stream.get() <<  "[FAIL] invalid pose.." << std::endl;
	failedLastTime = true;
    }

    if (currentPose.position.x() !=0.
            || currentPose.position.y() != 0.) {
        *stream.get() <<  "[WARN] currentPose different from (0,0).." << std::endl;
    }

    trajectoryFollower->start();
    usleep(100);

    followerDataReader->clear();
    poseReader->clear();
    trajectoryFollowerStateReader->clear();

    trajectoryWriter->write(trajectoriesIter->second);
}

void DriveLog::executeFunction()
{
    trajectory_follower::FollowerData followerData;
    if (wasFollowing) {
        while (followerDataReader->read(followerData) == RTT::NewData) {
            if (std::abs(followerData.distanceError) > maxDistanceError) {
                rating[trajectoriesIter->first].back().maxDistanceErrorReached = true;
                *stream.get() << "[[" << base::Time::now().toString() << "]] max distance error reached for " << trajectoriesIter->first << "; " << confValsStr << std::endl;
                fail();
		return;
            }

            rating[trajectoriesIter->first].back().numMotionCommands++;
            distanceErrSum += followerData.distanceError;
            angleErrSum += followerData.angleError;
            stabilitySum += std::abs(lastRot-followerData.motionCommand.rotation);
            lastRot = followerData.motionCommand.rotation;
        }
    }

    int trajectoryFollowerState;
    if (trajectoryFollowerStateReader->readNewest(trajectoryFollowerState) == RTT::NewData) {
        if (trajectoryFollowerState == trajectory_follower::Task_STATES::Task_STABILITY_FAILED) {
            rating[trajectoriesIter->first].back().stabilityFailed = true;
            *stream.get() << "[[" << base::Time::now().toString() << "]] stability failed for " << trajectoriesIter->first << "; " << confValsStr << std::endl;
            fail();
	    return;
        }

        if (!wasFollowing && trajectoryFollowerState == trajectory_follower::Task_STATES::Task_FOLLOWING_TRAJECTORY)
            wasFollowing = true;

        if (wasFollowing && trajectoryFollowerState == trajectory_follower::Task_STATES::Task_FINISHED_TRAJECTORIES) {
            wasFollowing = false;
            while (followerDataReader->readNewest(followerData) == RTT::NoData) {
                usleep(100);
            }

            distanceErrSum += followerData.distanceError;
            angleErrSum += followerData.angleError;
            stabilitySum += std::abs(lastRot-followerData.motionCommand.rotation);

            if (rating[trajectoriesIter->first].back().numMotionCommands > 0) {
                rating[trajectoriesIter->first].back().stability = stabilitySum/rating[trajectoriesIter->first].back().numMotionCommands;
                rating[trajectoriesIter->first].back().distanceErrorRelative = distanceErrSum/rating[trajectoriesIter->first].back().numMotionCommands;
                rating[trajectoriesIter->first].back().angleErrorRelative = angleErrSum/rating[trajectoriesIter->first].back().numMotionCommands;
            }

            rating[trajectoriesIter->first].back().angleError = followerData.angleError;
            rating[trajectoriesIter->first].back().distanceError = followerData.distanceError;

            *stream.get() << "[[" << base::Time::now().toString() << "]] current rating[" << rating[trajectoriesIter->first].back().numMotionCommands << "]: " << rating[trajectoriesIter->first].back() << "; " << confValsStr << std::endl;
            finish();
        }
    }
}

void DriveLog::exit()
{
    if (failedLastTime) {
	*stream.get() << "exit. last time failed... " << std::endl;
	return;
    }
    
    *stream.get() << "finished evaluation no " << evaluationCount << std::endl;

    // optimizer
    environment->stepAction();
    num_feedbacks = environment->getFeedback(feedbacks);
    feedback = 0.0;
    for(int i = 0; i < num_feedbacks; i++)
        feedback += feedbacks[i];
    
    rating[trajectoriesIter->first].back().error = feedback;

    blOptimizer->setEvaluationFeedback(feedbacks, num_feedbacks);
    environment->reset();
    evaluationCount++;

    if (evaluationCount > maxEvaluations
            || blOptimizer->isBehaviorLearningDone()
            || environment->isBehaviorLearningDone()) {
        blOptimizer->getBestParameters(outputs, numOutputs);
        *stream.get() << "[[" << base::Time::now().toString() << "]] evaluation done!!"  << std::endl << std::endl;

        std::vector< double > bestConfVals;
        opFuns.scaleToSearchSpace(outputs, bestConfVals);

        std::ostringstream oss;
        if (!bestConfVals.empty())
        {
            std::copy(bestConfVals.begin(), bestConfVals.end()-1, std::ostream_iterator<double>(oss, ","));
            oss << bestConfVals.back();
        }
        *stream.get() << "best conf values: " << oss.str() << std::endl;

        evaluationDone = true;
    }
}

int main(int argc, char** argv)
{
    bool gotLogDir = false;

    int logStart = 0;
    for (int i = 0; i<argc; i++) {
        if(strcmp(argv[i], "--logpath") == 0) {
            logStart = i + 1;
            gotLogDir = true;
            break;
        }
    }

    if (gotLogDir) {
        if (logStart > argc) {
            std::cout << "Usage ./Cmd --logpath <path>" << std::endl;
            return -1;
        }
    }

    CommonStartup csu;
    csu.start(argc, argv);
    std::cout << "csu.start.." << std::endl;

    DummyState *dummy = new DummyState();
    DriveLog *drive = new DriveLog(dummy, dummy);
    GenerateMap* gm = new GenerateMap(drive);
    drive->init = csu.init;

    if (gotLogDir)
        drive->setLogDir(std::string(argv[logStart]));

    csu.init->addEdge("Initialized", gm, [&] () {
        return csu.init->isInitialized();
    });

    std::cout << "csu.runLoop.." << std::endl;
    csu.runLoop([&] () {
        if (drive->isEvaluationDone()) {
            std::cout << "break csu.runLoop because evaluation is done.." << std::endl;
            return;
        }

        if (&csu.stateMachine->getCurrentState() == dummy) {
            csu.init->restart();
            usleep(1000);
            csu.stateMachine->start(drive);
        }
    });

    return 0;
}

bool DriveLog::updateConfig(const std::map< std::string, double > &confVals, trajectory_follower::ControllerType controllerType, RTT::TaskContext *task)
{
    std::string confDir = orocos_cpp::Bundle::getInstance().getConfigurationDirectory();
    std::string fullPath = confDir + std::string("trajectory_follower::Task.yml");
    orocos_cpp::YAMLConfigParser yamlParser;
    std::map<std::string, orocos_cpp::Configuration> subConfigs;
    yamlParser.loadConfigFile(fullPath, subConfigs);

    std::string controllerConfigName = "";
    std::string cType = "";
    switch (controllerType) {
    case trajectory_follower::ControllerType::CONTROLLER_NO_ORIENTATION:
        cType = ":CONTROLLER_NO_ORIENTATION";
        controllerConfigName = "noOrientationControllerConfig";
        break;
    case trajectory_follower::ControllerType::CONTROLLER_CHAINED:
        cType = ":CONTROLLER_CHAINED";
        controllerConfigName = "chainedControllerConfig";
        break;
    default:
        std::runtime_error("wrong initialization for currentTrajectoryControllerType");
        break;
    }

    bool needsApply = false;
    for (auto conf: subConfigs) {
        std::map<std::string, std::shared_ptr<orocos_cpp::ConfigValue> > confValues = conf.second.getValues();
        for (auto vals: confValues) {
            if (vals.second->getType() == orocos_cpp::ConfigValue::Type::COMPLEX) {
                orocos_cpp::ComplexConfigValue *configValue = dynamic_cast< orocos_cpp::ComplexConfigValue * >(vals.second.get());
                const std::map< std::string, std::shared_ptr< orocos_cpp::ConfigValue > > subConfValues = configValue->getValues();

                for (auto subVals: subConfValues) {
                    std::cout << "subConfigs value: " << subVals.first << std::endl;
                    if (subVals.first == "controllerType") {
                        if (subVals.second->getType() == orocos_cpp::ConfigValue::Type::SIMPLE) {
                            orocos_cpp::SimpleConfigValue *subConfigValue = dynamic_cast< orocos_cpp::SimpleConfigValue * >(subVals.second.get());
                            std::cout << "subVals: " << subVals.first << ", " << subConfigValue->getValue() << std::endl;

                            if (subConfigValue->getValue() != cType) {
                                std::shared_ptr< orocos_cpp::SimpleConfigValue > newValue(new orocos_cpp::SimpleConfigValue(cType));
                                newValue->setName(subVals.first);
                                subConfigValue->merge(newValue);
                                std::cout << "after merge: " << subVals.first << ", " << subConfigValue->getValue() << std::endl;
                                needsApply = true;
                            }
                        }
                    }

                    if (subVals.first == controllerConfigName) {
                        if (subVals.second->getType() == orocos_cpp::ConfigValue::Type::COMPLEX) {
                            orocos_cpp::ComplexConfigValue *subConfigValue = dynamic_cast< orocos_cpp::ComplexConfigValue * >(subVals.second.get());
                            const std::map< std::string, std::shared_ptr< orocos_cpp::ConfigValue > > subSubConfValues = subConfigValue->getValues();

                            for (auto subSubVals: subSubConfValues) {
                                auto confValsIter = confVals.find(subSubVals.first);
                                if (subSubVals.second->getType() == orocos_cpp::ConfigValue::Type::SIMPLE && confValsIter != confVals.end()) {
                                    orocos_cpp::SimpleConfigValue *subSubConfigValue = dynamic_cast< orocos_cpp::SimpleConfigValue * >(subSubVals.second.get());
                                    std::cout << "subSubVals: " << subSubVals.first << ", " << subSubConfigValue->getValue() << std::endl;
                                    std::shared_ptr< orocos_cpp::SimpleConfigValue > newValue(new orocos_cpp::SimpleConfigValue(std::to_string(confValsIter->second)));
                                    newValue->setName(subSubVals.first);
                                    subSubConfigValue->merge(newValue);
                                    std::cout << "after merge: " << subSubVals.first << ", " << subSubConfigValue->getValue() << std::endl;
                                }
                            }

                            needsApply = true;
                        }
                    }
                }
            }
        }

        if (needsApply) {
            orocos_cpp::ConfigurationHelper confHelper;
            confHelper.applyConfig(task, conf.second);
        }
    }

    return needsApply;
}

void DriveLog::readTrajectory(std::vector< base::Trajectory > &trajectory, const std::string& fullPath)
{
    if (!boost::filesystem::exists(fullPath))
        std::runtime_error("file does not exist");

    std::ifstream stream(fullPath, std::ios::in | std::ios::binary);
    std::string str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    std::vector< u_int8_t > data(str.begin(), str.end());

    std::cout << "Helper::readTrajectory: data.size() = " << data.size() << std::endl;

    RTT::types::TypekitRepository::getTransports();
    RTT::types::TypeInfoRepository::shared_ptr ti = RTT::types::TypeInfoRepository::Instance();
    RTT::types::TypeInfo* type = ti->type("/std/vector</base/Trajectory>");
    if (! type)
    {
        throw std::runtime_error("cannot find /std/vector</base/Trajectory> in the type info repository");
    }

    orogen_transports::TypelibMarshallerBase* transport = dynamic_cast<orogen_transports::TypelibMarshallerBase*>(type->getProtocol(orogen_transports::TYPELIB_MARSHALLER_ID));
    if (! transport)
    {
        throw std::runtime_error(std::string("cannot report ports of type ") + type->getTypeName() + std::string(" as no typekit generated by orogen defines it"));
    }

    orogen_transports::TypelibMarshallerBase::Handle* handle =  transport->createSample();
    transport->setOrocosSample(handle, (void *)&trajectory);
    transport->unmarshal(data, handle);
    transport->refreshOrocosSample(handle);
}

void DriveLog::readTrajectories(std::map< std::string, std::vector< base::Trajectory > > &trajectories, const std::string& dir)
{
    trajectories.clear();
    boost::filesystem::path directory(dir);
    boost::filesystem::directory_iterator endIter;

    if (!boost::filesystem::exists(directory) || !boost::filesystem::is_directory(directory))
        std::runtime_error("log folder does not exist");

    for (boost::filesystem::directory_iterator dirIter(directory); dirIter != endIter; dirIter++) {
        if (boost::filesystem::is_regular_file(dirIter->status())) {
            std::string fullPath = dir + std::string("/") + dirIter->path().filename().string();

            try {
                std::vector< base::Trajectory > trajectory;
                readTrajectory(trajectory, fullPath);
                if (!trajectory.empty())
                    trajectories[fullPath] = trajectory;
            } catch (...) {
                std::cout << "readTrajectories: " << fullPath << "not found.." << std::endl;
            }
        }
    }
}

DriveLog::~DriveLog()
{
    delete[] feedbacks;

    try {
        blLoader->releaseLibrary(strEnvironment);
    } catch(std::runtime_error e) {
        std::cout << e.what() << std::endl;
    }

    try {
        blLoader->releaseLibrary(strBLOptimizer);
    } catch(std::runtime_error e) {
        std::cout << e.what() << std::endl;
    }

    delete[] inputs;
    delete[] outputs;
}