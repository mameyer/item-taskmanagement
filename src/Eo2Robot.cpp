#include "Eo2Robot.hpp"

using namespace smurf;
using namespace Eigen;

Eo2Robot::Eo2Robot()
{   
    Frame *slam = new Frame("slam");
    availableFrames.push_back(slam);
    
    Frame *worldOsg = new Frame("world_osg");
    availableFrames.push_back(worldOsg);
    
    Frame *imu = new Frame("imu");
    availableFrames.push_back(imu);
    
    Frame *body = new Frame("body");
    availableFrames.push_back(body);
    
    Frame *laser = new Frame("laser");
    availableFrames.push_back(laser);
    
    Frame *bodyCenter = new Frame("body_center");
    availableFrames.push_back(bodyCenter);
    
    Frame *baseLink = new Frame("base_link");
    availableFrames.push_back(baseLink);
    
    Frame *bodyPlane = new Frame("body_plane");
    availableFrames.push_back(bodyPlane);
    
    Frame *odometry = new Frame("odometry");
    availableFrames.push_back(odometry);
    
    Frame *imuNwu = new Frame("imu_nwu");
    availableFrames.push_back(imuNwu);
    
    
    staticTransforms.push_back(
        new StaticTransformation(slam, worldOsg, 
                                 Eigen::Quaterniond::Identity(), 
                                 Eigen::Vector3d(0.0, 0.0, 0.0)));
    
    staticTransforms.push_back(
        new StaticTransformation(imu, body, 
                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()), 
                                 Eigen::Vector3d(0.5, 0.0, 1.6)));
    
    staticTransforms.push_back(
        new StaticTransformation(laser, body, 
                                 Eigen::Quaterniond::Identity(), 
                                 Eigen::Vector3d(0.0, 0.0, 0.0)));
    
    staticTransforms.push_back(
        new StaticTransformation(bodyCenter, body, 
                                 Eigen::Quaterniond::Identity(), 
                                 Eigen::Vector3d(0.0, 0.0, 0.0)));
    
    staticTransforms.push_back(
        new StaticTransformation(baseLink, body, 
                                 Eigen::Quaterniond::Identity(), 
                                 Eigen::Vector3d(-0.035, 0.000, -0.350)));
    
    staticTransforms.push_back(
        new StaticTransformation(bodyPlane, body, 
                                 Eigen::Quaterniond::Identity(), 
                                 Eigen::Vector3d(0.0, 0.0, -0.540)));
    
    
    dynamicTransforms.push_back(new DynamicTransformation(body, odometry, "perfect_odometry", "pose_samples"));
    dynamicTransforms.push_back(new DynamicTransformation(body, slam, "velodyne_slam", "pose_samples"));
    dynamicTransforms.push_back(new DynamicTransformation(imu, imuNwu, "xsens", "orientation_samples"));
}