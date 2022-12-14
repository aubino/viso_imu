#include <iostream>
#include <time.h>
#include <Eigen/Geometry> 
#include <boost/circular_buffer.hpp>
#include "ICM20948.h"
#pragma once 

/*Here you must enter the include to the imu sdk you are using. Here i will use the imu embedded with the imx219-83 stareo camera compatible with jetson nano*/

class Imu 
{
    public:
    std::string name;
    double ax,ay,az,vyaw,vpich,vroll,yaw,pitch,roll;
    double state_vector[15]; //the state vector of the imu (x,y,z,vx,vy,vz,ax,ay,az,yaw,pich,roll,vroll,vpch,vyaw)
    double bias_vector[15];
    bool state_vector_updated;
    Eigen::Quaternion<double> q;
    time_t stamp_from;
    time_t stamp_to;

    Imu(std::string imu_name,time_t start_time);
    Imu(std::string imu_name);
    void Update_state(double ax,double ay,double az,double vyaw,double vpich,double vroll,double yaw,double pitch,double roll,time_t stamp);
    void Update_state(double ax,double ay,double az,double vyaw,double vpich,double vroll,Eigen::Quaternion<double> dq,time_t stamp);
    void setBias(double bias[15]);
    void removeG();
    friend class ImuQueue;
    
};

class ImuQueue
{
    public:
    boost::circular_buffer<Imu> relative_queue;
    boost::circular_buffer<Imu> absolute_queue;
    std::shared_ptr<Imu>  device;
    void stack_absolute(Imu);
    void stack_absolute();
    void stack_relative(Imu);
    void stack_relative();
    Eigen::Vector3d gravity0;
    Eigen::Transform<double,3,Eigen::Affine> OriginTransform;
    int queue_size;
    double sampling_period;
    ImuQueue(std::shared_ptr<Imu> imu,double T,int max_size=10000);
    Eigen::Transform<double,3,Eigen::Affine> getTransform(time_t t);
    Eigen::Transform<double,3,Eigen::Affine> get_Transform(time_t t_from, time_t t_to);
    Eigen::Transform<double,3,Eigen::Affine> get_TransformSmart(time_t t_from, time_t t_to);
    Eigen::Transform<double,3,Eigen::Affine> getTransformSmart(time_t t);
    Eigen::Transform<double,3,Eigen::Affine> get_OriginTransform();
};