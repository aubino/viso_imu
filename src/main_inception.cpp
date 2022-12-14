#include "imu.h"
#include <iostream>
#include "transform_computer.hpp"
#include "image.h"
#include <signal.h>
#include <thread>
#include <mutex>
#include <chrono>
#define PI 3.14159265359
#define G 9.81
bool sigterm = 0;
// Image stream acquisition 
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int capture_width = 1280 ;
int capture_height = 720 ;
int display_width = 1280 ;
int display_height = 720 ;
int framerate = 30 ;
int flip_method = 0 ;



void signal_callback_handler(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   sigterm = 1;
   exit(signum);
}
// Imu stack datas as mutex ressources
std::shared_ptr<Imu> imu_ptr; 
ImuQueue imu_stack(imu_ptr,0.1,10000);
std::mutex imu_stack_mutex;

void imu_tread(double period)
{
    /*Set up the imu sdk if necessary*/
    IMU_EN_SENSOR_TYPE enMotionSensorType;
	IMU_ST_ANGLES_DATA stAngles;
	IMU_ST_SENSOR_DATA stGyroRawData;
	IMU_ST_SENSOR_DATA stAccelRawData;
	IMU_ST_SENSOR_DATA stMagnRawData;

	imuInit(&enMotionSensorType);
	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
	{
		printf("Motion sersor is ICM-20948\n" );
	}
	else
	{
		printf("Motion sersor NULL\n");
	}
    //End of seting up imu sdk
    imu_stack.sampling_period = period; 
    while(!sigterm)
    {
        imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        time_t t = time(NULL);
        if(imu_stack_mutex.try_lock())
        {
            if(imu_stack.absolute_queue.size()==0)
            { //first transfrom to stack. 
                Imu measure("imuFrame");
                measure.stamp_from= t;
                measure.Update_state(stAccelRawData.fX*G,
                    stAccelRawData.fY*G,
                    stAccelRawData.fZ*G,
                    stGyroRawData.fX*PI/180,
                    stGyroRawData.fY*PI/180,
                    stGyroRawData.fZ*PI/180,
                    stAngles.fRoll*PI/180,
                    stAngles.fPitch*PI/180,
                    stAngles.fYaw*PI/180,time(NULL));
                imu_stack.absolute_queue.push_front(measure);
                imu_stack_mutex.unlock();
                int slp_for = period*1000;
                std::this_thread::sleep_for (std::chrono::milliseconds(slp_for));
            }
            else
            {
                Imu measure = imu_stack.absolute_queue[0];
                Eigen::Vector3d P(measure.state_vector[0],measure.state_vector[1],measure.state_vector[2]);
                Imu measurediff(measure.name);
                measurediff.stamp_from=measure.stamp_to;
                measurediff.Update_state(stAccelRawData.fX*G,
                    stAccelRawData.fY*G,
                    stAccelRawData.fZ*G,
                    stGyroRawData.fX*PI/180,
                    stGyroRawData.fY*PI/180,
                    stGyroRawData.fZ*PI/180,
                    stAngles.fRoll*PI/180,
                    stAngles.fPitch*PI/180,
                    stAngles.fYaw*PI/180,t);
                Eigen::Transform<double,3,Eigen::Affine> difftrans = Eigen::Translation<double,3>(measurediff.state_vector[0],measurediff.state_vector[1],measurediff.state_vector[2])*(measurediff.q*measure.q.inverse()).toRotationMatrix();
                Eigen::Vector3d NEWP = difftrans*P;
                measurediff.state_vector[0] = NEWP[0];
                measurediff.state_vector[1] = NEWP[1];
                measurediff.state_vector[2] = NEWP[2];
                imu_stack.absolute_queue.push_front(measurediff);
                imu_stack_mutex.unlock();
                int slp_for = period*1000;
                std::this_thread::sleep_for (std::chrono::milliseconds(slp_for));
            }
            
        }
         
    }
}

void viso_thread(std::string csi_pipeline, std::string config_file)
{   
    cv::VideoCapture cap(csi_pipeline, cv::CAP_GSTREAMER);
    cv::FileStorage fs(config_file,cv::FileStorage::READ);
    cv::Mat intrinsics, distorsion; 
    if(fs.isOpened ())
    {
        fs["cameraMatrix"]>>intrinsics;
        fs["distCoeffs"]>>distorsion;
    }
    else
    {
        std::cout<<"Could not find any camera config file with the name  "<< config_file << std::endl;
        sigterm = true; //Exiting the infinite loop directly
    } 
        
    while(!sigterm)
    {
        ImageStamped img1;
        ImageStamped img2;
        if (!cap.read(img1.image)) {
		std::cout<<"Capture read error"<<std::endl;
		break;
	    }
        cap.read(img1.image);
        img1.t= time(NULL);
        cap.read(img2.image);
        img2.t= time(NULL);
        //Now undistord the image ; 
        cv::Mat dis_buff; 
        cv::undistort(img1.image,dis_buff,intrinsics,distorsion);
        img1.image = dis_buff;
        cv::undistort(img2.image,dis_buff,intrinsics,distorsion);
        img2.image = dis_buff;
        //An then we compute the essential matrix
        cv::Mat R, t, pts,E;
        pts = compute_transform_essential(img2.image,img1.image,intrinsics,R,t,E);


    }
    std::cout<<"Application recieved Sigterm"<<std::endl;
    
}

int main(int argc , char** argv)
{
    std::string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
    // Arguments list( in order) -config file for camera calib -others
    std::string config_file;
    std::cout<<"The number of arguments is "<<argc<<std::endl;
    if(argc ==1) std::cout<<"Unspecified config file or any other argument"<<std::endl;
    else
    {
        config_file = argv[1];
    }
    signal(SIGINT, signal_callback_handler); //to catch the sigterm if needed;
    std::thread t1(viso_thread,pipeline,config_file);
    std::thread t2(imu_tread,0.01);
    t1.join();
    t2.join(); 
    return 1;

}