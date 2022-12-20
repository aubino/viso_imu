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
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}
int capture_width = 1280 ;
int capture_height = 720 ;
int display_width = 1280 ;
int display_height = 720 ;
int framerate = 10 ;
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
    std::cout<<"Initiating the IMU calibration sequence. Please make no movement "<<std::endl;
    double initial_accel[3] ={0,0,0};
    double initial_ypr[3] = {0,0,0};
    for(int i=0; i<100; i++)
    {
        imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        initial_accel[0]+=stAccelRawData.fX*G;
        initial_accel[1]+=stAccelRawData.fY*G;
        initial_accel[2]+=stAccelRawData.fZ*G;
        initial_ypr[0]+= stAngles.fRoll*PI/180;
        initial_ypr[1]+=stAngles.fPitch*PI/180;
        initial_ypr[2]+=stAngles.fYaw*PI/180;
        std::cout<<"..";
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    std::cout<<std::endl;
    initial_accel[0]= initial_accel[0]/100;
    initial_accel[1]= initial_accel[1]/100;
    initial_accel[2]= initial_accel[2]/100;
    initial_ypr[0] = initial_ypr[0]/100;
    initial_ypr[1] = initial_ypr[1] /100;
    initial_ypr[2] = initial_ypr[2]/100;
    //register the zero motion data as gravity
    imu_stack.gravity0<< initial_accel[0],initial_accel[1],initial_accel[2];
    imu_stack.OriginTransform = Eigen::Translation<double,3>(0,0,0)*Eigen::AngleAxisd(initial_ypr[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(initial_ypr[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(initial_ypr[2], Eigen::Vector3d::UnitZ());
    std::cout<<" End of calibration . \n Estimated value of G at time zero is : \n " << imu_stack.gravity0<<std::endl;
    std::cout<<"Transform at Time 0 is : \n "<<imu_stack.OriginTransform.linear()<<std::endl;
    //End of seting up imu sdk
    imu_stack.sampling_period = period; 
    while(!sigterm)
    {
        imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        time_t t = time(NULL);
        std::cout<<" Raw aceleration Data "<<stAccelRawData.fX<<" "<<stAccelRawData.fY<<" "<<stAccelRawData.fZ<<std::endl;
        if(imu_stack_mutex.try_lock())
        {
            if(imu_stack.absolute_queue.size()==0)
            { //first transfrom to stack. 
                Imu measure("imuFrame");
                measure.stamp_from= t;
                //compute the new value of G.
                Eigen::Vector3d gravity_t  = Eigen::AngleAxisd(stAngles.fRoll, Eigen::Vector3d::UnitX())
                                            * Eigen::AngleAxisd(stAngles.fPitch, Eigen::Vector3d::UnitY())
                                            * Eigen::AngleAxisd(stAngles.fYaw, Eigen::Vector3d::UnitZ()) 
                                            * imu_stack.OriginTransform.linear().inverse() * imu_stack.gravity0;
                //Now remove the gravity from measures
                stAccelRawData.fX = stAccelRawData.fX - gravity_t[0]/G;
                stAccelRawData.fY = stAccelRawData.fY - gravity_t[1]/G;
                stAccelRawData.fZ = stAccelRawData.fZ - gravity_t[2]/G;
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
                std::cout<<"Transform from time to time is : "<<std::endl;
                std::cout<<"Rotation part    : "<<difftrans.linear()<<std::endl;
                std::cout<<"Translation part : "<<NEWP<<std::endl;
                std::this_thread::sleep_for (std::chrono::milliseconds(slp_for));
            }
            
        }
         
    }
}

void viso_thread(std::string config_file)
{   
    cv::FileStorage fs(config_file,cv::FileStorage::READ);
    cv::Mat intrinsics, distorsion; 
    if(fs.isOpened ())
    {
        fs["camera_matrix"]>>intrinsics;
        fs["distortion_coefficients"]>>distorsion;
        fs["image_width"]>>capture_width;
        fs["image_height"]>>capture_height;
        display_height = capture_height;
        display_width = capture_width;
        std::cout<<"Using Camra paramaters :"<<std::endl;
        std::cout<<"\t Intrinsics : "<<intrinsics<<std::endl;
        std::cout<<"\t Distorsion : "<<distorsion<<std::endl;
        std::cout<<"\t Image width : "<<capture_width<<std::endl;
        std::cout<<"\t Image Height : "<<capture_height<<std::endl;
        
        
    }
    else
    {
        std::cout<<"Could not find any camera config file with the name  "<< config_file << std::endl;
        sigterm = true; //Exiting the infinite loop directly
    }
    std::string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    std::cout<<"Waiting for 5 s"<<std::endl;        
    while(!sigterm)
    {
        std::cout<<"Viso thread entered infinite loop routine "<<std::endl;
        ImageStamped img1;
        ImageStamped img2;
        if (!cap.read(img1.image)) {
		std::cout<<"Capture read error"<<std::endl;
		break;
	    }
        else img1.t= time(NULL);
        if (!cap.read(img2.image)) {
		std::cout<<"Capture read error"<<std::endl;
		break;
	    }
        else img2.t= time(NULL);
        //show the images 
        
        //Now undistord the image ; 
        cv::Mat dis_buff; 
        cv::undistort(img1.image,dis_buff,intrinsics,distorsion);
        img1.image = dis_buff;
        cv::undistort(img2.image,dis_buff,intrinsics,distorsion);
        img2.image = dis_buff;
        cv::Mat R, t,pts,E;
        int recap_attempts = 0;
        while(!compute_transform(img2.image,img1.image,intrinsics,R,t,E))
        {
            std::cout<<"Not enough disparity for transform computation. Waiting for appropriate second image ...."<<std::endl;
            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (!cap.read(img2.image)) 
            {//retake second image if there is not enough disparity repeat the process for 200 atempts and switch image2 to the img1 
		        std::cout<<"Capture read error"<<std::endl;
                sigterm = true;
		        break;
	        }
            else 
            {
                cv::Mat other_buff;
                img2.t= time(NULL);
                cv::undistort(img2.image,other_buff,intrinsics,distorsion);
                img2.image = other_buff;
                cv::imshow("image_1",img1.image);
                cv::imshow("image_2",img2.image);
                recap_attempts++;
                int keycode = cv::waitKey(10) ;
                if (keycode == 'q') break ;
                
            }
            if(recap_attempts == 199) 
            {
                recap_attempts =0;
                img1.image = img2.image;
            }

        }
        #ifdef DEBUG
            std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> matches =  match_images(img2.image, img2.image,300,0.15,"Match_window","BruteForce-Hamming");
        #endif
        pts = compute_transform_essential(img2.image,img1.image,intrinsics,R,t,E);
        std::cout<<" Rendering results"<<std::endl;
        std::cout<<"Translation direction :  \t "<<t<<std::endl;
        std::cout<<"Rotation estimate : \t "<<R<<std::endl;
        std::cout<<"Considered inliers 3D coordinates : \t"<<pts<<std::endl;
        cv::imshow("image_1",img1.image);
        cv::imshow("image_2",img2.image);
        int keycode = cv::waitKey(10) ; 
        if (keycode == 'q') break ;

    }
    cap.release();
    std::cout<<"Application recieved Sigterm"<<std::endl;
    
}

int main(int argc , char** argv)
{
    // Arguments list( in order) -config file for camera calib -others
    std::string config_file;
    std::cout<<"The number of arguments is "<<argc<<std::endl;
    if(argc ==1) std::cout<<"Unspecified config file or any other argument"<<std::endl;
    else
    {
        config_file = argv[1];
    }
    signal(SIGINT, signal_callback_handler); //to catch the sigterm if needed;
    //std::thread t1(viso_thread,pipeline,config_file);
    //std::thread t2(imu_tread,0.01);
    //t1.join();
    //t2.join();
    //viso_thread(config_file); 
    imu_tread(0.01);
    return 1;

}
