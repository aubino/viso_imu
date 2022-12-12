#include "imu.h"
#include "transform_computer.hpp"
#include "image.h"
#include <signal.h>
#include <thread>
#include <mutex>
#include <chrono>
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

std::string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
// Sigterm callback handler 
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   sigterm = 1;
   exit(signum);
}
// Imu stack datas as mutex ressources
std::shared_ptr<Imu> imu_ptr; 
ImuQueue imu_stack(imu_ptr,0.1,10000);
std::mutex imu_stack_mutex;

void imu_tread(doudle period)
{
    /*Set up the imu sdk if necessary*/
    imu_stack.sampling_period = period; 
    while(!sigterm)
    {
        Imu measure /*= get raw measurements from sdk*/ ;
         
    }
}

void viso_thread(std::string csi_pipeline, std::string config_file)
{   
    cv::VideoCapture cap(csi_pipeline, cv::CAP_GSTREAMER);
    cv::FileStorage fs(config_file,FileStorage::READ);
    cv::Mat intrinsics, distorsion; 
    if(fs.isOpened ())
    {
        intrinsics = fs["cameraMatrix"];
        distorsion = fs["distCoeffs"];
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
    // Arguments list( in order) -config file for camera calib -others  
    signal(SIGINT, signal_callback_handler); //to catch the sigterm if needed;
}