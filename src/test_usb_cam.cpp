#pragma once 
#include <opencv4/opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <signal.h>

bool signal_trigered = false ; 

void trigger_sigterm(int signum)
{
    signal_trigered = true ; 
    cv::destroyAllWindows();
    return ; 
}





int main(int argc , char ** argv)
{
    signal(SIGTERM,trigger_sigterm) ; 
    signal(SIGINT,trigger_sigterm) ; 
    if(argc <2)
    {
        std::cout<<"Give device parameters! "  ; 
        return -1 ;
    }
        
    std::string device(argv[1]) ;
    cv::VideoCapture cap ; 
    if(! cap.open(device,cv::CAP_ANY))
    {
        std::cout<<"OPening of device "<<device <<" failed\n" ;
        return -1 ;
    } 
        
    cv::Mat frame ; 
    cap.set(cv::CAP_PROP_FRAME_WIDTH,2560) ; 
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,720) ; 
    while(!signal_trigered)
    {
        cap.read(frame) ; 
        cv::imshow("Camera Frame" , frame) ; 
        cv::waitKey(5) ; 
        std::cout<<"Frame Width "<< frame.size().width <<" Frame Height : "<<frame.size().height <<std::endl ; 
    }
    std::cout<<"Sigterm invoqued. Frame displaying will end" <<std::endl ; 
    return 0 ; 
}