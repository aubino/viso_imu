#include "imu.h"
#include <iostream>
#include "transform_computer.hpp"
#include "image.h"
#include <signal.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/filesystem.hpp>
#include <filesystem>
//#include <experimental/filesystem>
#include <vector>
#include <math.h>
#include<string>

#define PI 3.14159265359
#define G 9.81
#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 
//Directory navigation inside a file 

std::vector<std::string> get_all(std::filesystem::path const & root, std::string const & ext)
{
    std::vector<std::string> paths;
    for (auto p : std::filesystem::directory_iterator(root))
    {
        if (p.path().extension() == ext)
            paths.push_back(std::filesystem::canonical(p.path()).string());
    }
    return paths;
}


int main(int argc , char** argv)
{
    int rows = 3;
    int cols = 3;
    std::string right_root,left_root,config_file;
    int nbr_test;
    if(argc <5) 
    {   std::cout<<"Unspecified config file or any other argument"<<std::endl;
        return 0;

    }
    else
    {
        left_root = argv[1];
        right_root = argv[2];
        config_file = argv[3];
        nbr_test = std::stoi(argv[4]);
    }
    std::vector<std::string>  right_images_vector = get_all(right_root,".jpg");
    std::vector<std::string> left_images_vector = get_all(left_root,".jpg");
    /*for (auto path : right_images_vector)
    {
        std::cout<<"Right images path "<< path<<std::endl;
    }
        
    for (auto path :left_images_vector)
    {
        std::cout<<"Left images path "<< path<<std::endl;
    }*/
    std::cout<<  "Left images number "<<left_images_vector.size()<<std::endl; 
    std::cout<<  "Right images number "<<right_images_vector.size()<<std::endl; 
    if(nbr_test>left_images_vector.size() || nbr_test>right_images_vector.size())
    {
        std::cout<<"Number of test cases superior to datas provided. Will try to take all the datas instead"<<std::endl;
        nbr_test= std::min(left_images_vector.size(),right_images_vector.size());
    }
    //------------------------Loading file test -------------//
    cv::String cv_config_file(config_file);
    cv::FileStorage fs;
    fs.open(cv_config_file,cv::FileStorage::READ);
    cv::Mat left_intrinsics(rows, cols, cv::DataType<double>::type), 
            left_distorsion(rows, cols, cv::DataType<double>::type), 
            left_projection(rows, cols, cv::DataType<double>::type),
            right_intrinsics(rows, cols, cv::DataType<double>::type),
            right_distorsion(rows, cols, cv::DataType<double>::type), 
            right_projection(rows, cols, cv::DataType<double>::type);
    if(fs.isOpened ())
    {
        fs["K_101"]>>left_intrinsics;
        fs["D_101"]>>left_distorsion;
        fs["K_103"] >> right_intrinsics;
        fs["D_103"]>>right_distorsion;
    }
    std::cout<<"Using camera left camera parameters "<< left_intrinsics<<std::endl;
    std::cout<<"Using camera right camera parameters "<< right_distorsion<<std::endl;
    //----------------------------Loaded files ------------------------//
    for(int i =0; i<nbr_test; i++)
    {
        std::string left_image_path = cv::samples::findFile(left_images_vector[i]);
        std::string right_image_path = cv::samples::findFile(right_images_vector[i]);
        cv::Mat left_img = cv::imread(left_image_path, cv::IMREAD_COLOR);
        cv::Mat right_img = cv::imread(right_image_path, cv::IMREAD_COLOR);
        if(left_img.empty())
        {
            std::cout<<"Could not load the image "<<left_image_path<<std::endl;
            return 0;
        }
        if(right_img.empty())
        {
            std::cout<<"Could not load the image "<<left_image_path<<std::endl;
            return 0;
        }
        cv::imshow("Left Image DIsplay", left_img);
        cv::imshow("Right Image DIsplay", right_img);
        cv::Mat R, t, pts,E;
        //pts = compute_transform_essential(left_img,right_img,intrinsics,R,t,E);
        std::pair<std::vector<cv::KeyPoint>,cv::Mat> left_kp =  find_key_points_and_descriptors(left_img ,300,"left_kp");
        std::pair<std::vector<cv::KeyPoint>,cv::Mat> right_kp =  find_key_points_and_descriptors(right_img ,300,"right_kp");
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> matches =  match_images(left_img, right_img,300,0.15,"Match_window","BruteForce-Hamming");
        
        int k = cv::waitKey(0);
        if(k =='q')
            return 0 ;
        if(k =='p' && i>2)
        {
            i--; i--;
        }
            

    }

}
