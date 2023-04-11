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
//#include <vector>
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
    //Arguments in order left images root, right images root,config file path,number of tests to perform.
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
            right_projection(rows, cols, cv::DataType<double>::type),
            stereo_trans(rows,cols,cv::DataType<double>::type), // translation matrix from left to right camera 
            stereo_rot(rows,cols,cv::DataType<double>::type); // rotation matrix from left to right camera
    if(fs.isOpened ())
    {
        fs["K_101"]>>left_intrinsics;
        fs["D_101"]>>left_distorsion;
        fs["K_103"] >> right_intrinsics;
        fs["D_103"]>>right_distorsion;
        fs["R_103"]>>stereo_rot;
        fs["T_103"]>>stereo_trans;
    }
     std::cout<<" -----------------------------------------------------------------------------------------------------------------------"<<std::endl;
      std::cout<<" -----------------------------------------------------------------------------------------------------------------------"<<std::endl;
    std::cout<<"Using camera left camera intrinsics :"<<std::endl<< left_intrinsics<<std::endl;
    std::cout<<"Using camera left camera distorsion :"<< std::endl<< left_distorsion<<std::endl;
    std::cout<<" -----------------------------------------------------------------------------------------------------------------------"<<std::endl;
     std::cout<<" -----------------------------------------------------------------------------------------------------------------------"<<std::endl;
    std::cout<<"Using camera right camera parameters :"<<std::endl<< right_intrinsics<<std::endl;
    std::cout<<"Using camera right camera distorsion :"<<std::endl<< right_distorsion<<std::endl;
     std::cout<<" -----------------------------------------------------------------------------------------------------------------------"<<std::endl;
    std::cout<<" -----------------------------------------------------------------------------------------------------------------------"<<std::endl;
    //----------------------------Loaded files ------------------------//
    for(int i =0; i<nbr_test; i++)
    {
        std::string left_image_path = cv::samples::findFile(left_images_vector[i]);
        std::string right_image_path = cv::samples::findFile(right_images_vector[i]);
        cv::Mat left_img = cv::imread(left_image_path, cv::IMREAD_COLOR);
        cv::Mat right_img = cv::imread(right_image_path, cv::IMREAD_COLOR);
        cv::Mat left_img_undistord ,right_img_undistord;
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
        cv::undistort(left_img,left_img_undistord,left_intrinsics,left_distorsion);
        cv::undistort(right_img,right_img_undistord,right_intrinsics,right_distorsion);
        #ifdef DEBUG
            std::pair<std::vector<cv::KeyPoint>,cv::Mat> left_kp =  find_key_points_and_descriptors(left_img_undistord ,300,"left_kp");
            std::pair<std::vector<cv::KeyPoint>,cv::Mat> right_kp =  find_key_points_and_descriptors(right_img_undistord ,300,"right_kp");
            std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> matches =  match_images(left_img_undistord, right_img_undistord,300,0.15,"Match_window","BruteForce-Hamming");
        #else 
            std::pair<std::vector<cv::KeyPoint>,cv::Mat> left_kp =  find_key_points_and_descriptors(left_img_undistord ,300);
            std::pair<std::vector<cv::KeyPoint>,cv::Mat> right_kp =  find_key_points_and_descriptors(right_img_undistord ,300);
            std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> matches =  match_images(left_img_undistord, right_img_undistord,300,0.15,"BruteForce-Hamming"); 

        #endif
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
        //cv::Mat mask;
        //cv::Mat E =  cv::findEssentialMat(matches.first, matches.second,right_intrinsics,cv::RANSAC,0.999, 3,mask);
        //std::cout<<"Essential is "<<std::endl<<E<<std::endl;
        pts = compute_transform_essential(left_img_undistord,right_img_undistord,right_intrinsics,R,t,E);
        std::cout<< "Translation vector is "<<std::endl<<t<<std::endl;
        std::cout<<"Rotation matrix is "<<std::endl<<R<<std::endl;
        Eigen::Vector3d Translation; 
        Translation<< t.at<double>(0,0),t.at<double>(1,0),t.at<double>(3,0); 
        Eigen::Vector3d StereoTranslation; 
        StereoTranslation<< stereo_trans.at<double>(0,0),stereo_trans.at<double>(1,0),stereo_trans.at<double>(3,0);
        StereoTranslation = (1/StereoTranslation.norm())*StereoTranslation;
        Eigen::Matrix3d Rotation; 
        Rotation<< R.at<double>(0,0),R.at<double>(0,1), R.at<double>(0,2),
                                    R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
                                    R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
        Eigen::Matrix3d StereoRotation; 
        StereoRotation<< stereo_rot.at<double>(0,0),stereo_rot.at<double>(0,1), stereo_rot.at<double>(0,2),
                                    stereo_rot.at<double>(1,0),stereo_rot.at<double>(1,1),stereo_rot.at<double>(1,2),
                                    stereo_rot.at<double>(2,0),stereo_rot.at<double>(2,1),stereo_rot.at<double>(2,2);
        if(!Translation.norm())
            Translation=(1/Translation.norm())*Translation;
        std::cout<<"The direction error after reconstrunction is "<< (Translation.cross(StereoTranslation)).norm()<<" "<<std::endl;
        auto rotation_err = -Rotation.eulerAngles(0, 1, 2)+StereoRotation.eulerAngles(0,1,2);
        std::cout<<"Rotation matrix error in angles [Unit = degees] "<< rotation_err[0]*180/PI<<" on X axis ,"<< rotation_err[1]*180/PI<<" on Y axis ,"
                    << rotation_err[2]*180/PI<<" on Z axis "<<std::endl; 
        int k = cv::waitKey(0);
        if(k =='q')
            return 0 ;
        if(k =='p' && i>2)
        {
            i--; i--;
        }
            

    }

}
