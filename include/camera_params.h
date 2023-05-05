#pragma once 
#include <opencv4/opencv2/core.hpp>
#include "resolution.h"

class CameraParams
{
    private : 
        std::string file ;
        cv::Mat intrinsics ;
        cv::Mat distorsion ;
        cv::Mat projection_matrix ;
    public : 
        RESOLUTION resolution ;
        CameraParams(std::string from_file = "")
        {
            file = from_file ;
        };

        bool load_from(std::string config_file)
        {
            try 
            {
                cv::FileStorage fs(config_file,cv::FileStorage::READ);
                if(fs.isOpened())
                {
                    fs["camera_matrix"]>>intrinsics;
                    fs["distortion_coefficients"]>>distorsion;
                    fs["projection_matrix"] >> projection_matrix ;
                    resolution = RESOLUTION(fs["image_width"],fs["image_height"]);
                    return true ;
                }
                return false ;
            }
            catch (const std::out_of_range exception)
            {
                return false ;
            }
        };
        
        bool load()
        {
            return load_from(file);
        };
};
