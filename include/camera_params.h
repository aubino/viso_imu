#ifndef CAMERA_PARAMS_H
#define CAMERA_PARAMS_H
#include <opencv4/opencv2/core.hpp>
#include "resolution.h"

class CameraParams
{
    private : 
        std::string file ;
        cv::Mat intrinsics ;
        cv::Mat distorsion ;
        cv::Mat projection_matrix ;
        cv::Mat rectification_matrix ; 
    public : 
        RESOLUTION resolution ;
        cv::Mat rectification_map_x , rectification_map_y ; 
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
                    try
                    {
                        fs["rectification_matrix"] >> rectification_matrix ;
                        cv::initUndistortRectifyMap(intrinsics,
                                                    distorsion,
                                                    rectification_matrix,
                                                    projection_matrix,
                                                    cv::Size(resolution.width,resolution.heigh),
                                                    CV_16SC2,
                                                    rectification_map_x,
                                                    rectification_map_y) ; 
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                    resolution = RESOLUTION(int(fs["image_width"]),int(fs["image_height"]));
                    return true ;
                }
                return false ;
            }
            catch (std::out_of_range const& exception)
            {
                return false ;
            }
        };
        
        bool load()
        {
            return load_from(file);
        };

        cv::Mat getCameraMatrix()
        {
            return intrinsics;
        };
        
        cv::Mat getDistorsion()
        {
            return distorsion;
        };

        cv::Mat getProjectionMatrix()
        {
            return projection_matrix;
        };
        cv::Mat getRectificationMatrix()
        {
            return rectification_matrix ; 
        }
};

#endif