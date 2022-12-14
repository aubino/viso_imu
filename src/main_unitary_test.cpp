#include "imu.h"
#include <iostream>
#include "transform_computer.hpp"
#include "image.h"
#include <signal.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/filesystem.hpp>
#include <vector>

#define PI 3.14159265359
#define G 9.81
#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 

namespace fs = boost::filesystem;
//Directory navigation inside a file 

std::vector<fs::path> get_all(fs::path const & root, std::string const & ext)
{
    std::vector<fs::path> paths;

    if (fs::exists(root) && fs::is_directory(root))
    {
        for (auto const & entry : fs::recursive_directory_iterator(root))
        {
            if (fs::is_regular_file(entry) && entry.path().extension() == ext)
                paths.emplace_back(entry.path().filename());
        }
    }

    return paths;
} 


int main(int argc , char** argv)
{
    std::string right_root,left_root,config_file;
    if(argc <4) 
    {   std::cout<<"Unspecified config file or any other argument"<<std::endl;
        return 0; 
    }
    else
    {
        left_root = argv[1];
        right_root = argv[2];
        config_file = argv[3];
    }
    std::vector<fs::path> right_images_vector_ = get_all(right_root,"jpg");
    std::vector<fs::path> left_images_vector_ = get_all(left_root,"jpg");
    std::vector<std::string>  right_images_vector, left_images_vector; 
    for (auto path : right_images_vector_)
    {
        right_images_vector.push_back(fs::canonical(path).string());
        std::cout<<"Right images path "<< fs::canonical(path).string()<<std::endl;

    }
        
    for (auto path :left_images_vector_)
    {
        left_images_vector.push_back(fs::canonical(path).string());
        std::cout<<"Left images path "<< fs::canonical(path).string()<<std::endl;
    }
        

}