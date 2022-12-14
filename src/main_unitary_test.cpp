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
#include <vector>

#define PI 3.14159265359
#define G 9.81
#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 

namespace fs = boost::filesystem;
//Directory navigation inside a file 

std::vector<std::string> get_all(std::filesystem::path const & root, std::string const & ext)
{
    std::vector<std::string> paths;
    for (auto p : fs::recursive_directory_iterator(root))
    {
        if (p.path().extension() == ext)
            paths.push_back(std::canonical(p.path()).string());
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
    std::vector<std::string>  right_images_vector = get_all(right_root,"jpg");
    std::vector<std::string> left_images_vector = get_all(left_root,"jpg");
    for (auto path : right_images_vector)
    {
        right_images_vector.push_back(fs::canonical(path).string());
        std::cout<<"Right images path "<< path<<std::endl;

    }
        
    for (auto path :left_images_vector)
    {
        left_images_vector.push_back(fs::canonical(path).string());
        std::cout<<"Left images path "<< path<<std::endl;
    }
        

}