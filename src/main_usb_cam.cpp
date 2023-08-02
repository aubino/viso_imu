#include "usb_stereo.h"
#include <boost/program_options.hpp>
#include <memory>
#include <thread>
#include "cloud_computer.hpp"

namespace po = boost::program_options;

int main(int argc,char** argv)
{
    po::options_description desc("Mandatory options");
    int channel;
    bool debug = false;
    bool verbose = false;
    std::string left_path  ; 
    std::string right_path ;
    StereoImageRessource stereo_ressource ; 
    RGBDRessource rgbd_ressource ;
    desc.add_options ()
    ("help,h","This executable launches the usb stereo camera driver  ")
    ("channel,c",boost::program_options::value<int>(),"The usb channel of the stereo camera. Mandatory option")
    ("left_params,l",boost::program_options::value<std::string>(),"The parameters file of the left camera obtained from calibration. Absolute path . Mandatory option")
    ("right_params,r",boost::program_options::value<std::string>(),"The parameters file of the right camera obtained from calibration . Absolute path . Mandatory option")
    ("debug,D",boost::program_options::value<bool>(),"Whether to show debug options and logs or not. Optional argument")
    ("verbose,v",boost::program_options::value<bool>(),"Allow verbose logs or not . Optional argument") ; 

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if(vm.count("help,h") || !vm.count("channel"))
    {
        std::cout<<desc ;
        return 0 ;
    }
    else
    {
        channel = vm["channel"].as<int>();
        if(vm.count("debug"))
            debug = vm["debug"].as<bool>();
        if(vm.count("verbose"))
            verbose = vm["verbose"].as<bool>();
        if(vm.count("left_params")) 
            left_path = vm["left_params"].as<std::string>() ; 
        if(vm.count("right_params")) 
            right_path = vm["right_params"].as<std::string>() ;

    }
    stereo_ressource = std::make_shared<StereoImage>(left_path,right_path);
    rgbd_ressource = std::make_shared<RGBD_ImageStamped>() ; 

    std::cout<<"===================================================="<<std::endl;
    std::cout<<"|               Used arguments summary              |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Videoport Number    |             "<<channel<<"             |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Left frame params   |         "<<left_path<<"           |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Right frame params  |         "<<right_path<<"           |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Debug Option        |               "<<debug<<"           |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Verbose Option      |               "<<verbose<<"           |"<<std::endl;
    std::cout<<"===================================================="<<std::endl;
    // std::thread stereo_aquisition(stereoUsbCaptureThread,channel,stereo_ressource,undistord,verbose,debug) ; 
    // std::thread cloud_computer(stereoCloudComputingThread,stereo_ressource,rgbd_ressource,debug,verbose) ; 
    // stereo_aquisition.join() ; 
    // cloud_computer.join() ; 
    // return EXIT_SUCCESS ; 
    stereoUsbCaptureThread(channel,stereo_ressource,verbose,debug) ; 
    return EXIT_SUCCESS  ;
}
