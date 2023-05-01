#include "usb_stereo.h"
#include <boost/program_options.hpp>
#include <memory>
namespace po = boost::program_options;

int main(int argc,char** argv)
{
    po::options_description desc("Mandatory options");
    u_int channel;
    bool debug = false;
    bool verbose = false;
    desc.add_options ()
    ("help,h","This node is used to send cartesian commands to the robot arm. Allowed options are : ")
    ("channel,c",boost::program_options::value<int>(),"The topic on which the program will find camera infos. Mandatory option");
    ("debug,D",boost::program_options::value<bool>(),"Whether to show debug options and logs or not. Optional argument");
    ("verbose,v",boost::program_options::value<bool>(),"Allow verbose logs or not . Optional argument");
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

    }
    
    StereoImageRessource ressource = std::make_unique<StereoImage>(RESOLUTION(240,380));
    stereoUsbCaptureThread(0,ressource,verbose,debug);
    return 1;

}