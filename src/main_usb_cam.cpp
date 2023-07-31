#include "usb_stereo.h"
#include <boost/program_options.hpp>
#include <memory>
namespace po = boost::program_options;

int main(int argc,char** argv)
{
    po::options_description desc("Mandatory options");
    int channel;
    int width = 640;
    int heigh = 240;
    bool debug = false;
    bool verbose = false;
    bool undistord = false ; 
    desc.add_options ()
    ("help,h","This executable launches the usb stereo camera driver  ")
    ("channel,c",boost::program_options::value<int>(),"The topic on which the program will find camera infos. Mandatory option")
    ("undistord,u",boost::program_options::value<bool>(),"Whether to undistord the raw image or not. Optional argument")
    ("debug,D",boost::program_options::value<bool>(),"Whether to show debug options and logs or not. Optional argument")
    ("verbose,v",boost::program_options::value<bool>(),"Allow verbose logs or not . Optional argument")
    ("Width,W",boost::program_options::value<int>(),"width of the image . Optional argument")
    ("Heigh,H",boost::program_options::value<int>(),"Heigh of the image . Optional argument");
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
        if(vm.count("Width"))
            width = vm["Width"].as<int>();
        if(vm.count("Heigh"))
            heigh = vm["Heigh"].as<int>();

    }
    std::cout<<"===================================================="<<std::endl;
    std::cout<<"|               Used arguments summary              |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Videoport Number    |             "<<channel<<"             |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Frame Width         |             "<<width<<"           |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Frame Heigh         |             "<<heigh<<"           |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Undistord Option    |               "<<undistord<<"           |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Debug Option        |               "<<debug<<"           |"<<std::endl;
    std::cout<<"|---------------------------------------------------|"<<std::endl;
    std::cout<<"|   Verbose Option      |               "<<verbose<<"           |"<<std::endl;
    std::cout<<"===================================================="<<std::endl;
    StereoImageRessource ressource = std::make_shared<StereoImage>(RESOLUTION(width,heigh));
    return stereoUsbCaptureThread(channel,ressource,undistord,verbose,debug);
}