#include <opencv4/opencv2/core.hpp>
#include <image.h>
#include <map>
#include <signal.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

class StereoImage
{
    public : 
        std::mutex mutex;
        RESOLUTION resolution;
        StereoImage(RESOLUTION res);
        std::pair<ImageStamped,ImageStamped> getImages();
        ImageStamped getLeft();
        ImageStamped getRight();
        bool setLeft(const ImageStamped&);
        bool setRight(const ImageStamped&);
    private :
        ImageStamped left;
        ImageStamped right;
};

using StereoImageRessource = std::unique_ptr<StereoImage> ;

typedef struct RESOLUTION
{
    public : 
        int heigh= 0,width = 0;
        RESOLUTION()
        {
            heigh = 240 ;
            width = 380 ;
        }
        RESOLUTION(int w,int h)
        {
            heigh = h;
            width = w;
        };
} RESOLUTION ;

const std::map<RESOLUTION,float> RES_FPS_MAP({{RESOLUTION(10,20),10},{RESOLUTION(20,30),60}}); //TODO Take the good values from vendor and put theme here

int stereoUsbCaptureThread(int usb_channel,StereoImageRessource ressource, bool verbose = false,bool debug = false);