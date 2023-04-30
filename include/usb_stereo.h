#include <opencv4/opencv2/core.hpp>
#include <image.h>
#include <map>
#include <signal.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>
#include <math.h>


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
        
        bool operator <(const RESOLUTION& r1) const 
        {
            return sqrt(this->heigh*this->heigh + this->width*this->width) < sqrt(r1.heigh*r1.heigh + r1.width*r1.width);
        } 

} RESOLUTION ;

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

const std::map<RESOLUTION,float> RES_FPS_MAP({{RESOLUTION(10,20),10},{RESOLUTION(20,30),60}}); //TODO Take the good values from vendor and put theme here

int stereoUsbCaptureThread(int usb_channel,StereoImageRessource& ressource, bool verbose = false,bool debug = false);