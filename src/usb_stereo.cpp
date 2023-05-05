#include "usb_stereo.h"

void signal_callback_handler(int signum) 
    {
        std::cout << "Caught signal  " << signum << " Inside the stereoUsbCaptureThread thread. Exciting now  "<< std::endl;
        // Terminate program
        std::terminate() ;
    };

StereoImage::StereoImage(RESOLUTION res)
{
    resolution = res;
    return;
}

StereoImage::StereoImage(CameraParams lp, CameraParams rp)
{
    left_params = lp;
    right_params = rp;
    resolution = lp.resolution ;
}

std::pair<ImageStamped,ImageStamped> StereoImage::getImages()
{
    if(mutex.try_lock())
    {
        mutex.unlock();
        return std::make_pair(left,right);
    }
    else
        return std::make_pair(ImageStamped(),ImageStamped());
}

ImageStamped StereoImage::getLeft()
{
    if(mutex.try_lock())
    {
        mutex.unlock();
        return left;
    }
    else
        return ImageStamped();
}

ImageStamped StereoImage::getRight()
{
    if(mutex.try_lock())
    {
        mutex.unlock();
        return right;
    }
    else
        return ImageStamped();
}

bool StereoImage::setLeft(const ImageStamped& image)
{
    if(mutex.try_lock())
    {
        left = image;
        mutex.unlock();
        return true;
    }
    return false ;  
}

bool StereoImage::setRight(const ImageStamped& image)
{
    if(mutex.try_lock())
    {
        right = image;
        mutex.unlock();
        return true;
    }
    return false ;  
}


int stereoUsbCaptureThread(int usb_channel,StereoImageRessource& ressource, bool verbose ,bool debug)
{
    signal(SIGINT,signal_callback_handler);
    signal(SIGTERM,signal_callback_handler);
    float fps = 0;
    cv::VideoCapture capture_interface(usb_channel);
    try 
    {
        fps = RES_FPS_MAP.at(ressource->resolution) ;
    }   
    
    catch (const std::out_of_range exception)
    {
        if(verbose)
            std::cout<<"The resolution for is not registered in the vendor database. Falling back to 60 fps with resolution "<< ressource->resolution.width <<" x "<< ressource->resolution.heigh<<std::endl;
        fps = 60;
    }

    capture_interface.set(cv::CAP_PROP_FPS,fps);
    capture_interface.set(cv::CAP_PROP_FRAME_WIDTH,ressource->resolution.width);
    capture_interface.set(cv::CAP_PROP_FRAME_HEIGHT,ressource->resolution.heigh);
    while(true)
    {
        cv::Mat captured_image;
        if(capture_interface.read(captured_image))
        {
            if(debug)
                cv::imshow("Stereo camera raw output",captured_image);
            cv::Mat half_left(captured_image.size().height,int(captured_image.size().width/2),CV_8UC3),
            half_right(captured_image.size().height,int(captured_image.size().width/2),CV_8UC3); 
            
            for(int i =0; i<captured_image.size().width/2 ; i++ )
            {
                for(int j = 0; j<captured_image.size().height ; j++)
                    half_left.at<uchar>(i,j) = captured_image.at<uchar>(i,j); 
            }

            for(int i(captured_image.size().width/2) ; i< captured_image.size().width; i++)
            {
                for(int j = 0; j<captured_image.size().height ; j++)
                    half_right.at<uchar>(i,j) = captured_image.at<uchar>(i,j) ;
            }

            if(ressource->setLeft(ImageStamped(time(NULL),half_left)) && ressource->setRight(ImageStamped(time(NULL),half_right)))
            {
                if(debug)
                    std::cout<<"The images have been transfered and saved to mutex ressource"<<std::endl;
            }
        
            else
            {
                if(verbose)
                    std::cerr<<"The transfering of image between aquisition and mutex failed. The mutex must have been occupied "<<std::endl;
            }
        
        }
        
        else
            if(debug)   
                std::cout<<"The Image could not be red. Tips : check the connection port of hardware connection"<<std::endl;
        cv::waitKey(5);
    }
    capture_interface.release();
    cv::destroyAllWindows();
    return 0 ;
}
