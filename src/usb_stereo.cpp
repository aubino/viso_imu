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
    left_params.resolution.width = res.width/2 ; 
    left_params.resolution.heigh = res.heigh ;
    right_params = left_params ; 
    return;
}

StereoImage::StereoImage(CameraParams lp, CameraParams rp)
{
    left_params = lp;
    right_params = rp;
    resolution = RESOLUTION(lp.resolution.width*2,lp.resolution.heigh) ; 
}

StereoImage::StereoImage(std::string left_file, std::string right_file)
{
    left_params = CameraParams(left_file) ; 
    right_params = CameraParams(right_file) ;
    left_params.load() ; 
    right_params.load() ; 
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

bool StereoImage::rectifyImages(std::pair<ImageStamped,ImageStamped>& rectified_image)
{
    if( left_params.getRectificationMatrix().size().width > 0  &&  
        left_params.getRectificationMatrix().size().height > 0 &&
        right_params.getRectificationMatrix().size().width > 0 &&
        right_params.getRectificationMatrix().size().height > 0)
    {
        cv::remap(left.image,rectified_image.first.image,left_params.rectification_map_x,left_params.rectification_map_y,cv::INTER_LINEAR , cv::BORDER_DEFAULT) ;
        cv::remap(right.image,rectified_image.second.image,right_params.rectification_map_x,right_params.rectification_map_y,cv::INTER_LINEAR , cv::BORDER_DEFAULT) ;
        rectified_image.first.t = left.t ; 
        rectified_image.second.t = right.t ; 
        return true ; 
    }
    return false ; 
}



int stereoUsbCaptureThread(int usb_channel,StereoImageRessource ressource, bool undistord,bool verbose ,bool debug)
{
    signal(SIGINT,signal_callback_handler);
    signal(SIGTERM,signal_callback_handler);
    float fps = 0;
    std::stringstream ss ;
    ss<< "/dev/video";
    ss<< usb_channel;
    cv::VideoCapture capture_interface(ss.str(),cv::CAP_ANY);
    
    try 
    {
        fps = RES_FPS_MAP.at(ressource->resolution) ;
    }   
    
    catch (std::out_of_range const& exception)
    {
        ressource->resolution = RESOLUTION(640,240) ; 
        if(verbose)
            std::cout<<"The  resolution you aked for  is not registered in the vendor database. Falling back to 60 fps with resolution "<< ressource->resolution.width <<" x "<< ressource->resolution.heigh<<std::endl;
        fps = 60;
    }

    // capture_interface.set(cv::CAP_PROP_FPS,fps);
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
                    half_left.at<uchar>(j,i) = captured_image.at<uchar>(j,i); 
            }

            for(int i(captured_image.size().width/2) ; i< captured_image.size().width; i++)
            {
                for(int j = 0; j<captured_image.size().height ; j++)
                    half_right.at<uchar>(j, int(i-captured_image.size().width/2)) = captured_image.at<uchar>(j,i) ;
            }

            // if(undistord)
            // {
                // cv::Mat distorsion_buffer ; 
                // cv::undistort(half_left,distorsion_buffer,ressource->left_params.getCameraMatrix(),ressource->left_params.getDistorsion());
                // half_left = distorsion_buffer ;
                // cv::undistort(half_right,distorsion_buffer,ressource->right_params.getCameraMatrix(),ressource->right_params.getDistorsion());
                // half_right = distorsion_buffer ;
            // }

            if(debug)
            {
                cv::imshow("Stereo camera left Image",ressource->getLeft().image);
                cv::imshow("Stereo camera right Image",ressource->getRight().image);
            }

            if(ressource->setLeft(ImageStamped(time(0),half_left)) && ressource->setRight(ImageStamped(time(0),half_right)))
            {
                if(verbose)
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
