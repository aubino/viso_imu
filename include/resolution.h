#pragma once
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