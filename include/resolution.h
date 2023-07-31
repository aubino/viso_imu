#pragma once
#include <math.h>

typedef struct RESOLUTION
{
    public : 
        uint heigh = 0,width = 0;
        RESOLUTION()
        {
            heigh = 480 ;
            width = 640 ;
        }
        
        RESOLUTION(uint w,uint h)
        {
            heigh = h;
            width = w;
        };
        
        bool operator <(const RESOLUTION& r1) const 
        {
            return this->heigh*this->width  <  r1.heigh*r1.width ;
        } 

} RESOLUTION ;