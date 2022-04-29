/* add by tuligen 2022.4.28 */
#ifndef _LFUNCTIONS_H_H_
#define _LFUNCTIONS_H_H_

#include <math.h>
#include <string>
#define EARTH_RADIUS 6378137
#define PI           3.141592657

#define D2R(X)      ((X) * 0.0174532925388889)

/*全静态函数工具类*/
class Functions final
{
public:
    
    inline static double mector_lon(double lon)
    {
           return D2R(lon) * EARTH_RADIUS;
    }

    inline static double mector_lat(double lat)
    {
        auto temp = log(tan((90.0 + lat) * PI / 360.0) ) /(PI/180.0);

        return D2R(temp) * EARTH_RADIUS;
    }

};


#endif