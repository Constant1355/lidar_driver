#pragma once

#include <vector>

namespace velodyne_rawdata
{

    typedef struct vlp_point{
        float x;
        float y;
        float z;
        float i;
        float r;
    } vpoint;

    typedef std::vector<vpoint> vcloud;

} //velodyne_rawdata