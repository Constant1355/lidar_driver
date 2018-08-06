#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../driver/cloud_frame.hpp"

int main(){
    frame f("../lidar_forward","../VLP16db.yaml");
    for(;;){
        pcl::PointCloud<pcl::PointXYZI> pcloud;
        velodyne_rawdata::vcloud cloud;
        f.get_frame(cloud);
        
        for(auto p:cloud){
            pcl::PointXYZI tp;
            tp.x = p.x;
            tp.y = p.y;
            tp.z = p.z;
            tp.intensity = p.i;
            pcloud.push_back(tp);
        }
        std::cout<<"frame.size: "<<pcloud.size()<<std::endl;
    }
    return 1;
}