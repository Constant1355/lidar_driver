#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "driver/cloud_frame.hpp"

int main(){
    int argc;
    char** argv;
    ros::init(argc, argv, "example");
    ros::NodeHandle node;
    ros::Publisher pub_cloud = 
        node.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    sensor_msgs::PointCloud2 cloud_msgs;
    

    frame f("lidar_forward","VLP16db.yaml");

    ros::Rate loop_rate(100);
    while(ros::ok()){
        pcl::PointCloud<pcl::PointXYZI> pcloud;
        velodyne_rawdata::vcloud cloud;
        f.get_frame(cloud);
        std::cout<<"size: "<<std::dec<<cloud.size()<<std::endl;

        for(auto p: cloud){
            pcl::PointXYZI tp;
            tp.x = p.x;
            tp.y = p.y;
            tp.z = p.z;
            tp.intensity = p.i;
            pcloud.push_back(tp);
        }
    
        pcl::toROSMsg(pcloud, cloud_msgs);
        cloud_msgs.header.stamp = ros::Time::now();
        cloud_msgs.header.frame_id = "velodyne";
        pub_cloud.publish(cloud_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}