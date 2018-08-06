#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


// #include "../driver/cloud_frame.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "example");
    ros::NodeHandle node;
    ros::Publisher pub_cloud = 
        node.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    sensor_msgs::PointCloud2 cloud_msgs;

    // frame f("../lidar_forward","../VLP16db.yaml");
    ros::Rate loop_rate(1);
    while(ros::ok()){
        pcl::PointCloud<pcl::PointXYZI> pcloud;
        pcloud.resize(1000);

        // velodyne_rawdata::vcloud cloud;
        // f.get_frame(cloud);
        float temp =0.0f;
        for(auto p:pcloud){
            pcl::PointXYZI tp;
            tp.x = temp;
            tp.y = temp;
            tp.z = temp;
            tp.intensity = 255;
            pcloud.push_back(tp);
            temp+=0.02;
        }
        std::cout<<"frame.size: "<<pcloud.size()<<std::endl;
        pcl::toROSMsg(pcloud, cloud_msgs);
        cloud_msgs.header.stamp = ros::Time::now();
        cloud_msgs.header.frame_id = "velodyne";
        pub_cloud.publish(cloud_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}