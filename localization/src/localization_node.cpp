

#include <iostream>



#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h> 
#include <pcl/point_cloud.h> 

#include <pcl/registration/icp.h>

#include <pcl/filters/voxel_grid.h>


#include "Eigen/Core"
#include "Eigen/Geometry"


int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle n;

    ros::Publisher pub_scene = n.advertise<sensor_msgs::PointCloud2>("points_scene", 1);
    ros::Publisher pub_map = n.advertise<sensor_msgs::PointCloud2>("points_map", 1);

    ros::Rate loop_rate(10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSceneF (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (/* specify file name here */, *cloudScene) == -1) {
        PCL_ERROR ("Couldn't read file scene.pcd \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMapF (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (/* specify file name here */, *cloudMap) == -1) {
        PCL_ERROR ("Couldn't read file map.pcd \n");
        return (-1);
    }


    // please adjust leaf size for better performance and result  :)
    double leafSize1 = 0.3;
    pcl::VoxelGrid<pcl::PointXYZ> sor1;
    sor1.setInputCloud (cloudScene);
    sor1.setLeafSize (leafSize1, leafSize1, leafSize1);
    sor1.filter (*cloudSceneF);

    double leafSize2 = 0.5;
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloudMap);
    sor2.setLeafSize (leafSize2, leafSize2, leafSize2);
    sor2.filter (*cloudMapF);
    

    std::cout << std::endl << "scene size : " << cloudScene->points.size() << std::endl;
    std::cout << "filtered scene size : " << cloudSceneF->points.size() << std::endl;
    std::cout << "map size : " << cloudMap->points.size() << std::endl;
    std::cout << "filtered map size : " << cloudMapF->points.size() << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;


    // Hint: http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
    /* write your code here :) */
















    /* write code here  :) */


    Eigen::Matrix4f trans = icp.getFinalTransformation();
    std::cout << trans << std::endl;

    
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    
    // trabsform from Eigen::Matrix4f to tf::Transform
    Eigen::Matrix4d md(trans.cast<double>());
    Eigen::Affine3d affine(md);
    tf::transformEigenToTF(affine, transform);
    

    sensor_msgs::PointCloud2 msg_scene;
    pcl::toROSMsg(*cloudSceneF, msg_scene);
    msg_scene.header.frame_id = "/scene";

    sensor_msgs::PointCloud2 msg_map;
    pcl::toROSMsg(*cloudMapF, msg_map);
    msg_map.header.frame_id = "/map";


   


    while (ros::ok())
    {
        
        pub_scene.publish(msg_scene);
        pub_map.publish(msg_map);
        
        // map : parent frame
        // scene : child frame
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/scene"));

        loop_rate.sleep();
    }


    return 0;
}