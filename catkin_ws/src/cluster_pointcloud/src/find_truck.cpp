#include <ros/ros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "cluster_pointcloud/get_truck.h"

#include "tf/transform_listener.h"

#include <math.h>

#include <iostream>
#include <stdlib.h>


geometry_msgs::PoseArray truck_pose;

ros::Publisher pub;




void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{


    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);



    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud); //cloud_filtered

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.08); // 6cm
    ec.setMinClusterSize (10); // 100
    //ec.setMaxClusterSize (600); // 25000
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud); //cloud_filtered
    ec.extract (cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  clouds_list  ;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]); // cloud_filtered
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clouds_list.push_back(cloud_cluster);


      j++;
    }



    float object_width;


    int truck_counter = 0;


   static  tf::TransformListener listener;



      geometry_msgs::PoseStamped begin_truck_pose;

      begin_truck_pose.header.frame_id = "/base_link";

      begin_truck_pose.pose.orientation.w = 1.0;

      geometry_msgs::PoseStamped end_truck_pose;

      end_truck_pose.header.frame_id = "/base_link";

      end_truck_pose.pose.orientation.w = 1.0;

      geometry_msgs::PoseStamped map_begin_truck_pose;

      map_begin_truck_pose.header.frame_id = "/map";

      map_begin_truck_pose.pose.orientation.w = 1.0;

      geometry_msgs::PoseStamped map_end_truck_pose;

      map_end_truck_pose.header.frame_id = "/map";

      map_end_truck_pose.pose.orientation.w = 1.0;

     int index;


    for(int i = 0; i < clouds_list.size(); i++){

        pcl::PointCloud<pcl::PointXYZ>::Ptr tem_publish_cloud = clouds_list[i];

        float x1 = tem_publish_cloud->points[0].x;
        float y1 = tem_publish_cloud->points[0].y;
        float x2 = tem_publish_cloud->points[tem_publish_cloud->points.size() - 1].x;
        float y2 = tem_publish_cloud->points[tem_publish_cloud->points.size() - 1].y;

        object_width = sqrt(pow(( x1 - x2), 2) + pow((y1 - y2), 2));


        float m = (y2 - y1) / (x1 - x2);

        float b = y1 - (m * x1);

        if((object_width >= 0.83f) && (object_width <= 0.86f)){ // 0.18 breit 0.84 lang

             truck_counter = truck_counter + 1;


              begin_truck_pose.pose.position.x = x1;
              begin_truck_pose.pose.position.y = y1;

              end_truck_pose.pose.position.x = x2;
               end_truck_pose.pose.position.y = y2;

               index = i;
                }
         }

    if(truck_counter == 1){


        listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(3.0));
        listener.transformPose("/map", begin_truck_pose, map_begin_truck_pose);
        listener.transformPose("/map", end_truck_pose, map_end_truck_pose);

        truck_pose.poses.clear();
        truck_pose.poses.push_back(map_begin_truck_pose.pose);
        truck_pose.poses.push_back(map_end_truck_pose.pose);

        sensor_msgs::PointCloud2 pub_cloud;

        pcl::toROSMsg(*clouds_list[index], pub_cloud);

        pub_cloud.header.frame_id = "/base_link";

        pub.publish(pub_cloud);

        }

}







bool get_truck(cluster_pointcloud::get_truck::Request  &req,
         cluster_pointcloud::get_truck::Response &res)
{

    truck_pose.header.frame_id = "/map";

    if(truck_pose.poses.size() == 0){
        return false;
    }else{
        res.truck_pose = truck_pose ;
        return true;
    }

}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "find_truck");
  ros::NodeHandle nh;



     ros::ServiceServer service = nh.advertiseService("get_truck", get_truck);



  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/laser_cloud", 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/truck", 1);




  // Spin
  ros::spin ();
}
