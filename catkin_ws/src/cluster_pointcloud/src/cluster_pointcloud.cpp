#include <ros/ros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
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

#include "cluster_pointcloud/get_barrels.h"

#include <math.h>

#include <iostream>
#include <stdlib.h>


ros::Publisher pub;

ros::Publisher pub_pose;

geometry_msgs::PoseArray pose_list;



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
    ec.setClusterTolerance(0.18); // 6cm
    ec.setMinClusterSize (4); // 100
    //ec.setMaxClusterSize (25000); // 25000
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

        pose_list.poses.clear();


    float object_width;
    float distance = 10000000;
    float tem_distance = 0;





    float  x_pose = 0;
    float y_pose = 0;
    float z_pose = 0;

      std::vector<geometry_msgs::Pose> pose_vector;


      int pose_id = 0;


    for(int i = 0; i < clouds_list.size(); i++){

         distance = 10000000;

        pcl::PointCloud<pcl::PointXYZ>::Ptr tem_publish_cloud = clouds_list[i];

        float x1 = tem_publish_cloud->points[0].x;
        float y1 = tem_publish_cloud->points[0].y;
        float x2 = tem_publish_cloud->points[tem_publish_cloud->points.size() - 1].x;
        float y2 = tem_publish_cloud->points[tem_publish_cloud->points.size() - 1].y;

        object_width = sqrt(pow(( x1 - x2), 2) + pow((y1 - y2), 2));



       //if((object_width >= 0.04f) && (object_width <= 0.08f)){

        if(tem_publish_cloud->points.size() >= 7 && tem_publish_cloud->points.size() <= 30) {

            if(tem_publish_cloud->points.size() >= 13){
                if(!((object_width >= 0.04f) && (object_width <= 0.08f))){
                   continue;
                }
            }



            for(int n = 0; n < tem_publish_cloud->points.size(); n++){
                x_pose = tem_publish_cloud->points[n].x;
                y_pose = tem_publish_cloud->points[n].y;

                tem_distance = sqrt(pow(x_pose, 2) + pow(y_pose,2));

                if(tem_distance < distance){
                    distance = tem_distance;
                    pose_id = n;
                }


            }



            geometry_msgs::Pose pose;

            pose.position.x = tem_publish_cloud->points[pose_id].x;
            pose.position.y = tem_publish_cloud->points[pose_id].y;
            pose.position.z = 0;


           pose_vector.push_back(pose);



        }


    }

    if(pose_vector.size() == 0){
        ROS_INFO("No barrels found :(");
    } else{
        std::cout << "Found objects: " << pose_vector.size() << "\n";




      for(int i = 0; i < pose_vector.size(); i++){

          pose_list.poses.push_back( pose_vector[i]);

      }

         //pub_pose.publish(pose_list);

    }


}

bool get_barrel_list(cluster_pointcloud::get_barrels::Request  &req,
         cluster_pointcloud::get_barrels::Response &res)
{

    pose_list.header.frame_id = "/map";

  res.barrel_list = pose_list;

  return true;
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_pointcloud");
  ros::NodeHandle nh;

  //pub = nh.advertise<sensor_msgs::PointCloud2>("/barrel", 1);


  // pub_pose = nh.advertise<geometry_msgs::PoseArray>("/points_of_interest", 1);

   ros::ServiceServer service = nh.advertiseService("get_barrel_list", get_barrel_list);



  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/laser_cloud", 1, cloud_cb);




  // Spin
  ros::spin ();
}
