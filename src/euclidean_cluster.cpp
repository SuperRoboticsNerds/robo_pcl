

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <ras_msgs/Cam_transform.h>
#include <ras_msgs/pcl_object.h>
#include <ras_msgs/pcl_object_mult.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <math.h>


#define LOOP_RATE 10

class EuclideanCluster{
public:
	ros::NodeHandle n;
	ros::Publisher clusters_pub;	
	ros::Subscriber  sub_filter_cloud;
	int min_cluster_size = 10;
	int max_cluster_size = 20000;
	int cluster_tolerance = 1;


	EuclideanCluster(){
		// sub_filter_cloud = n.subscribe("/pcl/final_filtered_cloud", 1, &EuclideanCluster::cluster_Cb, this);
		sub_filter_cloud = n.subscribe("/pcl/clouds_no_walls", 1, &EuclideanCluster::cluster_Cb, this);
		clusters_pub = n.advertise<ras_msgs::pcl_object_mult>("/pcl/clusters", 1);
		// object_pub

	}



	void cluster_Cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::fromROSMsg(*cloud_msg, *cloud);

		// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		// tree->setInputCloud(cloud);


        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        ROS_INFO("hiiii  %d", cloud->points.size());

        // Use Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (cluster_tolerance);
        ec.setMinClusterSize (min_cluster_size);
        ec.setMaxClusterSize (max_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

  		ROS_INFO("no of clusters is: %d", cluster_indices.size());
  		ras_msgs::pcl_object_mult cluster_mult;
  		int count = 0;


  		for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
  			++count;	
  			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

  			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
				cloud_cluster->points.push_back (cloud->points[*pit]); //*
  			}

  			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			Eigen::Vector4f centroid; 
			pcl::compute3DCentroid(*cloud_cluster,centroid);
  			ras_msgs::pcl_object cluster;

			cluster.x = centroid[0];
			cluster.y = centroid[1];
			cluster.z = -centroid[2];

			ROS_INFO("cloud centroid is : %d", cluster.x);

			cluster.isdebri = 0;

			ROS_INFO("cluster centroid coordinates are : %d, %d, %d", centroid[0], centroid[1], centroid[2]);

			if(cluster.y >= 0.04){
				cluster.isdebri = 1;
			}

			cluster_mult.vector_objs.push_back(cluster);

  		}

  		cluster_mult.num_objects = count;
  		clusters_pub.publish(cluster_mult);
	}

};



int main(int argc, char ** argv){

	ros::init(argc, argv, "clusterpcl");
	// ros::NodeHandle node("~");
	EuclideanCluster  ec_Node;

	ros::Rate loop_rate(LOOP_RATE);

	while(ec_Node.n.ok()){
		ros::spinOnce();
    	loop_rate.sleep();

	}

	ros::spin();
	return 0;

}


