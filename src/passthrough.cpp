

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>


#include <pcl/filters/extract_indices.h>
#include <ras_msgs/Cam_transform.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <math.h>


#define POINT_THRESHOLD 2000
#define LOOP_RATE 10


class PassThroughFilter{

public:
	ros::NodeHandle n;
	ros::Publisher pub_cloud1;
	ros::Publisher pub_cloud2;
	ros::Publisher pub_cloud3;


	ros::Subscriber sub_cloud;
	const double x_min = -0.4;
	const double x_max = 0.4;
	const double y_min = -0.2;
	const double y_max = 0.4;
	const double z_min = -0.3;
	const double z_max = 0.4;


	PassThroughFilter(){
		n = ros::NodeHandle("~");
		sub_cloud = n.subscribe("/pcl/plane_transformed", 1, &PassThroughFilter::ptf_cb, this);
		pub_cloud1 = n.advertise<sensor_msgs::PointCloud2>("/pcl/clouds_all", 1);
		pub_cloud2 = n.advertise<sensor_msgs::PointCloud2>("/pcl/clouds_no_walls", 1);
		pub_cloud3 = n.advertise<sensor_msgs::PointCloud2>("/pcl/final_filtered_cloud", 1);

		// pub_filter = n.advertise<ras_msgs::>
	}




	void  ptf_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_wall(new pcl::PointCloud<pcl::PointXYZ>);
		// the aim is to filter out the wall planes from the image ....

		pcl::fromROSMsg(*cloud_msg, *cloud);

		if(cloud->width > 0){
			pcl::PassThrough<pcl::PointXYZ> pass_x;
			// removing nans and unimportant points.

			pass_x.setInputCloud(cloud);
			pass_x.setFilterFieldName("x");
			pass_x.setFilterLimits(x_min, x_max);
			pass_x.filter(*cloud_filtered_x);

			pcl::PassThrough<pcl::PointXYZ> pass_y;
			pass_y.setInputCloud(cloud_filtered_x);
			pass_y.setFilterFieldName("y");
			pass_y.setFilterLimits(y_min, y_max);
			pass_y.filter(*cloud_filtered_xy);

			pcl::PassThrough<pcl::PointXYZ> pass_z;
			pass_z.setInputCloud(cloud_filtered_xy);
			pass_z.setFilterFieldName("z");
			pass_z.setFilterLimits(z_min, z_max);
			pass_z.filter(*cloud_filtered_xyz);

			cloud_filtered_temp = cloud_filtered_xyz;


			if(cloud_filtered_xyz->width > 0){
				pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
				pcl::SACSegmentation<pcl::PointXYZ> seg;

				seg.setOptimizeCoefficients(true);
				seg.setModelType (pcl::SACMODEL_PLANE);
	  			
	  			seg.setMethodType (pcl::SAC_RANSAC);
	  			seg.setMaxIterations(1000);
	  			seg.setDistanceThreshold(0.01);


	  			seg.setInputCloud(cloud_filtered_temp);
	  			seg.segment(*inliers, *coefficients);

	  			int num_points = (int) cloud_filtered_temp->points.size();

				while(inliers->indices.size() > POINT_THRESHOLD || cloud_filtered_temp->points.size() > 0.5 *num_points ){
	  				pcl::ExtractIndices<pcl::PointXYZ> extract;
					extract.setInputCloud(cloud_filtered_temp);
					extract.setIndices (inliers);
					extract.setNegative(true);
					extract.filter(*cloud_filtered_temp);

					pcl::SACSegmentation<pcl::PointXYZ> seg1;
					seg1.setOptimizeCoefficients(true);
					seg1.setModelType (pcl::SACMODEL_PLANE);
	  			
	  				seg1.setMethodType (pcl::SAC_RANSAC);
	  				seg1.setMaxIterations(1000);
	  				seg1.setDistanceThreshold(0.01);

	  				seg1.setInputCloud(cloud_filtered_temp);
	  				seg1.segment(*inliers, *coefficients);

				}


				// seperate the walls 
				pcl::PassThrough<pcl::PointXYZ> pass_wall;
				pass_wall.setInputCloud(cloud_filtered_temp);
				pass_wall.setFilterFieldName("y");
				pass_wall.setFilterLimits(0.1, 0.35);
				pass_wall.filter(*cloud_filtered_wall);

			}

		}

	}

};






int main(int argc, char ** argv){

	ros::init(argc, argv, "filterpcl");
	// ros::NodeHandle node("~");
	PassThroughFilter  ptf_Node;

	ros::Rate loop_rate(LOOP_RATE);

	while(ptf_Node.n.ok()){
		ros::spinOnce();
    	loop_rate.sleep();
	}

	ros::spin();
	return 0;

}
