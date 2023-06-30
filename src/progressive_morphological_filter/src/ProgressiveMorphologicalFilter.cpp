//
// Created by tevhit on 30.06.2023.
//

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

using namespace std;

class ProgressiveMorphologicalFilter
{
public:
    ProgressiveMorphologicalFilter ()
    {
        sub_ = nh_.subscribe ("/velodyne_points", 1,  &ProgressiveMorphologicalFilter::cloudCallback, this);

        pub_point_cloud_pmf = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_pmf", 1);
    }

private:
    // Callback
    void cloudCallback(const pcl::PCLPointCloud2::ConstPtr& callback)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*callback, *cloud);

        std::cout << "Cloud Row Data:" << std::endl;
        std::cerr << *cloud << std::endl;
        std::cout << "----------------------------" << std::endl;

        // Create the filtering object
        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud(cloud);
        pmf.setMaxWindowSize(20);
        pmf.setSlope(1.0f);
        pmf.setInitialDistance(0.5f);
        pmf.setMaxDistance(3.0f);
        pcl::PointIndicesPtr ground (new pcl::PointIndices);
        pmf.extract(ground->indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Extrach non-ground data
        extract.setInputCloud(cloud);
        extract.setIndices(ground);
        extract.setNegative(true);
        extract.filter(*cloud_filtered);

        std::cout << "Cloud PMF Filtered Data:" << std::endl;
        std::cerr << *cloud_filtered << std::endl;
        std::cout << "----------------------------" << std::endl;

        pub_point_cloud_pmf.publish(*cloud_filtered);
        ros::Rate loop_rate(10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_point_cloud_pmf;
};

int main(int argc, char** argv)
{
    ros::init (argc, argv, "Progressive_Morphological_Filter", ros::init_options::AnonymousName);

    ProgressiveMorphologicalFilter progressiveMorphologicalFilter;
    ros::spin();

    return (0);
}