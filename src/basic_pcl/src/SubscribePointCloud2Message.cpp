//
// Created by tevhit on 27.06.2023.
//

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

class SubscribePointCloud2Message
{
public:
    SubscribePointCloud2Message()
    {
        // velodyne_points  -> VLP-16
        // velodyne_points2 -> HDL-32E
        sub_ = nh_.subscribe ("/velodyne_points2", 1,  &SubscribePointCloud2Message::cloudCallback, this);
    }

private:
    // Callback
    void cloudCallback(const pcl::PCLPointCloud2::ConstPtr& callback)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*callback, *cloud);

        std::cout << "Real Time Point Cloud Data:" << std::endl;
        std::cerr << *cloud << std::endl;
        std::cout << "Point Cloud data has: " << cloud->points.size () << " data points." << std::endl;
        std::cout << "----------------------------" << std::endl;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init (argc, argv, "Subscribe_PointCloud2_Message", ros::init_options::AnonymousName);

    SubscribePointCloud2Message subscribePointCloud2Message;
    ros::spin();

    return (0);
}