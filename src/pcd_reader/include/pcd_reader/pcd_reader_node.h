#ifndef POINTCLOUD_PUBLISHER_NODE_H_
#define POINTCLOUD_PUBLISHER_NODE_H_

#include <ros/ros.h>
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

/**
 * This class reads a pointcloud from a .pcd file and publishes it to a sensor_msgs/PointCloud2 topic
 */
class PointcloudPublisherNode
{
    public:
        /**
         * Constructor
         *
         * @param nh
         * ROS NodeHandle object
         */
        PointcloudPublisherNode(ros::NodeHandle &nh);

        /**
         * Destructor
         */
        virtual ~PointcloudPublisherNode();


    private:
        /**
         * Reads .pcd file at pointcloud_file_path and publishes it to output PointCloud2 topic
         *
         * @param pointcloud_file_path
         * Full path to input .pcd file
         *
         */
        void publish_pointcloud(const bfs::path &pointcloud_file_path);

    private:
        /**
         * ROS NodeHandle object
         */
        ros::NodeHandle nh_;

        /**
         * PointCloud2 publisher
         */
        ros::Publisher pointcloud_publisher_;
};

#endif
