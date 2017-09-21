#ifndef POINTCLOUD_WRITER_NODE_H_
#define POINTCLOUD_WRITER_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * This class subscribes to a sensor_msgs/PointCloud2 topic and writes the first received pointcloud to file
 */
class PointcloudWriterNode
{
    public:
        /**
         * Constructor
         *
         * @param nh
         * ROS NodeHandle object
         */
        PointcloudWriterNode(ros::NodeHandle &nh);

        /**
         * Destructor
         */
        virtual ~PointcloudWriterNode();

        /**
         * Callback for pointcloud subscriber
         *
         * @param msg
         * sensor_msgs PointCloud2 received from topic
         */
        void pointcloudCallback(const sensor_msgs::PointCloud2Ptr &image);


    private:
        /**
         * ROS NodeHandle object
         */
        ros::NodeHandle nh_;

        /**
         * Pointcloud subscriber
         */
        ros::Subscriber pointcloud_subscriber_;

        /**
         * Full path to output file
         */
        std::string output_file_name_;
};

#endif
