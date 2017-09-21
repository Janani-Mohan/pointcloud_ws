#include <pcd_writer/pcd_writer_node.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

PointcloudWriterNode::PointcloudWriterNode(ros::NodeHandle &nh) : nh_(nh)
{
    std::string input_topic;
    if (!nh_.getParam("input_topic", input_topic))
    {
        ROS_ERROR("Input topic needs to be specified");
        exit(0);
    }

    if (!nh_.getParam("output_file", output_file_name_))
    {
        ROS_ERROR("Output file needs to be specified");
        exit(0);
    }

    pointcloud_subscriber_ = nh_.subscribe(input_topic, 0, &PointcloudWriterNode::pointcloudCallback, this);
}

PointcloudWriterNode::~PointcloudWriterNode()
{
}

void PointcloudWriterNode::pointcloudCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*msg, *pcl_input_cloud);
    pcl::io::savePCDFile(output_file_name_, *pcl_input_cloud);
    ROS_INFO("[pointcloud_writer] wrote to %s", output_file_name_.c_str());
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_writer");

    ros::NodeHandle n("~");

    ROS_INFO("[pcd_writer] node started");

    PointcloudWriterNode pointcloud_writer_node(n);

    ros::spin();

    return 0;
}
