#include <pcd_reader/pcd_reader_node.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

PointcloudPublisherNode::PointcloudPublisherNode(ros::NodeHandle &nh) : nh_(nh)
{
    pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("output_pointcloud", 1);

    std::string input_file;

    if (!nh_.getParam("input_file", input_file))
    {
        ROS_ERROR("Input .pcd file needs to be specified");
    }
    else
    {
        publish_pointcloud(input_file);
    }
}

PointcloudPublisherNode::~PointcloudPublisherNode()
{
}

void PointcloudPublisherNode::publish_pointcloud(const bfs::path &pointcloud_file_path)
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

    if (pcl::io::loadPCDFile(pointcloud_file_path.string(), *cloud) == -1)
    {
        ROS_ERROR("Could not open %s", pointcloud_file_path.string().c_str());
        return;
    }

    double frames_per_second = 0.0;

    sensor_msgs::PointCloud2 ros_output_cloud;
    pcl_conversions::fromPCL(*cloud, ros_output_cloud);

    std::string frame_id;
    if (nh_.getParam("frame_id", frame_id))
    {
        ros_output_cloud.header.frame_id = frame_id;
    }

    if (!nh_.getParam("frames_per_second", frames_per_second))
    {
        ros_output_cloud.header.stamp = ros::Time::now();
        pointcloud_publisher_.publish(ros_output_cloud);
        ROS_INFO("Published pointcloud. Exiting");
        ros::shutdown();
        return;
    }

    ros::Rate loop_rate(frames_per_second);

    ROS_INFO("Going to publish pointcloud at %.2f fps", frames_per_second);
    while(ros::ok())
    {
        ros_output_cloud.header.stamp = ros::Time::now();
        pointcloud_publisher_.publish(ros_output_cloud);
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_reader");

    ros::NodeHandle n("~");

    ROS_INFO("[pointcloud_publisher] node started");

    PointcloudPublisherNode pointcloud_publisher_node(n);

    ros::spinOnce();

    return 0;
}
