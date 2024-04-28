
/*****************************************************************************************
This demo node is for the application running on the topic "/pointcloud"
Before running this node, you should make sure that the topic "/pointcloud" is published coorectly. 

----------------------------------------------MOST INMPORTANTLY!!!---------------------------------
you should check the configuration of your LiDAR sensor, which should be registered in the ./include/projection_params.h file. 

Then, you can run this node as follows in your terminal:
```bash
source {the path of your catkin workspace}/devel/setup.bash
roscore
roslaunch dipgseg demo.launch
```
Also, this node would publish the ground and non-ground pointclouds to the topic "/ground" and "/non_ground" respectively.
Thus, you can visualize the results in rviz by importing the rviz config file in DipG-Seg/rviz/reprj.rviz.
*****************************************************************************************/

// #include "time_utils.h"
#include "kitti_loader.h"
#include "evaluate.h"
#include "dipgseg.h"

#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace KittiLoader;

int verbose = true;

class SegmentationNode : public rclcpp::Node {
public:
  SegmentationNode(const rclcpp::NodeOptions &node_options);
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<DIPGSEG::Dipgseg> dipgseg_;
};


SegmentationNode::SegmentationNode(const rclcpp::NodeOptions &node_options)
    : Node("ground_segmentation", node_options) {

  dipgseg_ = std::make_shared<DIPGSEG::Dipgseg>();

  std::string ground_topic, obstacle_topic, input_topic;
  ground_topic = this->declare_parameter("ground_output_topic", "ground_cloud");
  obstacle_topic = this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
  input_topic = this->declare_parameter("input_topic", "/livox/lidar_pcd2");

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1));
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic, rclcpp::SensorDataQoS());
  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "Segmentation node initialized");
}

void SegmentationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);

  pcl::PointCloud<pcl::PointXYZI> ground_cloud, obstacle_cloud;
    
  dipgseg_->segment_ground(cloud, ground_cloud, obstacle_cloud);
  if(verbose){
    double time_all = dipgseg_->get_whole_time();
    double time_seg = dipgseg_->get_seg_time();
    printf("-------------time_all: %f, time_seg: %f\n", time_all, time_seg);
    printf("cloud size: %ld, ground_cloud size: %ld, obstacle_cloud size: %ld\n", cloud.size(), ground_cloud.size(), obstacle_cloud.size());
  }
  auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto obstacle_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(ground_cloud, *ground_msg);
  pcl::toROSMsg(obstacle_cloud, *obstacle_msg);
  ground_msg->header = msg->header;
  obstacle_msg->header = msg->header;
  ground_pub_->publish(*ground_msg);
  obstacle_pub_->publish(*obstacle_msg);
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SegmentationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}