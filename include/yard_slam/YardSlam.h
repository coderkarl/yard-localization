// Copyright 2022 coderkarl. Subject to the BSD license.

#ifndef YardSlam_H
#define YardSlam_H

#include <mutex>
//ROS Includes
#include <rclcpp/rclcpp.hpp>
#include <my_interfaces/msg/odom_inputs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "yard_slam/EKFPose.h"

class YardSlam: public rclcpp::Node
{
  public:
    YardSlam();
    ~YardSlam();

    //returns parameter rate_ in hz
    // used to define ros::Rate
    double get_tf_rate();
    void update_odom_tf();
    void update_map_tf(const rclcpp::Time& tf_time);
    void send_map_tf(const rclcpp::Time& tf_time);
    void update_periodic();

  private:
    void odomInputsCallback(const my_interfaces::msg::OdomInputs::SharedPtr odomInputs);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::QoS qos_;

    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tree_map_pub_;

    rclcpp::Subscription<my_interfaces::msg::OdomInputs>::SharedPtr odomInputs_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan> > scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

    std::shared_ptr<tf2_ros::TransformListener> listener;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    tf2::Transform map_to_odom, odom_to_base;
    std::mutex map_to_odom_mutex;
    rclcpp::Duration transform_timeout;

    rclcpp::Time time_scan;
    rclcpp::Time time_odom;
    rclcpp::Time time_accum_meas;

    //parameters
    std::string param_topic_pointcloud_in;
    std::string param_topic_pointcloud_out;
    double min_range_;
    double max_range_;
    double tf_rate_;
    double meas_reset_time_sec_;

    std::vector<double> x_trees_, y_trees_;
    double tree_dist_thresh_;
    pcl::PointCloud<pcl::PointXYZ> tree_map_pc;

    yard::EKFPose ekf_pose;
    yard::ObsList map_obs_list;
};

#endif
