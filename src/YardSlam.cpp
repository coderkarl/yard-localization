// Copyright 2024 coderkarl. Subject to the BSD license.

#define BOOST_BIND_NO_PLACEHOLDERS

#include "yard_slam/ObsList.h"
#include "yard_slam/YardSlam.h"
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.h> // tf2::getYaw(orientation or quat)

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

//Constructor
YardSlam::YardSlam() :
    Node("avoid_obs"),
    qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
		transform_timeout(rclcpp::Duration(0.5 * 1000000000)),
		ekf_pose(3, 2, 2)
{
  // https://github.com/ros-planning/navigation2/blob/foxy-devel/nav2_amcl/src/amcl_node.cpp
  tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
  tfBuffer->setCreateTimerInterface(timer_interface);
  listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

	tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  declare_parameter<std::string>("topic_pointcloud_in","/cloud");
  declare_parameter<std::string>("topic_pointcloud_out", "/keypoints");

  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();

  //Topic you want to subscribe
	odomInputs_sub_ = create_subscription<my_interfaces::msg::OdomInputs>("odomInputs", 10,
			std::bind(&YardSlam::odomInputsCallback, this, _1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(),
			std::bind(&YardSlam::scanCallback, this, _1)); //receive laser scan
  pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10,
			std::bind(&YardSlam::pcCallback, this, _1));
  pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out, 2);
	tree_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("tree_pc", 2);

	meas_reset_time_sec_ = declare_parameter("meas_reset_time_sec", 1.0);

  tf_rate_ = declare_parameter("tf_rate_hz", 1.0);
  min_range_ = declare_parameter("min_range", 0.15);
  max_range_ = declare_parameter("max_range", 25.0);

  x_trees_ = declare_parameter("x_trees", std::vector<double>({5.0, 0.0}));
  y_trees_ = declare_parameter("y_trees", std::vector<double>({5.0, 0.0}));
  tree_dist_thresh_ = declare_parameter("tree_dist_tresh", 1.0);

	pcl::PointXYZ pt;
	size_t num_trees = x_trees_.size();
	for (size_t k = 0; k < num_trees; ++k) {
	    pt.x = x_trees_[k];
			pt.y = y_trees_[k];
			pt.z = 0;
			tree_map_pc.push_back(pt);
	}

  time_scan = now();
	time_accum_meas = now();

  m_timer = create_wall_timer(std::chrono::milliseconds((int)(1000./tf_rate_) ),
            std::bind(&YardSlam::update_periodic, this) );


	double init_x = declare_parameter("init_map_x", 0.0);
	double init_y = declare_parameter("init_map_y", 0.0);
	double init_yaw_deg = declare_parameter("init_yaw_deg", 0.0);
	double init_x_sigma = declare_parameter("init_x_sigma", 1.0);
	double init_y_sigma = declare_parameter("init_y_sigma", 1.0);
	double init_yaw_sigma_deg = declare_parameter("init_yaw_sigma_deg", 5.0);
	double enc_sigma = declare_parameter("enc_sigma", 0.1);
	double yaw_rate_sigma = declare_parameter("yaw_rate_sigma_rad", 0.03);
	double meas_sigma = declare_parameter("meas_sigma", 0.5);
  size_t num_states = 3;
  size_t num_meas = 2;
  Eigen::VectorXd initState(num_states);
  Eigen::MatrixXd initP(num_states, num_states);
  Eigen::MatrixXd inputNoiseQ(2, 2);
  Eigen::MatrixXd measNoiseR(num_meas, num_meas);
	initState << init_x, init_y, init_yaw_deg * M_PI / 180.0;
	double init_yaw_sigma = init_yaw_sigma_deg*M_PI/180.0;
	initP << init_x_sigma*init_x_sigma, 0, 0,   0, init_y_sigma*init_y_sigma, 0,   0, 0, init_yaw_sigma*init_yaw_sigma;
	inputNoiseQ << enc_sigma*enc_sigma, 0,   0, yaw_rate_sigma*yaw_rate_sigma;
	measNoiseR << meas_sigma*meas_sigma, 0,  0, meas_sigma*meas_sigma;
  ekf_pose.init(initState, initP, inputNoiseQ, measNoiseR);

	tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., ekf_pose.yaw_rad());
	map_to_odom.setRotation(q);
	map_to_odom.setOrigin(tf2::Vector3(init_x, init_y, 0));

  RCLCPP_INFO(get_logger(), "Starting Yard Slam");
}

YardSlam::~YardSlam(){}

void YardSlam::update_odom_tf() {
	try{
		geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped = tfBuffer->lookupTransform("odom", "base_link", rclcpp::Time(0) );
		//tf2::fromMsg(tf_stamped, odom_to_base);
		const geometry_msgs::msg::Vector3& v = tf_stamped.transform.translation;
		const geometry_msgs::msg::Quaternion& q_geom = tf_stamped.transform.rotation;
		tf2::Quaternion q(q_geom.x, q_geom.y, q_geom.z, q_geom.w);
		odom_to_base.setRotation(q);
		odom_to_base.setOrigin(tf2::Vector3(v.x, v.y, v.z));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "YardSlam: %s",ex.what());
  }
}

// To be called after the measurement update
void YardSlam::update_map_tf(const rclcpp::Time& tf_time)
{
	//std::unique_lock<std::mutex> lock(map_to_odom_mutex);
  // map_to_odom = map_to_base * inv(odom_to_base)
	tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., ekf_pose.yaw_rad());
	tf2::Transform map_to_base(q, tf2::Vector3(ekf_pose.x(), ekf_pose.y(), 0.0));
	map_to_odom = map_to_base * (odom_to_base.inverse());
	RCLCPP_INFO(get_logger(), "update map to odom. ekf x: %.2f, y: %.2f, yaw_deg %.2f\n",
	                 ekf_pose.x(), ekf_pose.y(), ekf_pose.yaw_rad()*180./M_PI);
	send_map_tf(tf_time);
	return;
}

void YardSlam::send_map_tf(const rclcpp::Time& tf_time) {
	//std::unique_lock<std::mutex> lock(map_to_odom_mutex);
	geometry_msgs::msg::TransformStamped msg;
	msg.transform = tf2::toMsg(map_to_odom);
	msg.child_frame_id = "odom";
	msg.header.frame_id = "map";
	msg.header.stamp = tf_time; // + transform_timeout; //this->now() + transform_timeout;
	std::cout << "SEND IT\n";
	
	tfBroadcaster->sendTransform(msg);
	//RCLCPP_INFO(get_logger(), "map tf time: %.2f", tf_time.nanoseconds()*1.e-9);
	return;
}

void YardSlam::update_periodic() {
	return;
}

void YardSlam::odomInputsCallback(const my_interfaces::msg::OdomInputs::SharedPtr odomInputs) {
	std::unique_lock<std::mutex> lock(map_to_odom_mutex);
	update_odom_tf();
	tf2::Transform map_to_base = map_to_odom * odom_to_base;

	tf2::Matrix3x3 rot_mat(map_to_base.getRotation());
	double roll, pitch, yaw;
	rot_mat.getRPY(roll, pitch, yaw);

	const tf2::Vector3& trans = map_to_base.getOrigin();
	const size_t n = 3;
	Eigen::VectorXd new_state(n);
	new_state << trans.x(), trans.y(), yaw;

	double dist = (odomInputs->enc_left + odomInputs->enc_right)/2;
	double dt = 0.05; //(odomInputs->header.stamp - time_odom).nanoseconds()*1e-9;
	time_odom = odomInputs->header.stamp;
	std::cout << "ekf update\n";
	ekf_pose.update(new_state, dist, dt);
	std::cout << "send map tf\n";

	RCLCPP_INFO(get_logger(), "odomInputs. ekf x: %.2f, y: %.2f, yaw_deg %.2f\n",
	                 ekf_pose.x(), ekf_pose.y(), ekf_pose.yaw_rad()*180./M_PI);
	send_map_tf(time_odom);
}

void YardSlam::pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "PC Callback %s", msg->header.frame_id);
}

// See https://github.com/ros-perception/pointcloud_to_laserscan/blob/rolling/src/laserscan_to_pointcloud_node.cpp
void YardSlam::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) //use a point cloud instead, use laser2pc.launch
{
	std::unique_lock<std::mutex> lock(map_to_odom_mutex);
  time_scan = scan->header.stamp;

  // https://answers.ros.org/question/273205/transfer-a-pointxyz-between-frames/
  geometry_msgs::msg::TransformStamped odom_to_laser_geom, base_to_laser_geom;
  try{
    //transformStamped = tfBuffer->lookupTransform("laser", "odom", scan->header.stamp, rclcpp::Duration(0.05));
    odom_to_laser_geom = tfBuffer->lookupTransform("odom", "laser", rclcpp::Time(0) );
		//base_to_laser_geom = tfBuffer->lookupTransform("base_link", "laser", rclcpp::Time(0) );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "YardSlam scan: %s",ex.what());
    return;
  }

	tf2::Transform odom_to_laser;
	tf2::fromMsg(odom_to_laser_geom.transform, odom_to_laser);
	// const geometry_msgs::msg::Vector3& v = odom_to_laser_geom.transform.translation;
	// const geometry_msgs::msg::Quaternion& q_geom = odom_to_laser_geom.transform.rotation;
	// tf2::Quaternion q(q_geom.x, q_geom.y, q_geom.z, q_geom.w);
	// odom_to_laser.setRotation(q);
	// odom_to_laser.setOrigin(tf2::Vector3(v.x, v.y, v.z));
	tf2::Transform map_to_laser = map_to_odom * odom_to_laser;
	
	geometry_msgs::msg::TransformStamped map_to_laser_geom;
	map_to_laser_geom.header.frame_id = "map";
	map_to_laser_geom.child_frame_id = "laser";
	map_to_laser_geom.header.stamp = time_scan;
	map_to_laser_geom.transform = tf2::toMsg(map_to_laser);

	geometry_msgs::msg::PointStamped laser_point, base_point, map_point;
	laser_point.header.frame_id = "laser";
	laser_point.header.stamp = scan->header.stamp;//ros::Time();
	laser_point.point.z = 0;
	
	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::PointXYZ pt;

	size_t num_trees = x_trees_.size();

	std::vector<pcl::PointXYZ> obs_list;
	obs_list.resize(num_trees);

	size_t num_rays = scan->ranges.size();
	for (size_t i = 0; i < num_rays;i++)
	{
	  float range = scan->ranges[i];
	  if(range < min_range_ || range > max_range_ || not std::isfinite(range))
	    continue;

	  float angle  = scan->angle_min +(float(i) * scan->angle_increment);

	  //	    if(fabs(angle) < 60*M_PI/180.0 && range > 3.0)
	  //	    {
	  //	      continue;
	  //	    }

	  laser_point.point.x = range*cos(angle) ;
	  laser_point.point.y = range*sin(angle) ;
	  int count = 0;
	  bool good_tf = true;
	  while(count < 3)
	  {
	    ++count;
	    try{
	      //listener.transformPoint("odom", laser_point, odom_point);
	      //tfBuffer->transform(laser_point, odom_point, "odom", tf2::durationFromSec(1.0));
	      tf2::doTransform(laser_point, map_point, map_to_laser_geom);
				//tf2::doTransform(laser_point, base_point, base_to_laser_geom);
	      good_tf = true; // Without this, must have good tf on first try
	      break;
	    }
	    catch(tf2::TransformException& ex){
	      good_tf = false;
	      RCLCPP_ERROR(get_logger(), "YardSlam Received an exception trying to transform a point : %s", ex.what());
	    }
	  }

	  if (!good_tf) {break;}

	  // Got a map point
	  for (size_t k = 0; k < num_trees; ++k) {
	    double dx  = map_point.point.x - x_trees_[k];
	    double dy = map_point.point.y - y_trees_[k];
	    double dist_sqd = dx*dx + dy*dy;
	    if (dist_sqd < tree_dist_thresh_ * tree_dist_thresh_) {
	      // pt.x = map_point.point.x;
	      // pt.y = map_point.point.y;
	      // pt.z = map_point.point.z;
	      // pc.push_back(pt);
				map_obs_list.add_point(k, map_point.point.x, map_point.point.y);

				// measurement
				// ekf_pose.updateExpectedMeasure(x_trees_[k], y_trees_[k]);
				// ekf_pose.applyMeasure(base_point.point.x, base_point.point.y);
				// update_map_tf(time_scan);
	    }
	  }

	} // loop thru scan
	
	// Accumulated measurement
	size_t min_obs_points = 3;
	size_t num_valid_obs = 0;
	for (auto iter = map_obs_list.obs_map.begin(); iter != map_obs_list.obs_map.end(); ++iter) {
		const yard::ObsStats& os = iter->second;
		if (os.n >= min_obs_points) {
			++num_valid_obs;
		}
		pt.x = os.meanPoint.x;
		pt.y = os.meanPoint.y;
		pt.z = 0;
		pc.push_back(pt);
	}

	if (num_valid_obs >= 2) {
		geometry_msgs::msg::TransformStamped base_to_map_geom;

		// May need to use ekf pose to get base to map = inv(map to base)
		try{
			base_to_map_geom = tfBuffer->lookupTransform("base_link", "map", rclcpp::Time(0) );
		} catch (tf2::TransformException &ex) {
			RCLCPP_WARN(get_logger(), "YardSlam scan: %s",ex.what());
			return;
		}

		for (auto iter = map_obs_list.obs_map.begin(); iter != map_obs_list.obs_map.end(); ++iter) {
			const yard::ObsStats& os = iter->second;
			map_point.point.x = os.meanPoint.x;
			map_point.point.y = os.meanPoint.y;
			map_point.point.z = 0.0;
			try{
	      tf2::doTransform(map_point, base_point, base_to_map_geom);
	    }
	    catch(tf2::TransformException& ex){
	      RCLCPP_ERROR(get_logger(), "YardSlam Exception trying to transform a map to base : %s", ex.what());
				return;
	    }
			// measurement
			ekf_pose.updateExpectedMeasure(x_trees_[iter->first], y_trees_[iter->first]);
			ekf_pose.applyMeasure(base_point.point.x, base_point.point.y);
			update_map_tf(time_scan);
		}
		map_obs_list.clear();
		time_accum_meas = time_scan;
	}

	if ((time_scan - time_accum_meas).nanoseconds()*1e-9 > meas_reset_time_sec_) {
		map_obs_list.clear();
		time_accum_meas = time_scan;
	}
	// End Accumulated measurement

	// Publish pc in map frame
	auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
	pcl::toROSMsg(pc, *pc2_msg);
	pc2_msg->header.frame_id = "map";
	pc2_msg->header.stamp = time_scan;
	pc_pub_->publish(*pc2_msg);

	pcl::toROSMsg(tree_map_pc, *pc2_msg);
	pc2_msg->header.frame_id = "map";
	pc2_msg->header.stamp = time_scan;
	tree_map_pub_->publish(*pc2_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YardSlam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
