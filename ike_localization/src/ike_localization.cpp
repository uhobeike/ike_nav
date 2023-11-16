// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_localization/ike_localization.hpp"

#include "ike_localization/mcl/particle.hpp"

#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/string_utils.hpp>

#include <nav_msgs/srv/get_map.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ike_nav
{

IkeLocalization::IkeLocalization(const rclcpp::NodeOptions & options)
: Node("ike_localization", options),
  ros_clock_(RCL_SYSTEM_TIME),
  scan_receive_(false),
  map_receive_(false)
{
  RCLCPP_INFO(this->get_logger(), "Run IkeLocalization");

  getParam();

  initPubSub();
  initServiceServer();
  initServiceClient();

  getMap();

  loopMcl();
}
IkeLocalization::~IkeLocalization() { RCLCPP_INFO(this->get_logger(), "Done IkeLocalization."); }

void IkeLocalization::getParam()
{
  RCLCPP_INFO(get_logger(), "Run getParam.");

  this->param_listener_ =
    std::make_shared<ike_localization::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  loop_mcl_ms_ = 1 / this->params_.loop_mcl_hz * 1000;

  transform_tolerance_ = this->params_.transform_tolerance;

  particle_size_ = get_parameter("particle_size").get_value<int>();

  initial_pose_x_ = this->params_.initial_pose_x;
  initial_pose_y_ = this->params_.initial_pose_y;
  initial_pose_a_ = this->params_.initial_pose_a;

  map_frame_ = this->params_.map_frame;
  odom_frame_ = this->params_.odom_frame;
  robot_frame_ = this->params_.robot_frame;

  alpha1_ = this->params_.alpha_trans_trans;
  alpha2_ = this->params_.alpha_trans_rotate;
  alpha3_ = this->params_.alpha_rotate_trans;
  alpha4_ = this->params_.alpha_rotate_rotate;

  likelihood_dist_ = this->params_.likelihood_dist;

  publish_particles_scan_match_point_ = this->params_.publish_particles_scan_match_point;

  RCLCPP_INFO(get_logger(), "Done getParam.");
}

void IkeLocalization::initPubSub()
{
  RCLCPP_INFO(get_logger(), "Run initPubSub.");

  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", 2);
  likelihood_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "likelihood_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  particles_scan_match_point_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("mcl_match", 2);
  marginal_likelihood_publisher_ =
    create_publisher<std_msgs::msg::Float32>("marginal_likelihood", 2);
  mcl_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("mcl_pose", 2);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 1, std::bind(&IkeLocalization::receiveScan, this, std::placeholders::_1));
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&IkeLocalization::receiveInitialPose, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Done initPubSub.");
}

void IkeLocalization::initServiceServer()
{
  auto publish_likelihoodfield_map =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    if (init_likelihood_map_) {
      likelihood_map_pub_->publish(map_);
      response->success = true;
      response->message = "Called /publish_likelihoodfield_map. Publish map done.";
    } else {
      response->success = false;
      response->message =
        "Called /publish_likelihoodfield_map.  LikelihoodField Map has not been created yet.";
    }
  };
  publish_likelihoodfield_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "publish_likelihoodfield_map", publish_likelihoodfield_map);
}

void IkeLocalization::initServiceClient()
{
  get_map_srv_client_ = this->create_client<ike_nav_msgs::srv::GetMap>("get_map");
}

void IkeLocalization::getMap()
{
  RCLCPP_INFO(get_logger(), "Run getMap.");

  while (!get_map_srv_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }

  auto request = std::make_shared<ike_nav_msgs::srv::GetMap::Request>();
  using ServiceResponseFuture = rclcpp::Client<ike_nav_msgs::srv::GetMap>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    map_ = result.get()->map;
    map_receive_ = true;
    RCLCPP_INFO(get_logger(), "Received map.");
  };
  auto future_result = get_map_srv_client_->async_send_request(request, response_received_callback);

  RCLCPP_INFO(get_logger(), "Done getMap.");
}

void IkeLocalization::receiveScan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_ = *msg;
  scan_receive_ = true;
}

void IkeLocalization::receiveInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Run receiveInitialPose");

  mcl_->initParticles(
    msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation),
    particle_size_);

  RCLCPP_INFO(get_logger(), "Done receiveInitialPose.");
};

void IkeLocalization::initTf()
{
  RCLCPP_INFO(get_logger(), "Run initTf.");

  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface(),
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  latest_tf_ = tf2::Transform::getIdentity();
  init_tf_ = true;

  RCLCPP_INFO(get_logger(), "Done initTf.");
}

// https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/amcl_node.cpp#L975-L1016
void IkeLocalization::transformMapToOdom()
{
  RCLCPP_INFO(get_logger(), "Run transformMapToOdom.");

  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, maximum_likelihood_particle_.pose.euler.yaw);
    tf2::Transform tmp_tf(
      q, tf2::Vector3(
           maximum_likelihood_particle_.pose.position.x,
           maximum_likelihood_particle_.pose.position.y, 0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = robot_frame_;
    tmp_tf_stamped.header.stamp = scan_.header.stamp;

    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_);
  } catch (tf2::TransformException & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return;
  }

  auto stamp = tf2_ros::fromMsg(scan_.header.stamp);
  tf2::TimePoint transform_expiration = stamp + tf2::durationFromSec(transform_tolerance_);

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = map_frame_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_;
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);

  RCLCPP_INFO(get_logger(), "Done transformMapToOdom.");
}

void IkeLocalization::getCurrentRobotPose(geometry_msgs::msg::PoseStamped & current_pose)
{
  while (rclcpp::ok() &&
         not tf_buffer_->canTransform(odom_frame_, robot_frame_, tf2::TimePoint())) {
    RCLCPP_WARN(get_logger(), "Wait Can Transform");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

  std_msgs::msg::Header header;
  header.set__frame_id(robot_frame_);
  header.set__stamp(rclcpp::Time());
  robot_pose.set__header(header);

  tf_buffer_->transform(robot_pose, current_pose, odom_frame_);
}

void IkeLocalization::setParticles(nav2_msgs::msg::ParticleCloud & particles)
{
  RCLCPP_INFO(get_logger(), "Run setParticles.");

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.nanosec = ros_clock_.now().nanoseconds();
  particles.set__header(header);

  particles.particles.resize(particle_size_);
  for (auto i = 0; i < particle_size_; ++i) {
    particles.particles[i].pose.position.x = mcl_->particles_[i].pose.position.x;
    particles.particles[i].pose.position.y = mcl_->particles_[i].pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, mcl_->particles_[i].pose.euler.yaw);
    particles.particles[i].pose.orientation = tf2::toMsg(q);

    particles.particles[i].weight = mcl_->particles_[i].weight;
  }

  RCLCPP_INFO(get_logger(), "Done setParticles.");
}

geometry_msgs::msg::PoseStamped IkeLocalization::getMclPose(const Particle particle)
{
  RCLCPP_INFO(get_logger(), "Run getMclPose.");

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.nanosec = ros_clock_.now().nanoseconds();

  geometry_msgs::msg::Pose pose;
  pose.position.x = particle.pose.position.x;
  pose.position.y = particle.pose.position.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, particle.pose.euler.yaw);
  pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::PoseStamped mcl_pose;
  mcl_pose.set__header(header);
  mcl_pose.set__pose(pose);

  RCLCPP_INFO(get_logger(), "Done getMclPose.");

  return mcl_pose;
}

visualization_msgs::msg::MarkerArray IkeLocalization::createSphereMarkerArray(
  const std::vector<std::vector<double>> particles_scan_match_point)
{
  int id = 0;
  std::string name = "";
  std_msgs::msg::Header header;
  header.frame_id = map_frame_;
  header.stamp.nanosec = ros_clock_.now().nanoseconds();

  auto marker_array = visualization_msgs::msg::MarkerArray();
  for (auto hit_xy : particles_scan_match_point) {
    auto marker = visualization_msgs::msg::Marker();

    marker.set__header(header);
    marker.ns = name;
    marker.id = id;
    id++;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.;
    marker.color.r = 1.;
    marker.color.g = 0.;
    marker.color.b = 0.;

    marker.pose.position.x = hit_xy[0];
    marker.pose.position.y = hit_xy[1];
    marker.pose.position.z = 0.2;

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

void IkeLocalization::initMcl()
{
  RCLCPP_INFO(get_logger(), "Run initMcl.");

  mcl_.reset();
  mcl_ = std::make_shared<mcl::Mcl>(
    initial_pose_x_, initial_pose_y_, initial_pose_a_, alpha1_, alpha2_, alpha3_, alpha4_,
    particle_size_, likelihood_dist_, map_.info.width, map_.info.height, map_.info.resolution,
    map_.info.origin.position.x, map_.info.origin.position.y, map_.data, scan_.angle_min,
    scan_.angle_max, scan_.angle_increment, scan_.range_min, scan_.range_max,
    publish_particles_scan_match_point_);

  mcl_->observation_model_->likelihood_field_->getLikelihoodField(map_.data);
  likelihood_map_pub_->publish(map_);
  mcl_->initParticles(initial_pose_x_, initial_pose_y_, initial_pose_a_, particle_size_);

  init_mcl_ = true;
  init_likelihood_map_ = true;

  RCLCPP_INFO(get_logger(), "Done initMcl.");
}

void IkeLocalization::mcl_to_ros2()
{
  RCLCPP_INFO(get_logger(), "Run mcl_to_ros2.");

  nav2_msgs::msg::ParticleCloud particles;
  transformMapToOdom();
  setParticles(particles);
  publishParticles(particles);
  publishMclPose(getMclPose(maximum_likelihood_particle_));
  publishMarginalLikelihood(mcl_->getMarginalLikelihood());
  if (publish_particles_scan_match_point_)
    publishParticlesScanMatchPoint(createSphereMarkerArray(mcl_->getParticlesScanMatchPoint()));

  RCLCPP_INFO(get_logger(), "Done mcl_to_ros2.");
}

void IkeLocalization::loopMcl()
{
  mcl_loop_timer_ = create_wall_timer(std::chrono::milliseconds{loop_mcl_ms_}, [this]() {
    if (rclcpp::ok() && scan_receive_ && map_receive_ && init_tf_ && init_mcl_) {
      RCLCPP_INFO(get_logger(), "Run IkeLocalization::loopMcl");
      getCurrentRobotPose(current_pose_);

      mcl_->motion_model_->getDelta(
        delta_x_, delta_y_, delta_yaw_, current_pose_.pose.position.x, past_pose_.pose.position.x,
        current_pose_.pose.position.y, past_pose_.pose.position.y,
        tf2::getYaw(current_pose_.pose.orientation), tf2::getYaw(past_pose_.pose.orientation));

      mcl_->motion_model_->update(
        mcl_->particles_, tf2::getYaw(current_pose_.pose.orientation), delta_x_, delta_y_,
        delta_yaw_);

      mcl_->observation_model_->update(mcl_->particles_, scan_.ranges);

      mcl_->resampling_->resampling(mcl_->particles_);

      mcl_->getMeanParticle(maximum_likelihood_particle_);
      past_pose_ = current_pose_;

      mcl_to_ros2();
    } else {
      if (!init_tf_) initTf();
      if (init_tf_) {
        getCurrentRobotPose(current_pose_);
        past_pose_ = current_pose_;
      }
      if (map_receive_ && scan_receive_ && !init_mcl_) initMcl();
      if (not scan_receive_)
        RCLCPP_WARN(get_logger(), "Not yet received scan. Therefore, MCL cannot be initiated.");
      if (not map_receive_)
        RCLCPP_WARN(get_logger(), "Not yet received map. Therefore, MCL cannot be initiated.");
    }
  });
}
}  // namespace ike_nav

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_nav::IkeLocalization)
