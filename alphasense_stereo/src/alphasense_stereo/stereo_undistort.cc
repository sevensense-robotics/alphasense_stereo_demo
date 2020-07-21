#include "alphasense_stereo/stereo_undistort.h"

#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <yaml-cpp/yaml.h>

namespace alphasense_stereo {
StereoUndistortRos::StereoUndistortRos(
    const YAML::Node& yaml_node)
    : camera_pair_(yaml_node) {
  if (!yaml_node["cam0"]["rostopic"] || !yaml_node["cam1"]["rostopic"]) {
    throw std::runtime_error(
        "Cam0 node is missing rostopic, the yaml file is invalid.");
  }
  const auto& cam_right_topic_name =
      yaml_node["cam0"]["rostopic"].as<std::string>();
  const auto& cam_left_topic_name =
      yaml_node["cam1"]["rostopic"].as<std::string>();
  sub_image_left_.subscribe(it_, cam_left_topic_name, kQueueSize);
  sub_image_right_.subscribe(it_, cam_right_topic_name, kQueueSize);
  auto cam_right_adapted_topic_name = cam_right_topic_name;
  constexpr char kRectifiedTopic[] = "/image_rect";
  constexpr char kCameraInfoTopic[] = "/camera_info";

  pub_image_left_ = node_handle_.advertise<sensor_msgs::Image>(
      cam_left_topic_name + kRectifiedTopic, kQueueSize);
  pub_image_right_ = node_handle_.advertise<sensor_msgs::Image>(
      cam_right_adapted_topic_name + kRectifiedTopic, kQueueSize);
  pub_camera_info_left_ = node_handle_.advertise<sensor_msgs::CameraInfo>(
      cam_left_topic_name + kCameraInfoTopic, kQueueSize);
  pub_camera_info_right_ = node_handle_.advertise<sensor_msgs::CameraInfo>(
      cam_right_adapted_topic_name + kCameraInfoTopic, kQueueSize);

  exact_sync_.reset(new ExactSync(
      ExactPolicy(kQueueSize), sub_image_left_, sub_image_right_));
  exact_sync_->registerCallback(
      boost::bind(&StereoUndistortRos::imageCallback, this, _1, _2));
  ROS_INFO_STREAM(
      "Stereo node is waiting for images under left: "
      << cam_left_topic_name << " and right: " << cam_right_topic_name);
  ROS_INFO_STREAM(
      "Stereo node will published rectified flipped with info under: "
      << cam_left_topic_name << " and right: " << cam_right_adapted_topic_name);
}

void StereoUndistortRos::imageCallback(
    const sensor_msgs::ImageConstPtr& img_left_msg,
    const sensor_msgs::ImageConstPtr& img_right_msg) {
  const auto cv_img_left =
      cv_bridge::toCvShare(img_left_msg, sensor_msgs::image_encodings::MONO8);
  const auto cv_img_right =
      cv_bridge::toCvShare(img_right_msg, sensor_msgs::image_encodings::MONO8);
  camera_pair_.getCamLeft().undistort_rectify(cv_img_left->image);
  if (camera_pair_.isRightFlipped()) {
    // Flip the image upside down inplace.
    cv::flip(cv_img_right->image, cv_img_right->image, -1);
  }
  camera_pair_.getCamRight().undistort_rectify(cv_img_right->image);

  pub_image_left_.publish(cv_img_left->toImageMsg());
  pub_image_right_.publish(cv_img_right->toImageMsg());
  auto cam_left_info = camera_pair_.getCamLeft().generateCameraInfo();
  cam_left_info.header = img_left_msg->header;
  auto cam_right_info = camera_pair_.getCamRight().generateCameraInfo();
  cam_right_info.header = img_right_msg->header;
  pub_camera_info_left_.publish(cam_left_info);
  pub_camera_info_right_.publish(cam_right_info);
}

}  // namespace alphasense_stereo
