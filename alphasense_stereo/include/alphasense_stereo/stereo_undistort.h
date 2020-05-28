#ifndef ALPHASENSE_STEREO_STEREO_UNDISTORT_H_
#define ALPHASENSE_STEREO_STEREO_UNDISTORT_H_

#include <memory>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "alphasense_stereo/fisheye_camera_pair.h"

namespace alphasense_stereo {

using ExactPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, sensor_msgs::Image>;
using ExactSync = message_filters::Synchronizer<ExactPolicy>;

class StereoUndistortRos {
 public:
  StereoUndistortRos(
      const YAML::Node& yaml_node, const bool is_right_flipped = true);

 private:
  void imageCallback(
      const sensor_msgs::ImageConstPtr& img_left_msg,
      const sensor_msgs::ImageConstPtr& img_right_msg);

  static constexpr size_t kQueueSize = 10u;

  // If the right image is flipped.
  bool is_right_flipped_{true};
  CameraPair camera_pair_;
  std::unique_ptr<ExactSync> exact_sync_;

  image_transport::SubscriberFilter sub_image_left_, sub_image_right_;
  ros::Publisher pub_image_left_, pub_image_right_, pub_camera_info_left_,
      pub_camera_info_right_, pub_image_right_flipped_;
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport it_{node_handle_};
};
}  // namespace alphasense_stereo

#endif  // ALPHASENSE_STEREO_STEREO_UNDISTORT_H_
