#ifndef ALPHASENSE_STEREO_FISHEYE_CAMERA_PAIR_H_
#define ALPHASENSE_STEREO_FISHEYE_CAMERA_PAIR_H_

#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

namespace alphasense_stereo {

class FisheyeCamera {
 public:
  FisheyeCamera(
      const cv::Matx33d& K, const cv::Matx41d& D, const cv::Size& size)
      : K_(K), D_(D), size_(size) {}

  FisheyeCamera() = default;

  void undistort_rectify(cv::Mat image) const;

  void initUndistortRectifyMaps(const cv::Mat& R, const cv::Mat& P);

  // Generate camera info for ros with fisheye distortion.
  sensor_msgs::CameraInfo generateCameraInfo() const;

  const cv::Matx33d& K() const {
    return K_;
  }
  const cv::Matx41d& D() const {
    return D_;
  }
  const cv::Size& size() const {
    return size_;
  }

  // Invert the camera image upside down because of the mounting. Construct a
  // new camera.
  FisheyeCamera getFlipped() const;

 private:
  bool initialized_{false};
  // Intrinsics.
  cv::Matx33d K_;
  // Distortion parameters.
  cv::Matx41d D_;
  // Projection matrix, Rotation matrix.
  cv::Mat P_, R_;
  // Fixed mapping between input and output images, integerized coordinates and
  // fixed-point offsets respectively.
  cv::Mat map_1_, map_2_;
  cv::Size size_;
};

class CameraPair {
 public:
  explicit CameraPair(
      const YAML::Node& yaml_node, const bool is_right_flipped = true);

  const FisheyeCamera& getCamLeft() const {
    return cam_left_;
  }

  const FisheyeCamera& getCamRight() const {
    return cam_right_;
  }

  const FisheyeCamera& getCamRightFlipped() const {
    return cam_right_flipped_;
  }

  bool isRightFlipped() const {
    return is_right_flipped_;
  }

 private:
  FisheyeCamera fromYAMLNode(const YAML::Node& node);

  // Keeping the same function signature as in opencv. R1, R2, P1, P2, Q are the
  // function outputs. They matche the Rotation and Projection matrix follwoing
  // the ROS convention and are part of the camera info. Q matrix is the
  // disparity mapping matrix. Here the output variablesare now passed by
  // reference dueto an issue that could occur on some platforms leading to
  // undefined behavior.
  void stereoRectify(
      cv::Matx33d K1, cv::Matx41d D1, cv::Matx33d K2, cv::Matx41d D2,
      cv::Size size, cv::Matx33d R, cv::Matx31d t, cv::Mat& R1,  // NOLINT
      cv::Mat& R2, cv::Mat& P1, cv::Mat& P2, cv::Mat& Q,         // NOLINT
      int flags = cv::CALIB_ZERO_DISPARITY,
      const cv::Size new_size = cv::Size(), double balance = 0.0,
      double fov_scale = 1.0);

  // An opencv function that estimates a new camera intrinsics matrix after
  // undistortion and rectification given the original camera and a rotation.
  void estimateNewCameraMatrixForUndistortRectify(
      cv::Matx33d K, cv::Matx41d D, const cv::Size& image_size, cv::Matx33d R,
      cv::Matx33d& Knew, double balance, const cv::Size& new_size,  // NOLINT
      double fov_scale);

  // The right camera image could be mounted upside down.
  bool is_right_flipped_{true};
  // Alphasense has a right side camera that's typically flipped. Hence a
  // virtual flipped camera model for the right camera is created.
  FisheyeCamera cam_left_, cam_right_, cam_right_flipped_;
  // Transformation between cameras, default from cam_left_ to cam right
  cv::Matx33d R_;
  cv::Matx31d t_;
};

}  // namespace alphasense_stereo

#endif  // ALPHASENSE_STEREO_FISHEYE_CAMERA_PAIR_H_
