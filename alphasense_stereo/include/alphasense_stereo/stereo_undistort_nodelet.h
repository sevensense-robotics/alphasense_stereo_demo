#ifndef ALPHASENSE_STEREO_STEREO_UNDISTORT_NODELET_H_
#define ALPHASENSE_STEREO_STEREO_UNDISTORT_NODELET_H_

#include <memory>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "alphasense_stereo/stereo_undistort.h"

namespace alphasense_stereo {
class StereoUndistortNodelet : public nodelet::Nodelet {
 public:
  void onInit() override;

 private:
  std::unique_ptr<StereoUndistortRos> stereo_undistorter_;
};
}  // namespace alphasense_stereo

#endif  // ALPHASENSE_STEREO_STEREO_UNDISTORT_NODELET_H_
