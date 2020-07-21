#include "alphasense_stereo/stereo_undistort_nodelet.h"

#include <exception>
#include <string>

#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>

PLUGINLIB_EXPORT_CLASS(
    alphasense_stereo::StereoUndistortNodelet, nodelet::Nodelet)

namespace alphasense_stereo {

void StereoUndistortNodelet::onInit() {
  std::string kalibr_file;
  if (!getPrivateNodeHandle().getParam("calibration_file", kalibr_file)) {
    throw std::runtime_error(
        "The calibration file is missing. Add yaml file path to launch file.");
  }
  try {
    NODELET_INFO_STREAM(
        "Loading File from the following path: " << kalibr_file);
    stereo_undistorter_.reset(
        new StereoUndistortRos(YAML::LoadFile(kalibr_file)));
  } catch (const std::exception& e) {
    NODELET_FATAL_STREAM(
        "Initializing node failed due to "
        << e.what() << ". The file \"" << kalibr_file
        << "\" is not a valid calibration file.");
  }
}

}  // namespace alphasense_stereo
