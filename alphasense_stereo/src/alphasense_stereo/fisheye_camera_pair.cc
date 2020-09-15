#include "alphasense_stereo/fisheye_camera_pair.h"

#include <initializer_list>
#include <vector>

namespace alphasense_stereo {

void FisheyeCamera::undistort_rectify(cv::Mat image) const {
  if (!initialized_) {
    throw std::runtime_error(
        "Fisheye camera must be initialized before reprojection.");
  }
  cv::remap(
      image, image, map_1_, map_2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void FisheyeCamera::initUndistortRectifyMaps(
    const cv::Mat& R, const cv::Mat& P) {
  cv::fisheye::initUndistortRectifyMap(
      K_, D_, R, P, size_, CV_16SC2, map_1_, map_2_);
  P_ = P;
  R_ = R;
  initialized_ = true;
}

sensor_msgs::CameraInfo FisheyeCamera::generateCameraInfo() const {
  sensor_msgs::CameraInfo info;
  info.height = size_.height;
  info.width = size_.width;
  memcpy(info.K.c_array(), K_.val, info.K.size() * sizeof(double));
  memcpy(info.P.c_array(), P_.data, info.P.size() * sizeof(double));
  memcpy(info.R.c_array(), R_.data, info.R.size() * sizeof(double));
  info.D.resize(4);
  memcpy(info.D.data(), D_.val, info.D.size() * sizeof(double));
  info.distortion_model = "equidistant";
  return info;
}

FisheyeCamera FisheyeCamera::getFlipped() const {
  // For pinhole camera with fisheye distortion, flipping the camera simply
  // means modifying cx and cy in intrinsics.
  auto K_inv = K_;
  K_inv(1, 2) = size_.height - K_(1, 2);
  K_inv(0, 2) = size_.width - K_(0, 2);
  return FisheyeCamera(K_inv, D_, size_);
}

CameraPair::CameraPair(const YAML::Node& yaml_node) {
  const auto cam0_node = yaml_node["cam0"];
  if (!cam0_node) {
    throw std::runtime_error("Cam0 node is missing, the yaml file is invalid.");
  }
  const auto cam1_node = yaml_node["cam1"];
  if (!cam1_node) {
    throw std::runtime_error("Cam1 node is missing, the yaml file is invalid.");
  }
  
  // Parse 2d vector.
  std::vector<std::vector<double>> rotation;
  std::vector<double> translation;
  if (!cam1_node["T_cn_cnm1"]) {
    throw std::runtime_error("Extrinsics missing between stereo camera pair.");
  }

  // Convert extrinsics transformation to cv Mat.
  // OpenCV uses the convention of transformation of point coordinates from left
  // to right camera frame. (T_r_l)
  for (const auto& row : cam1_node["T_cn_cnm1"]) {
    const std::vector<double> row_in_T = row.as<std::vector<double>>();
    rotation.emplace_back(
        std::initializer_list<double>{row_in_T[0], row_in_T[1], row_in_T[2]});
    translation.emplace_back(row_in_T[3]);
  }

  is_right_flipped_ = rotation[0][0] < 0 && translation[0] > 0;
  is_left_flipped_ = rotation[0][0] < 0 && translation[0] < 0;

  //This is the equivalent to premultiplying a flipping matrix
  if (is_right_flipped_) {
    R_ = cv::Matx33d(
           -1*rotation[0][0], -1*rotation[0][1], -1*rotation[0][2], -1*rotation[1][0],
           -1*rotation[1][1], -1*rotation[1][2], rotation[2][0], rotation[2][1],
           rotation[2][2]);
    t_ = cv::Matx31d(-1*translation[0], -1*translation[1], translation[2]);
  } else if (is_left_flipped_) {
    R_ = cv::Matx33d(
           -1*rotation[0][0], -1*rotation[0][1], -rotation[0][2], -1*rotation[1][0],
           -1*rotation[1][1], rotation[1][2], -1*rotation[2][0], -1*rotation[2][1],
           rotation[2][2]);
    t_ = cv::Matx31d(translation[0], translation[1], translation[2]);
  } else {
    R_ = cv::Matx33d(
           rotation[0][0], rotation[0][1], rotation[0][2], rotation[1][0],
           rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1],
           rotation[2][2]);
    t_ = cv::Matx31d(translation[0], translation[1], translation[2]);
  }

  // “cam0” is on the right on Alphasense, parse it from YAML.
  cam_right_ = fromYAMLNode(cam0_node);
  cam_right_ = is_right_flipped_ ? cam_right_.getFlipped() : cam_right_;
  // “cam1” is on the left on Alphasense.
  cam_left_ = fromYAMLNode(cam1_node);
  cam_left_ = is_left_flipped_ ? cam_left_.getFlipped() : cam_left_;


  // Checking that the x component of translation is negative.
  if (t_(0, 0) > 0.0) {
    std::cerr << "Translation is " << t_ << std::endl;
    throw std::runtime_error(
        "Camera right must be right in the coordinate frame of left camera.");
  }

  // Create rectification transformations.
  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(
      cam_left_.K(), cam_left_.D(), cam_right_.K(), cam_right_.D(),
      cam_left_.size(), R_, t_, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY);
  cam_right_.initUndistortRectifyMaps(R2, P2);
  cam_left_.initUndistortRectifyMaps(R1, P1);
}

FisheyeCamera CameraPair::fromYAMLNode(const YAML::Node& node) {
  if (node["camera_model"].as<std::string>() != "pinhole") {
    throw std::runtime_error("Only the pinhole camera model is supported.");
  }
  if (node["distortion_model"].as<std::string>() != "equidistant") {
    throw std::runtime_error(
        "Only the equidistant distortion model is supported.");
  }
  const std::vector<double> intrinsics =
      node["intrinsics"].as<std::vector<double>>();
  const std::vector<double> distortion_coeffs =
      node["distortion_coeffs"].as<std::vector<double>>();
  const std::vector<int> img_size = node["resolution"].as<std::vector<int>>();
  const cv::Size size(img_size[0], img_size[1]);
  return FisheyeCamera(
      cv::Matx33d(
          intrinsics[0], 0, intrinsics[2], 0, intrinsics[1], intrinsics[3], 0,
          0, 1),
      cv::Matx41d(
          distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2],
          distortion_coeffs[3]),
      size);
}

// The following is mostly identical to the opencv stereoRectify function from
// https://github.com/opencv/opencv/blob/master/modules/calib3d/src/fisheye.cpp#L623
// The function signature is changed so that references are used for output.
void CameraPair::stereoRectify(
    cv::Matx33d K1, cv::Matx41d D1, cv::Matx33d K2, cv::Matx41d D2,
    cv::Size size, cv::Matx33d R, cv::Matx31d _t, cv::Mat& R1, cv::Mat& R2,
    cv::Mat& P1, cv::Mat& P2, cv::Mat& Q, int flags, const cv::Size new_size,
    double balance, double fov_scale) {
  cv::Vec3d rvec = cv::Affine3d(R).rvec();  // Rodrigues vector
  // rectification algorithm
  rvec *= -0.5;  // get average rotation
  cv::Matx33d r_r;
  cv::Rodrigues(rvec, r_r);  // rotate cameras to same orientation by averaging

  const cv::Vec3d tvec(_t(0, 0), _t(1, 0), _t(2, 0));
  // Translation of the virtual camera.
  const cv::Vec3d t = r_r * tvec;
  // Baseline vector.
  const cv::Vec3d uu(t[0] > 0 ? 1 : -1, 0, 0);

  // calculate global Z rotation, Z vector in rectified frame.
  cv::Vec3d ww = t.cross(uu);
  const double nw = cv::norm(ww);
  if (nw > 0.0) {
    // Rodrigues vector has maginitude cos(theta)
    ww *= acos(fabs(t[0]) / cv::norm(t)) / nw;
  }
  cv::Matx33d wr;
  cv::Rodrigues(ww, wr);

  // Apply rectifying rotation to both views.
  const cv::Matx33d ri1 = wr * r_r.t();
  cv::Mat(ri1, false).convertTo(R1, R1.empty() ? CV_64F : R1.type());
  const cv::Matx33d ri2 = wr * r_r;
  cv::Mat(ri2, false).convertTo(R2, R2.empty() ? CV_64F : R2.type());
  const cv::Vec3d tnew = ri2 * tvec;

  // Calculate projection/camera matrices. these contain the relevant rectified
  // image internal params (fx, fy=fx, cx, cy)
  cv::Matx33d newK1, newK2;
  estimateNewCameraMatrixForUndistortRectify(
      K1, D1, size, R1, newK1, balance, new_size, fov_scale);
  estimateNewCameraMatrixForUndistortRectify(
      K2, D2, size, R2, newK2, balance, new_size, fov_scale);

  double fc_new = std::min(newK1(1, 1), newK2(1, 1));
  cv::Point2d cc_new[2] = {cv::Vec2d(newK1(0, 2), newK1(1, 2)),
                           cv::Vec2d(newK2(0, 2), newK2(1, 2))};

  // Vertical focal length must be the same for both images to keep the epipolar
  // constraint use fy for fx also. For simplicity, set the principal points for
  // both cameras to be the average of the two principal points (either one of
  // or both x- and y- coordinates)
  if (flags & cv::CALIB_ZERO_DISPARITY)
    cc_new[0] = cc_new[1] = (cc_new[0] + cc_new[1]) * 0.5;
  else
    cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y) * 0.5;

  cv::Mat(
      cv::Matx34d(
          fc_new, 0, cc_new[0].x, 0, 0, fc_new, cc_new[0].y, 0, 0, 0, 1, 0),
      false)
      .convertTo(P1, P1.empty() ? CV_64F : P1.type());

  cv::Mat(
      cv::Matx34d(
          fc_new, 0, cc_new[1].x,
          tnew[0] * fc_new,  // baseline * focal length;,
          0, fc_new, cc_new[1].y, 0, 0, 0, 1, 0),
      false)
      .convertTo(P2, P2.empty() ? CV_64F : P2.type());

  cv::Mat(
      cv::Matx44d(
          1, 0, 0, -cc_new[0].x, 0, 1, 0, -cc_new[0].y, 0, 0, 0, fc_new, 0, 0,
          -1. / tnew[0], (cc_new[0].x - cc_new[1].x) / tnew[0]),
      false)
      .convertTo(Q, Q.empty() ? CV_64F : Q.depth());
}


// This is moved from opencv, changed output Knew format for consistency.
void CameraPair::estimateNewCameraMatrixForUndistortRectify(
    cv::Matx33d K, cv::Matx41d D, const cv::Size& image_size, cv::Matx33d R,
    cv::Matx33d& Knew, double balance, const cv::Size& new_size,
    double fov_scale) {
  int w = image_size.width, h = image_size.height;
  balance = std::min(std::max(balance, 0.0), 1.0);

  // Find 4 points in center of image edges.
  cv::Mat points(1, 4, CV_64FC2);
  cv::Vec2d* pptr = points.ptr<cv::Vec2d>();
  pptr[0] = cv::Vec2d(w / 2, 0);
  pptr[1] = cv::Vec2d(w, h / 2);
  pptr[2] = cv::Vec2d(w / 2, h);
  pptr[3] = cv::Vec2d(0, h / 2);

  cv::fisheye::undistortPoints(points, points, K, D, R);
  cv::Scalar center_mass = mean(points);
  cv::Vec2d cn(center_mass.val);

  double aspect_ratio = K(0, 0) / K(1, 1);

  // convert to identity ratio
  cn[0] *= aspect_ratio;
  for (size_t i = 0; i < points.total(); ++i)
    pptr[i][1] *= aspect_ratio;

  double minx = DBL_MAX, miny = DBL_MAX, maxx = -DBL_MAX, maxy = -DBL_MAX;
  for (size_t i = 0; i < points.total(); ++i) {
    miny = std::min(miny, pptr[i][1]);
    maxy = std::max(maxy, pptr[i][1]);
    minx = std::min(minx, pptr[i][0]);
    maxx = std::max(maxx, pptr[i][0]);
  }

  double f1 = w * 0.5 / (cn[0] - minx);
  double f2 = w * 0.5 / (maxx - cn[0]);
  double f3 = h * 0.5 * aspect_ratio / (cn[1] - miny);
  double f4 = h * 0.5 * aspect_ratio / (maxy - cn[1]);

  double fmin = std::min(f1, std::min(f2, std::min(f3, f4)));
  double fmax = std::max(f1, std::max(f2, std::max(f3, f4)));

  double f = balance * fmin + (1.0 - balance) * fmax;
  f *= fov_scale > 0 ? 1.0 / fov_scale : 1.0;

  cv::Vec2d new_f(f, f), new_c = -cn * f + cv::Vec2d(w, h * aspect_ratio) * 0.5;

  // restore aspect ratio
  new_f[1] /= aspect_ratio;
  new_c[1] /= aspect_ratio;

  if (!new_size.empty()) {
    double rx = new_size.width / static_cast<double>(image_size.width);
    double ry = new_size.height / static_cast<double>(image_size.height);

    new_f[0] *= rx;
    new_f[1] *= ry;
    new_c[0] *= rx;
    new_c[1] *= ry;
  }

  Knew = cv::Matx33d(new_f[0], 0, new_c[0], 0, new_f[1], new_c[1], 0, 0, 1);
}

}  // namespace alphasense_stereo
