/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tegra_stereo/tegra_stereo_proc.hpp"

namespace tegra_stereo
{

TegraStereoProc::TegraStereoProc() {}
TegraStereoProc::~TegraStereoProc() {}

void TegraStereoProc::onInit()
{

    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    private_nh.param<int> ("P1", p1_, 20);
    private_nh.param<int> ("P2", p2_, 100);

    private_nh.param<int> ("queue_size", queue_size_, 10u);
    private_nh.param<bool> ("rectify_images", rectifyImages_, true);

    //camera calibration files
    std::string cameraCalibrationFileLeft;
    std::string cameraCalibrationFileRight;
    private_nh.param<std::string> ("camera_calibration_file_left", cameraCalibrationFileLeft, "");
    private_nh.param<std::string> ("camera_calibration_file_right", cameraCalibrationFileRight, "");

    if (! (cameraCalibrationFileRight.empty() && cameraCalibrationFileLeft.empty()))
    {
        std::string cameraName;
        mCameraInfoLeftPtr_ = boost::make_shared<sensor_msgs::CameraInfo>();
        mCameraInfoRightPtr_ = boost::make_shared<sensor_msgs::CameraInfo>();
        NODELET_INFO_STREAM ("Stereo calibration left:" << cameraCalibrationFileLeft);
        NODELET_INFO_STREAM ("Stereo calibration right:" << cameraCalibrationFileRight);
        camera_calibration_parsers::readCalibration (cameraCalibrationFileLeft, cameraName, *mCameraInfoLeftPtr_);
        camera_calibration_parsers::readCalibration (cameraCalibrationFileRight, cameraName, *mCameraInfoRightPtr_);

        left_model_.fromCameraInfo (mCameraInfoLeftPtr_);
        right_model_.fromCameraInfo (mCameraInfoRightPtr_);
        stereo_model_.fromCameraInfo (mCameraInfoLeftPtr_, mCameraInfoRightPtr_);
        NODELET_INFO ("Stereo calibration initialized from file");

    }
    else
    {
        NODELET_INFO ("Stereo calibration files are not specified");
    }

    imageTransport_ = boost::make_shared<image_transport::ImageTransport> (nh);

    left_raw_sub_.subscribe (*imageTransport_.get(), "/stereo/left/image_raw", 1);
    right_raw_sub_.subscribe (*imageTransport_.get(), "/stereo/right/image_raw", 1);

    left_info_sub_.subscribe (nh, "/stereo/left/camera_info", 1);
    right_info_sub_.subscribe (nh, "/stereo/right/camera_info", 1);

    left_rect_pub_ = imageTransport_->advertise ("/stereo/left/image_rect", 1);
    right_rect_pub_ = imageTransport_->advertise ("/stereo/right/image_rect", 1);
    pub_disparity_raw_ = imageTransport_->advertise ("/stereo/disparity_raw", 1);
    pub_disparity_ = nh.advertise<stereo_msgs::DisparityImage> ("/stereo/disparity", 1);
    pub_points_ = nh.advertise<sensor_msgs::PointCloud> ("/stereo/points", 1);
    pub_points2_ = nh.advertise<sensor_msgs::PointCloud2> ("/stereo/points2", 1);

    // Synchronize input topics
    info_exact_sync_ = boost::make_shared<InfoExactSync_t> (InfoExactPolicy_t (10u), left_info_sub_, right_info_sub_);
    info_exact_sync_->registerCallback (boost::bind (&TegraStereoProc::infoCallback, this, _1, _2));

    image_exact_sync_ = boost::make_shared<ImageExactSync_t> (ImageExactPolicy_t (10u), left_raw_sub_, right_raw_sub_);


    // Initialize Semi-Global Matcher
    init_disparity_method (static_cast<uint8_t> (p1_), static_cast<uint8_t> (p2_));
    NODELET_INFO ("Init done; P1 %d; P2: %d", p1_, p2_);

}

void TegraStereoProc::infoCallback (
    const sensor_msgs::CameraInfoConstPtr &l_info_msg,
    const sensor_msgs::CameraInfoConstPtr &r_info_msg)
{

    std::call_once (calibration_initialized_flag_, [ &, this] ()
    {

        left_model_.fromCameraInfo (l_info_msg);
        right_model_.fromCameraInfo (r_info_msg);
        stereo_model_.fromCameraInfo (l_info_msg, r_info_msg);
        //we do not need to listen to this anymore
        right_info_sub_.unsubscribe();
        left_info_sub_.unsubscribe();
        image_exact_sync_->registerCallback (boost::bind (&TegraStereoProc::imageCallback, this, _1, _2));
        NODELET_INFO ("Stereo calibration initialized from first message");
    });
}

void TegraStereoProc::imageCallback (
    const sensor_msgs::ImageConstPtr &l_image_msg,
    const sensor_msgs::ImageConstPtr &r_image_msg)
{

    //Convert to CV format MONO 8bit
    const cv::Mat left_raw = cv_bridge::toCvShare (l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
    const cv::Mat right_raw = cv_bridge::toCvShare (r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

    float elapsed_time_ms;
    cv::Mat disparity;

    if (rectifyImages_)
    {
        cv::Mat left_rect;
        cv::Mat right_rect;

        left_model_.rectifyImage (left_raw, left_rect, cv::INTER_LINEAR);
        right_model_.rectifyImage (right_raw, right_rect, cv::INTER_LINEAR);

        // Compute
        disparity = compute_disparity_method (left_rect, right_rect, &elapsed_time_ms);
        publishRectifiedImages (left_rect, right_rect, l_image_msg, r_image_msg);
    }
    else
    {
        disparity = compute_disparity_method (left_raw, right_raw, &elapsed_time_ms);
    }
    elapsed_time_ms_acc_ += elapsed_time_ms;
    elapsed_time_counter_++;

    if(elapsed_time_counter_ %1000 == 0)
    {
        elapsed_time_ms_acc_ = 1000000/elapsed_time_ms_acc_;
        NODELET_INFO ("Disparity computation at %f [fps] (1000 samples avg);", static_cast<double>(elapsed_time_ms_acc_));
        elapsed_time_ms_acc_ = 0.0;
    }
    auto disparityMsgPtr = boost::make_shared<stereo_msgs::DisparityImage>();

    //processDisparity (disparity, l_image_msg->header, disparityMsgPtr);


}

void TegraStereoProc::publishRectifiedImages (const cv::Mat &left_rect,
        const cv::Mat &right_rect,
        const sensor_msgs::ImageConstPtr &l_image_msg,
        const sensor_msgs::ImageConstPtr &r_image_msg)
{

    sensor_msgs::ImagePtr left_rect_msg = cv_bridge::CvImage (l_image_msg->header, l_image_msg->encoding, left_rect).toImageMsg();
    sensor_msgs::ImagePtr right_rect_msg = cv_bridge::CvImage (r_image_msg->header, r_image_msg->encoding, right_rect).toImageMsg();
    left_rect_pub_.publish (left_rect_msg);
    right_rect_pub_.publish (right_rect_msg);
}

bool TegraStereoProc::processRectified(const cv::Mat left_rect_cv, const cv::Mat right_rect_cv, const std_msgs::Header &header)
{

  // Do block matching to produce the disparity image
  if (pub_disparity_.getNumSubscribers() >0 || pub_points_.getNumSubscribers() > 0 || pub_points2_.getNumSubscribers() >0)
  {

    float elapsed_time_ms;
    cv::Mat disparity_raw = compute_disparity_method (left_rect_cv, right_rect_cv, &elapsed_time_ms);
    elapsed_time_ms_acc_ += elapsed_time_ms;
    elapsed_time_counter_++;

    if(elapsed_time_counter_ %1000 == 0)
    {
        elapsed_time_ms_acc_ = 1000000/elapsed_time_ms_acc_;
        NODELET_INFO ("Disparity computation at %f [fps] (1000 samples avg);", static_cast<double>(elapsed_time_ms_acc_));
        elapsed_time_ms_acc_ = 0.0;
    }
    stereo_msgs::DisparityImagePtr disparity_msgPtr = boost::make_shared<stereo_msgs::DisparityImage>();

    if(pub_disparity_raw_.getNumSubscribers() >0)
    {
        sensor_msgs::ImagePtr raw_disp_msg = cv_bridge::CvImage (header, sensor_msgs::image_encodings::MONO8, disparity_raw).toImageMsg();
        pub_disparity_raw_.publish (raw_disp_msg);
    }

    processDisparity (disparity_raw, header, disparity_msgPtr);

    if(pub_disparity_.getNumSubscribers()>0)
    {
        if (pub_disparity_.getNumSubscribers() > 0)
        {
            pub_disparity_.publish (disparity_msgPtr);
        }
    }

    // Project disparity image to 3d point cloud
    if (pub_points_.getNumSubscribers() > 0)
    {
      //processPoints(disparity_msgPtr, output.left.rect_color, output.left.color_encoding, model, output.points);
    }

    // Project disparity image to 3d point cloud
    if (pub_points2_.getNumSubscribers() > 0)
    {
      //processPoints2(disparity_msgPtr, output.left.rect_color, output.left.color_encoding, model, output.points2);
    }

  }

  return true;
}

void TegraStereoProc::processDisparity (const cv::Mat &disparity, const std_msgs::Header &header, stereo_msgs::DisparityImagePtr &disparityMsgPtr)
{

    // Publish disparity
    static const int DPP = 1; // disparities per pixel
    static const double inv_dpp = 1.0 / DPP;


    disparityMsgPtr->header = disparityMsgPtr->image.header = header;

    auto &dimage = disparityMsgPtr->image;
    dimage.height = left_model_.cameraInfo().height; // TODO hack
    dimage.width = left_model_.cameraInfo().width;
    dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    dimage.step = dimage.width * sizeof (float);
    dimage.data.resize (dimage.step * dimage.height);

    const cv::Mat_<float> dmat (dimage.height, dimage.width, static_cast<float*> (static_cast<void*> (dimage.data.data())),
                                dimage.step);
    // TODO check why these are different
    //const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

    // Stereo parameters
    disparityMsgPtr->f = stereo_model_.right().fx();
    disparityMsgPtr->T = stereo_model_.baseline();

    // Disparity search range
    disparityMsgPtr->min_disparity = 0;
    disparityMsgPtr->max_disparity = 127;
    disparityMsgPtr->delta_d = inv_dpp;

    // We convert from fixed-point to float disparity and also adjust for any
    // x-offset between
    // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
    disparity.convertTo (dmat, dmat.type(), inv_dpp,
                         - (stereo_model_.left().cx() - stereo_model_.right().cx()));

    ROS_ASSERT (dmat.data == &dimage.data[0]);
}


inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void TegraStereoProc::processPoints(const stereo_msgs::DisparityImage& disparity,
                                    const cv::Mat& color, const std::string& encoding,
                                    sensor_msgs::PointCloud& points) const
{
  // Calculate dense point cloud
  const sensor_msgs::Image& dimage = disparity.image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  stereo_model_.projectDisparityImageTo3d(dmat, dense_points_, true);

  // Fill in sparse point cloud message
  points.points.resize(0);
  points.channels.resize(3);
  points.channels[0].name = "rgb";
  points.channels[0].values.resize(0);
  points.channels[1].name = "u";
  points.channels[1].values.resize(0);
  points.channels[2].name = "v";
  points.channels[2].values.resize(0);

  for (int32_t u = 0; u < dense_points_.rows; ++u) {
    for (int32_t v = 0; v < dense_points_.cols; ++v) {
      if (isValidPoint(dense_points_(u,v))) {
        // x,y,z
        geometry_msgs::Point32 pt;
        pt.x = dense_points_(u,v)[0];
        pt.y = dense_points_(u,v)[1];
        pt.z = dense_points_(u,v)[2];
        points.points.push_back(pt);
        // u,v
        points.channels[1].values.push_back(u);
        points.channels[2].values.push_back(v);
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  points.channels[0].values.reserve(points.points.size());
  if (encoding == enc::MONO8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v) {
        if (isValidPoint(dense_points_(u,v))) {
          uint8_t g = color.at<uint8_t>(u,v);
          int32_t rgb = (g << 16) | (g << 8) | g;
          points.channels[0].values.push_back(*(float*)(&rgb));
        }
      }
    }
  }
  else if (encoding == enc::RGB8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& rgb = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          points.channels[0].values.push_back(*(float*)(&rgb_packed));
        }
      }
    }
  }
  else if (encoding == enc::BGR8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& bgr = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          points.channels[0].values.push_back(*(float*)(&rgb_packed));
        }
      }
    }
  }
  else {
    ROS_WARN("Could not fill color channel of the point cloud, unrecognized encoding '%s'", encoding.c_str());
  }
}

void TegraStereoProc::processPoints2(const stereo_msgs::DisparityImage& disparity,
                                     const cv::Mat& color, const std::string& encoding,
                                     sensor_msgs::PointCloud2& points) const
{
  // Calculate dense point cloud
  const sensor_msgs::Image& dimage = disparity.image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  stereo_model_.projectDisparityImageTo3d(dmat, dense_points_, true);

  // Fill in sparse point cloud message
  points.height = dense_points_.rows;
  points.width  = dense_points_.cols;
  points.fields.resize (4);
  points.fields[0].name = "x";
  points.fields[0].offset = 0;
  points.fields[0].count = 1;
  points.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[1].name = "y";
  points.fields[1].offset = 4;
  points.fields[1].count = 1;
  points.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[2].name = "z";
  points.fields[2].offset = 8;
  points.fields[2].count = 1;
  points.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[3].name = "rgb";
  points.fields[3].offset = 12;
  points.fields[3].count = 1;
  points.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  //points.is_bigendian = false; ???
  points.point_step = 16;
  points.row_step = points.point_step * points.width;
  points.data.resize (points.row_step * points.height);
  points.is_dense = false; // there may be invalid points

  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int i = 0;
  for (int32_t u = 0; u < dense_points_.rows; ++u) {
    for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
      if (isValidPoint(dense_points_(u,v))) {
        // x,y,z,rgba
        memcpy (&points.data[i * points.point_step + 0], &dense_points_(u,v)[0], sizeof (float));
        memcpy (&points.data[i * points.point_step + 4], &dense_points_(u,v)[1], sizeof (float));
        memcpy (&points.data[i * points.point_step + 8], &dense_points_(u,v)[2], sizeof (float));
      }
      else {
        memcpy (&points.data[i * points.point_step + 0], &bad_point, sizeof (float));
        memcpy (&points.data[i * points.point_step + 4], &bad_point, sizeof (float));
        memcpy (&points.data[i * points.point_step + 8], &bad_point, sizeof (float));
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  i = 0;
  if (encoding == enc::MONO8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
          uint8_t g = color.at<uint8_t>(u,v);
          int32_t rgb = (g << 16) | (g << 8) | g;
          memcpy (&points.data[i * points.point_step + 12], &rgb, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::RGB8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& rgb = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          memcpy (&points.data[i * points.point_step + 12], &rgb_packed, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::BGR8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& bgr = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          memcpy (&points.data[i * points.point_step + 12], &rgb_packed, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else {
    ROS_WARN("Could not fill color channel of the point cloud, unrecognized encoding '%s'", encoding.c_str());
  }
}

}  // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (tegra_stereo::TegraStereoProc, nodelet::Nodelet)
