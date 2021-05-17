/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <boost/format.hpp>
#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

namespace image_view {

class ImageSaverNodelet : public nodelet::Nodelet
{
  image_transport::Subscriber sub_image_;
  image_transport::CameraSubscriber sub_image_and_camera_;
  ros::ServiceServer srv_start_;
  ros::ServiceServer srv_end_;
  ros::ServiceServer srv_save_;

  boost::format g_format_;
  std::string encoding_ = "bgr8";
  bool save_all_image_ = true;
  bool save_image_service_ = false;
  bool request_start_end_ = false;
  bool is_first_image_ = true;
  bool has_camera_info_ = false;
  size_t count_ = 0;
  ros::Time start_time_;
  ros::Time end_time_;

  virtual void onInit();

  void callbackWithoutCameraInfo(const sensor_msgs::ImageConstPtr& image_msg);
  void callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info);
  bool callbackStartSave(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackEndSave(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackSave(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool saveImage(const sensor_msgs::ImageConstPtr& image_msg, std::string &filename);
};

void ImageSaverNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("image");

  ros::NodeHandle local_nh = getPrivateNodeHandle();
  std::string format_string = "left%04i.%s";
  local_nh.param("filename_format", format_string, format_string);
  local_nh.param("encoding", encoding_, encoding_);
  local_nh.param("save_all_image", save_all_image_, save_all_image_);
  local_nh.param("request_start_end", request_start_end_, request_start_end_);
  g_format_.parse(format_string);

  // Useful when CameraInfo is being published
  sub_image_and_camera_ = it.subscribeCamera(topic, 1, &ImageSaverNodelet::callbackWithCameraInfo, this);

  // Useful when CameraInfo is not being published
  sub_image_ = it.subscribe(topic, 1, boost::bind(&ImageSaverNodelet::callbackWithoutCameraInfo, this, _1));

  srv_start_ = local_nh.advertiseService("start", &ImageSaverNodelet::callbackStartSave, this);
  srv_end_ = local_nh.advertiseService("end", &ImageSaverNodelet::callbackEndSave, this);
  srv_save_ = local_nh.advertiseService("save", &ImageSaverNodelet::callbackSave, this);
}

void ImageSaverNodelet::callbackWithoutCameraInfo(const sensor_msgs::ImageConstPtr& image_msg)
{
  if (is_first_image_)
  {
    is_first_image_ = false;

    // Wait a tiny bit to see whether callbackWithCameraInfo is called
    ros::Duration(0.001).sleep();
  }

  if (has_camera_info_)
    return;

  // saving flag priority:
  //  1. request by service.
  //  2. request by topic about start and end.
  //  3. flag 'save_all_image'.
  if (!save_image_service_ && request_start_end_)
  {
    if (start_time_ == ros::Time(0))
      return;
    else if (start_time_ > image_msg->header.stamp)
      return;  // wait for message which comes after start_time
    else if ((end_time_ != ros::Time(0)) && (end_time_ < image_msg->header.stamp))
      return;  // skip message which comes after end_time
  }

  // save the image
  std::string filename;
  if (!saveImage(image_msg, filename))
    return;

  count_++;
}

void ImageSaverNodelet::callbackWithCameraInfo(
  const sensor_msgs::ImageConstPtr& image_msg,
  const sensor_msgs::CameraInfoConstPtr& info)
{
  has_camera_info_ = true;

  if (!save_image_service_ && request_start_end_)
  {
    if (start_time_ == ros::Time(0))
      return;
    else if (start_time_ > image_msg->header.stamp)
      return;  // wait for message which comes after start_time
    else if ((end_time_ != ros::Time(0)) && (end_time_ < image_msg->header.stamp))
      return;  // skip message which comes after end_time
  }

  // save the image
  std::string filename;
  if (!saveImage(image_msg, filename))
    return;

  // save the CameraInfo
  if (info)
  {
    filename = filename.replace(filename.rfind("."), filename.length(), ".ini");
    camera_calibration_parsers::writeCalibration(filename, "camera", *info);
  }
  count_++;
}

bool ImageSaverNodelet::callbackStartSave(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Received start saving request");
  start_time_ = ros::Time::now();
  end_time_ = ros::Time(0);

  res.success = true;
  return true;
}

bool ImageSaverNodelet::callbackEndSave(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Received end saving request");
  end_time_ = ros::Time::now();

  res.success = true;
  return true;
}


bool ImageSaverNodelet::callbackSave(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  save_image_service_ = true;
  return true;
}

bool ImageSaverNodelet::saveImage(const sensor_msgs::ImageConstPtr& image_msg, std::string &filename)
{
  cv::Mat image;
  try
  {
    image = cv_bridge::toCvShare(image_msg, encoding_)->image;
  } catch(cv_bridge::Exception)
  {
    ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding_.c_str());
    return false;
  }

  if (!image.empty()) {
    try {
      filename = (g_format_).str();
    } catch (...) { g_format_.clear(); }
    try {
      filename = (g_format_ % count_).str();
    } catch (...) { g_format_.clear(); }
    try {
      filename = (g_format_ % count_ % "jpg").str();
    } catch (...) { g_format_.clear(); }

    if ( save_all_image_ || save_image_service_ ) {
      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());

      save_image_service_ = false;
    } else {
      return false;
    }
  } else {
    ROS_WARN("Couldn't save image, no data!");
    return false;
  }
  return true;
}

} // namespace image_view

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_view::ImageSaverNodelet, nodelet::Nodelet)
