/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Johns Hopkins University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the Johns Hopkins University nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
 * Edited by: Alessio Levratti, alessio.levratti@unimore.it, University of Modena and Reggio Emilia
 * Version 0.2
 */

// ROS Dependencies
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <iostream>
#include <fstream>
#include <string.h>
#include <strings.h>
#include <std_msgs/Int16.h>
#include <skeleton_tracker/user_IDs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
// NITE Dependencies
#include "NiTE.h"
#include <openni2_camera/openni2_device.h>
#include <openni2/OniCTypes.h>
#include <openni2/OpenNI.h>
#include <openni2/OniCEnums.h>
#include "openni_camera/openni_depth_image.h"
#include "openni_camera/openni_exception.h"
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};


#define USER_MESSAGE(msg) \
        {printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

typedef std::map<std::string, nite::SkeletonJoint> JointMap;

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

void updateUserState(const nite::UserData& user, unsigned long long ts)
{
  if (user.isNew())
    USER_MESSAGE("New")
  else if (user.isVisible() && !g_visibleUsers[user.getId()])
    USER_MESSAGE("Visible")
  else if (!user.isVisible() && g_visibleUsers[user.getId()])
    USER_MESSAGE("Out of Scene")
  else if (user.isLost())
    USER_MESSAGE("Lost")

  g_visibleUsers[user.getId()] = user.isVisible();

  if (g_skeletonStates[user.getId()] != user.getSkeleton().getState())
  {
    switch (g_skeletonStates[user.getId()] = user.getSkeleton().getState())
    {
      case nite::SKELETON_NONE:
        USER_MESSAGE("Stopped tracking.")
        break;
      case nite::SKELETON_CALIBRATING:
        USER_MESSAGE("Calibrating...")
        break;
      case nite::SKELETON_TRACKED:
        USER_MESSAGE("Tracking!")
        break;
      case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
      case nite::SKELETON_CALIBRATION_ERROR_HANDS:
      case nite::SKELETON_CALIBRATION_ERROR_LEGS:
      case nite::SKELETON_CALIBRATION_ERROR_HEAD:
      case nite::SKELETON_CALIBRATION_ERROR_TORSO:
        USER_MESSAGE("Calibration Failed... :-|")
        break;
    }
  }
}

bool publishJointTF(ros::NodeHandle& nh, tf::TransformBroadcaster& br, std::string j_name, nite::SkeletonJoint j,
                    std::string tf_prefix, std::string relative_frame, int uid)
{

  if (j.getPositionConfidence() > 0.0)
  {
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(j.getPosition().x / 1000.0, j.getPosition().y / 1000.0, j.getPosition().z / 1000.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    std::stringstream frame_id_stream;
    std::string frame_id;
    frame_id_stream << "/" << tf_prefix << "/user_" << uid << "/" << j_name;
    frame_id = frame_id_stream.str();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), relative_frame, frame_id));
  }
  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "openni2_tracker");
  ros::NodeHandle nh;
  ros::Publisher user = nh.advertise<skeleton_tracker::user_IDs>("/people", 1);
  ros::Publisher pub_point_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/camera/point_cloud", 5);
  ros::Publisher pub_depth_info_ = nh.advertise<sensor_msgs::CameraInfo>("camera/depth_info", 5);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("/camera/rgb/image", 1);
//  image_transport::Publisher pub_depth_image = it.advertise("/camera/depth/image", 5);
  ros::NodeHandle pnh("~"); //private node handler
  std::string tf_prefix, relative_frame = "";
  tf::TransformBroadcaster br;
  ROS_WARN("WARNING!!!! This node is deprecated! Use \"xtion_tracker\" instead!");

  // Get Tracker Parameters
  if (!pnh.getParam("tf_prefix", tf_prefix))
  {
    ROS_ERROR("tf_prefix not found on Param Server! Check your launch file!");
    return -1;
  }
  if (!pnh.getParam("relative_frame", relative_frame))
  {
    ROS_ERROR("relative_frame not found on Param Server! Check your launch file!");
    return -1;
  }

  // Initial OpenNI
  if (openni::OpenNI::initialize() != openni::STATUS_OK)
  {
    ROS_ERROR("openni initial error: \n");
    return -1;
  }

  openni::Device devDevice;
  if (devDevice.open(openni::ANY_DEVICE) != openni::STATUS_OK)
  {
    ROS_ERROR("Can't Open Device: \n");
    return -1;
  }
  ROS_INFO("Device opened");

  // NITE Stuff
  nite::UserTracker userTracker;
  nite::Status niteRc;
  nite::NiTE::initialize();
  niteRc = userTracker.create();
  if (niteRc != nite::STATUS_OK)
  {
    printf("Couldn't create user tracker\n");
    return 3;
  }

  // Loop
  printf(
      "\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

  nite::UserTrackerFrameRef userTrackerFrame;
  ros::Rate rate(100.0);

  // Create color stream
  openni::VideoStream vsColorStream;

  if (vsColorStream.create(devDevice, openni::SENSOR_COLOR) == openni::STATUS_OK)
  {
    // set video mode
    openni::VideoMode mMode;
    mMode.setResolution(640, 480);
    mMode.setFps(30);
    mMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

    if (vsColorStream.setVideoMode(mMode) != openni::STATUS_OK)
    {
      ROS_INFO("Can't apply videomode\n");
    }

    // image registration
    if (devDevice.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
      devDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }
    vsColorStream.setMirroringEnabled(false);
  }
  else
  {
    ROS_ERROR("Can't create color stream on device: ");
    return -1;
  }

  // start
  vsColorStream.start();

  openni::VideoFrameRef depthFrame;
  openni::VideoStream depthStream;

  int cols, rows;

  if (depthStream.create(devDevice, openni::SENSOR_DEPTH) == openni::STATUS_OK)
  {
    // set video mode
    openni::VideoMode depthMode;
    depthMode = depthStream.getVideoMode();
    cols = depthMode.getResolutionX();
    rows = depthMode.getResolutionY();

    if (depthStream.setVideoMode(depthMode) != openni::STATUS_OK)
    {
      //ROS_WARN("Can't apply depth-videomode\n");
    }

    depthStream.setMirroringEnabled(false);
  }
  else
  {
    ROS_ERROR("Can't create depth stream on device: ");
    return -1;
  }

  while (nh.ok())
  {
    skeleton_tracker::user_IDs ids;
    // Video capture

    // get color frame
    openni::VideoFrameRef vfColorFrame;
    sensor_msgs::ImagePtr msg;
    //cv::Mat mImageBGR;
    if (vsColorStream.readFrame(&vfColorFrame) == openni::STATUS_OK)
    {
      // convert data to OpenCV format
      const cv::Mat mImageRGB(vfColorFrame.getHeight(), vfColorFrame.getWidth(), CV_8UC3,
                              const_cast<void*>(vfColorFrame.getData()));

      // Check if grabbed frame is actually full with some content
      if (!mImageRGB.empty())
      {
        msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", mImageRGB).toImageMsg();
        image_pub.publish(msg);
        cv::flip(mImageRGB, mImageRGB, 1);
        msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", mImageRGB).toImageMsg();
      }
      vfColorFrame.release();
    }

    // get depth frame
    float centerX, centerY;
    unsigned depthStep = 1, depthSkip = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZRGB>());
    depthStream.start();

    if (depthStream.readFrame(&depthFrame) == openni::STATUS_OK)
    {
      unsigned color_step, color_skip;

      openni::DeviceInfo info = devDevice.getDeviceInfo();
      const char* uri = info.getUri();
      std::string stringa(uri);
      openni2_wrapper::OpenNI2Device dev(stringa);
      cloud_msg->header.stamp = 0;
      cloud_msg->width = depthFrame.getWidth();
      cloud_msg->height = depthFrame.getHeight();
      centerX = (cloud_msg->width >> 1) - 0.5f;
      centerY = (cloud_msg->height >> 1) - 0.5f;
      cloud_msg->is_dense = false;
      cloud_msg->points.resize((unsigned long)(cloud_msg->height * cloud_msg->width));



      // Publish camera_info
      sensor_msgs::CameraInfo info_msg;
//      info_msg->header.stamp = 0;
      info_msg.header.frame_id = "camera_depth_optical_frame";
      info_msg.width = 640;
      info_msg.height = 480;
      info_msg.D = std::vector<double>(5, 0.0);
      info_msg.distortion_model = "plumb_bob";
      double f = dev.getDepthFocalLength(depthFrame.getHeight());
      info_msg.K[0] = info_msg.K[4] = 570.3422241210938;
      info_msg.K[2] = 314.5; //(info_msg.width / 2) - 0.5;
      info_msg.K[5] = 235.5; //(info_msg.width * 3. / 8.) - 0.5; //aspect ratio for the camera center on kinect and presumably other devices is 4/3
      info_msg.K[8] = 1.0;
      // no rotation: identity
      info_msg.R[0] = info_msg.R[4] = info_msg.R[8] = 1.0;
      // no rotation, no translation => P=K(I|0)=(K|0)
      info_msg.P[0] = info_msg.P[5] = info_msg.K[0];
      info_msg.P[2] = info_msg.K[2];
      info_msg.P[6] = info_msg.K[5];
      info_msg.P[10] = 1.0;
      pub_depth_info_.publish(info_msg);

      color_step = 3 * msg->width / cloud_msg->width;
      color_skip = 3 * (msg->height / cloud_msg->height - 1) * msg->width;

      const uint8_t* rgb_buffer = &msg->data[0];

      const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();

      float bad_point = std::numeric_limits<float>::quiet_NaN();

      float constant = 0.001 / dev.getDepthFocalLength(depthFrame.getHeight());

      cloud_msg->header.frame_id = relative_frame;

      int color_idx = 0, depth_idx = 0;
      pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud_msg->begin();
      for (int v = 0; v < (int)cloud_msg->height; ++v, color_idx += color_skip)
      {
        for (int u = 0; u < (int)cloud_msg->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
        {
          pcl::PointXYZRGB& pt = *pt_iter;

          if (pDepth[depth_idx] == 0 || pDepth[depth_idx] > 10000)
          {
            // not valid
            pt.x = pt.y = pt.z = bad_point;
            continue;
          }
          // Fill in XYZRGB
          pt.x = (u - centerX) * pDepth[depth_idx] * constant;
          pt.y = -(v - centerY) * pDepth[depth_idx] * constant;
          pt.z = pDepth[depth_idx] * 0.001;
          RGBValue color;
          color.Red = rgb_buffer[color_idx];
          color.Green = rgb_buffer[color_idx + 1];
          color.Blue = rgb_buffer[color_idx + 2];
          color.Alpha = 0;
          pt.rgb = color.float_value;
        }
      }
      sensor_msgs::PointCloud2 pc;
      pcl::toROSMsg(*cloud_msg, pc);
      pc.header.stamp = ros::Time::now();
      pub_point_cloud.publish(pc);
    }
    else
    {
      ROS_WARN("Failed to get the Point Cloud");
    }
    depthStream.stop();
    niteRc = userTracker.readFrame(&userTrackerFrame);
    if (niteRc != nite::STATUS_OK)
    {
      printf("Get next frame failed\n");
      continue;
    }

    const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

    for (int i = 0; i < users.getSize(); ++i)
    {
      const nite::UserData& user = users[i];
      updateUserState(user, userTrackerFrame.getTimestamp());
      if (user.isNew())
      {
        userTracker.startSkeletonTracking(user.getId());
      }
      else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
      {
        JointMap named_joints;

        named_joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
        named_joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK));
        named_joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
        named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
        named_joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
        named_joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
        named_joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
        named_joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
        named_joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO));
        named_joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
        named_joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
        named_joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
        named_joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
        named_joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
        named_joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));

        for (JointMap::iterator it = named_joints.begin(); it != named_joints.end(); ++it)
        {
          publishJointTF(nh, br, it->first, it->second, tf_prefix, relative_frame, user.getId());
        }
        ids.users.push_back(int(user.getId()));
      }
    }
    user.publish(ids);
    rate.sleep();
  }
  nite::NiTE::shutdown();
}


