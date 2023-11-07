/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#pragma once

#include <string>
#include <iostream>
#include <mutex>

// #include "gz/common/Plugin.hh"
#include "gz/utils.hh"
// #include "gz/gazebo.hh"
// #include "gz/util/system.hh"
#include "gz/transport.hh"
#include "gz/msgs.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include <gz/sim/System.hh>
#include <gz/rendering/Scene.hh>
#include "gz/rendering/Camera.hh"

#include <gz/sensors/Noise.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gst/gst.h>

namespace gz
{
namespace sim
{
namespace systems
{
/**
 * @class GstCamera
 * A Gazebo plugin that can be attached to a camera and then streams the video data using gstreamer.
 * It streams to a configurable UDP IP and UDP Port, defaults are respectively 127.0.0.1 and 5600.
 *
 * Connect to the stream via command line with:
 * gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
 *  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
 */

class GstCamera:
  public System
{
  /// \brief Constructor
  public: GstCamera();

  /// \brief Destructor
  public: virtual ~GstCamera();

  public: virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf,
    gz::sim::EntityComponentManager &_ecm);

  public: virtual void OnNewFrame(const unsigned char *image,
    unsigned int width, unsigned int height,
	unsigned int depth, const std::string &format);

  public: void startGstThread();
  public: void stopGstThread();
  public: void gstCallback(GstElement *appsrc);

//   public: void cbVideoStream(const boost::shared_ptr<const msgs::Int> &_msg);
  private: void startStreaming();
  private: void stopStreaming();

  /// \brief Camera width
  protected: unsigned int width = 0.0;

  /// \brief Camera height
  protected: unsigned int height = 0.0;

  /// \brief Camera depth
  protected: unsigned int depth = 0.0;

  public: float rate;

  protected: std::string format;

  protected: std::string udpHost;

  protected: int udpPort;

  protected: bool useRtmp;

  protected: std::string rtmpLocation;

  protected: bool useCuda;

  protected: sensors::CameraSensorPtr parentSensor;

  protected: rendering::CameraPtr camera;

  private: gz::common::ConnectionPtr newFrameConnection;

  // private: transport::NodePtr node_handle_;
  private: std::string namespace_;

//   private: transport::SubscriberPtr mVideoSub; //CHECK
  private: pthread_t mThreadId;
  private: const std::string mTopicName = "~/video_stream";
  private: bool mIsActive;

  public: GMainLoop *gst_loop;
  public: GstElement *source;
};

} /* namespace gazebo */
}
}
