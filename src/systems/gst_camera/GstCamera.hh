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
#include <gz/sim/System.hh>
#include "gz/sim/Sensor.hh"
#include "gz/sim/rendering/MarkerManager.hh"
#include "gz/sim/components/ParentEntity.hh"
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

// Forward declaration
// class GstCameraPrivate;

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
  public System,
  public ISystemConfigure,
  public ISystemPostUpdate
{
	// public: components::ParentEntity *sensor_parent;
	// protected: rendering::CameraPtr camera;
	// public: GMainLoop *gst_loop;
	/// \brief Constructor
	public: GstCamera();

	/// \brief Destructor
	public: virtual ~GstCamera();

	//Documentation inherited
	public: void Configure(const Entity &_entity,
							const std::shared_ptr<const sdf::Element> &_sdf,
							EntityComponentManager &_ecm,
							EventManager &_eventMgr) override;

	    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

	public: void cbVideoStream(const std::shared_ptr<const msgs::Int32> &_msg);

	public: virtual void OnNewFrame(const void* image,
		unsigned int w, unsigned int h, unsigned int d, const std::string &format);

	public: void startGstThread();
	public: void stopGstThread();
	public: void gstCallback(GstElement *appsrc);

	private: void startStreaming();
	private: void stopStreaming();

	// /// \brief Sensor interface
	// public: Sensor sensor{kNullEntity};

	/// \brief Camera width
	public: unsigned int width = 0;

	/// \brief Camera height
	public: unsigned int height = 0;
	public: unsigned int depth = 0;

	public: float rate;

	public: std::string frameformat;

	public: std::string udpHost;

	public: int udpPort;

	public: bool useRtmp;

	public: std::string rtmpLocation;

	public: bool useCuda;

	/// \brief Name of the camera
  	public: std::string cameraName;

	/// \brief Transport node
  	public: transport::Node node;

	/// \brief Current simulation time.
	public: std::chrono::steady_clock::duration simTime{0};

    /// \brief Pointer to the camera being recorded
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the 3D scene
    public: rendering::ScenePtr scene{nullptr};

	/// \brief Pointer to the event manager
  	public: EventManager *eventMgr = nullptr;

    /// \brief Image from user camera
    public: rendering::Image cameraImage;

	private: common::ConnectionPtr newFrameConnection;

	public: components::ParentEntity *sensor_parent;

	/// \brief Camera entity.
  	public: Entity entity;

	/// \brief Name of service for recording video NECESSARY?????
  	public: std::string service;

	/// \brief Marker manager
	public: MarkerManager markerManager;

	/// \brief Topic that the sensor publishes to
	public: std::string sensorTopic;

	// private: transport::NodePtr node_handle_;
	private: std::string namespace_;

	//   private: transport::SubscriberPtr mVideoSub; //CHECK
	private: pthread_t mThreadId;
	private: const std::string mTopicName = "~/video_stream";
	private: bool mIsActive = false;

	public: GMainLoop *gst_loop = nullptr;
	public: GstElement *gst_source = nullptr;

	/// \brief Copy of the sdf configuration used for this plugin
  	public: sdf::ElementPtr sdfConfig;



    /// \brief Private data pointer
	private: std::unique_ptr<GstCamera> dataPtr;
};
}
}
}
