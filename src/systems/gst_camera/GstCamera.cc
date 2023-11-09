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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "GstCamera.hh"
#include <gst/app/gstappsrc.h>

#include <math.h>
#include <string>
#include <iostream>
#include <thread>
#include <time.h>
// #include "Int32.pb.h"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/Name.hh"
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <gz/rendering/RenderingIface.hh>
#include <sdf/Camera.hh>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace gz;
using namespace sim;
using namespace systems;
using namespace cv;


static void* start_thread(void* param) {
  GstCamera* plugin = static_cast<GstCamera*>(param);
  plugin->startGstThread();
  return nullptr;
}

/////////////////////////////////////////////////
GstCamera::GstCamera()
: System(), dataPtr(std::make_unique<GstCamera>())
{
}

/////////////////////////////////////////////////
void GstCamera::startGstThread() {
	gst_init(nullptr, nullptr);

	this->gst_loop = g_main_loop_new(nullptr, FALSE);
	if (!this->gst_loop) {
		gzerr << "Create loop failed. \n";
		return;
	}

	GstElement* pipeline = gst_pipeline_new(nullptr);
	if (!pipeline) {
		gzerr << "ERR: Create pipeline failed. \n";
		return;
	}

	GstElement* source = gst_element_factory_make("appsrc", nullptr);
	GstElement* queue = gst_element_factory_make("queue", nullptr);
	GstElement* converter  = gst_element_factory_make("videoconvert", nullptr);

	GstElement* encoder;
	if (this->useCuda) {
		encoder = gst_element_factory_make("nvh264enc", nullptr);
		g_object_set(G_OBJECT(encoder), "bitrate", 800, "preset", 1, nullptr); //  where is this g_object_set defined?
	} else {
		encoder = gst_element_factory_make("x264enc", nullptr);
		g_object_set(G_OBJECT(encoder), "bitrate", 800, "speed-preset", 6,
			"tune", 4, "key-int-max", 10, nullptr);
	}

	GstElement* payloader;
	GstElement* sink;

	if (useRtmp) {
		payloader = gst_element_factory_make("flvmux", nullptr);
		sink = gst_element_factory_make("rtmpsink", nullptr);
		g_object_set(G_OBJECT(sink), "location", this->rtmpLocation.c_str(),
			nullptr);
	} else {
		payloader = gst_element_factory_make("rtph264pay", nullptr);
		sink  = gst_element_factory_make("udpsink", nullptr);
		g_object_set(G_OBJECT(sink), "host", this->udpHost.c_str(), "port",
			this->udpPort, nullptr);
	}

	if (!source || !queue || !converter || !encoder || !payloader || !sink) {
		gzerr << "ERR: Create elements failed. \n";
		return;
	}

	// Configure source element
	g_object_set(G_OBJECT(source), "caps",
		gst_caps_new_simple ("video/x-raw",
		"format", G_TYPE_STRING, "I420",
		"width", G_TYPE_INT, this->width,
		"height", G_TYPE_INT, this->height,
		"framerate", GST_TYPE_FRACTION, (unsigned int)this->rate, 1, nullptr),
		"is-live", TRUE,
		"do-timestamp", TRUE,
		"stream-type", GST_APP_STREAM_TYPE_STREAM,
		"format", GST_FORMAT_TIME, nullptr);

	// Connect all elements to pipeline
	gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
	payloader, sink, nullptr);

	// Link all elements
	if (gst_element_link_many(source, queue, converter, encoder, payloader,
		sink, nullptr) != TRUE) {
		gzerr << "ERR: Link all the elements failed. \n";
		return;
	}

	this->gst_source = source;
	gst_object_ref(this->gst_source);

	// Start
	gst_element_set_state(pipeline, GST_STATE_PLAYING);
	g_main_loop_run(this->gst_loop);

	// Clean up
	gst_element_set_state(pipeline, GST_STATE_NULL);
	gst_object_unref(GST_OBJECT(pipeline));
	gst_object_unref(this->gst_source);
	g_main_loop_unref(this->gst_loop);
	this->gst_loop = nullptr;
	this->gst_source = nullptr;
}

/////////////////////////////////////////////////
void GstCamera::stopGstThread()
{
	if(this->gst_loop) {
		g_main_loop_quit(this->gst_loop);
	}
}


/////////////////////////////////////////////////
GstCamera::~GstCamera()
{
	// this->sensor_parent->reset();
	// this->camera.reset();
	if (this->gst_loop) {
		g_main_loop_quit(this->gst_loop);
	}
}

/////////////////////////////////////////////////
// void GstCamera::cbVideoStream(const boost::shared_ptr<const msgs::Int> &_msg)
// {
//   gzwarn << "Video Streaming callback: " << _msg->data() << "\n";
//   int enable = _msg->data();
//   if(enable)
//     startStreaming();
//   else
//     stopStreaming();
// }

/////////////////////////////////////////////////
void GstCamera::startStreaming()
{
	if(!this->mIsActive) {
		this->newFrameConnection = this->camera->ConnectNewImageFrame(
			std::bind(&GstCamera::OnNewFrame,this, std::placeholders::_1, this->width, //CHECK THIS
			this->height, this->depth, this->frameformat));

		// this->sensor_parent->SetActive(true);

		/* start the gstreamer event loop */
		pthread_create(&mThreadId, NULL, start_thread, this);
		this->mIsActive = true;
	}

}

/////////////////////////////////////////////////
void GstCamera::stopStreaming()
{
	if(this->mIsActive) {
		stopGstThread();

		pthread_join(mThreadId, NULL);

		// this->sensor_parent->SetActive(false);

		this->newFrameConnection->~Connection();
		this->mIsActive = false;
	}

}

/////////////////////////////////////////////////
void GstCamera::OnNewFrame(const void* image,
                              unsigned int w,
                              unsigned int h,
							  unsigned int d,
							  const std::string &format)
{
	// this->camera->Capture(img);
	this->cameraImage = this->camera->CreateImage();

	// Alloc buffer
	const guint size = w * h * 1.5;
	GstBuffer* buffer = gst_buffer_new_allocate(NULL, size, NULL);

	if (!buffer) {
		gzerr << "gst_buffer_new_allocate failed" << endl;
		return;
	}

	GstMapInfo map;

	if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
		gzerr << "gst_buffer_map failed" << endl;
		return;
	}

	// Color Conversion from RGB to YUV
	Mat frame = Mat(h, w, CV_8UC3);
	Mat frameYUV = Mat(h, w, CV_8UC3);
	frame.data = this->cameraImage.Data<unsigned char>();

	cvtColor(frame, frameYUV, COLOR_RGB2YUV_I420);
	memcpy(map.data, frameYUV.data, size);
	gst_buffer_unmap(buffer, &map);

	GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(this->gst_source),
	buffer);

	if (ret != GST_FLOW_OK) {
		/* something wrong, stop pushing */
		gzerr << "gst_app_src_push_buffer failed" << endl;
		g_main_loop_quit(this->gst_loop);
	}
}

void GstCamera::Configure(const Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    EntityComponentManager &_ecm,
                    EventManager &)
{
	this->dataPtr->sensor = Sensor(_entity);
	if(!this->dataPtr->sensor.Valid(_ecm))
	{
		gzerr << "GstCamera plugin should be attached to a sensor entity. "
			<< "Failed to initialize." << std::endl;
		return;
	}

	this->dataPtr->sdfConfig = _sdf->Clone();

	// std::string parentName = _ecm.ComponentData<components::Name>(
	// 	_parent->Data())->data;

	// get scene
	if (!this->scene)
	{
		this->scene = rendering::sceneFromFirstRenderEngine();
	}

	// return if scene not ready or no sensors available.
	if (!this->scene->IsInitialized() ||
		this->scene->SensorCount() == 0)
	{
		return;
	}

	// get camera
    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->NodeByIndex(i));
      if (cam && cam->HasUserData("user-camera") &&
          std::get<bool>(cam->UserData("user-camera")))
      {
        this->camera = cam;
        gzdbg << "GstCamera plugin is using camera ["
               << this->camera->Name() << "]" << std::endl;
        break;
      }
    }

	this->width = this->camera->ImageWidth();
	this->height = this->camera->ImageHeight();
	this->frameformat = this->camera->ImageFormat();
	this->depth = 0;
	this->rate = 60.0;

	this->udpHost = "127.0.0.1";
	const char *host_ip = std::getenv("PX4_VIDEO_HOST_IP");
	if (host_ip) {
		this->udpHost = std::string(host_ip);
	} else if (_sdf->HasElement("udpHost")) {
		this->udpHost = _sdf->Get<std::string>("udpHost");
	}

	this->udpPort = 5600;
	if (_sdf->HasElement("udpPort")) {
		this->udpPort = _sdf->Get<int>("udpPort");
	}

	gzwarn << "[gst_camera_plugin] Streaming video to ip: " << this->udpHost << " port: "  << this->udpPort << std::endl;

	if (_sdf->HasElement("rtmpLocation")) {
		this->rtmpLocation = _sdf->Get<std::string>("rtmpLocation");
		this->useRtmp = true;
	} else {
		this->useRtmp = false;
	}

	if (_sdf->HasElement("useCuda")) {
		this->useCuda = _sdf->Get<bool>("useCuda");
	} else {
		this->useCuda = false;
	}

	// components::Camera *cameraComponent = nullptr;
	// if(_ecm.EntityByComponents(components::Camera()) != kNullEntity){
	// 	Entity cameraEntity = _ecm.EntityByComponents(components::Camera());
	// 	cameraComponent = _ecm.Component<components::Camera>(cameraEntity);
	// }
	if (this->camera && this->camera->HasUserData("user-camera") &&
          std::get<bool>(this->camera->UserData("user-camera")))
		GstCamera::startStreaming();
	else
		GstCamera::stopStreaming();

	// And start by default
	GstCamera::startStreaming();

}

GZ_ADD_PLUGIN(GstCamera, System, GstCamera::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(GstCamera, "gz::sim::systems::GstCamera")
