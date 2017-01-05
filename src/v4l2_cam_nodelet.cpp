/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 ThundeRatz

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <thread>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "v4l2_cam/camera.h"

namespace v4l2_cam
{
class V4L2CamNodelet : public nodelet::Nodelet
{
 public:
  virtual void onInit()
  {
    ros::NodeHandle& node = getPrivateNodeHandle();

    image_transport::ImageTransport transport(node);
    publisher = transport.advertise("image", 1);

    camera_thread = new std::thread(image_fetch_thread, &publisher);
  }

  ~V4L2CamNodelet()
  {
    camera_thread->join();
    delete camera_thread;
  }

 private:
  std::thread *camera_thread;
  image_transport::Publisher publisher;

  static void image_fetch_thread(image_transport::Publisher *publisher)
  {
    Camera camera("/dev/video0");
    while (ros::ok())
    {
      const boost::shared_ptr<Camera::V4L2Image> image = camera.capture();
      if (image != nullptr)
        publisher->publish(image);
    }
  }
};
}  // namespace v4l2_cam

PLUGINLIB_EXPORT_CLASS(v4l2_cam::V4L2CamNodelet, nodelet::Nodelet)
