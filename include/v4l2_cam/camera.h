/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 ThundeRatz
 *
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

#ifndef V4L2_CAM_CAMERA_H
#define V4L2_CAM_CAMERA_H

#include <image_transport/image_transport.h>

#include <string>
#include <vector>

namespace v4l2_cam
{

class Camera
{
 public:
  class V4L2Image : public sensor_msgs::Image
  {
   public:
    V4L2Image(Camera& camera, int buffer_index) : camera(camera), buffer_index(buffer_index)
    {}

    virtual ~V4L2Image()
    {
      camera.enqueue(buffer_index);
    }

   private:
    Camera& camera;
    int buffer_index;
  };
  explicit Camera(const std::string& device);
  ~Camera();
  void set_format(const std::string& fourcc, unsigned width, unsigned height);
  void set_framerate(unsigned numerator, unsigned denominator);
  const boost::shared_ptr<V4L2Image> capture();
  void enqueue(int index);

 private:
  class Buffer
  {
   public:
    uint8_t *start;
    size_t length;
  };
  int fd_;
  unsigned bytesperline, width, height;
  std::vector<Buffer> buffers;

  void setup_transfer();
  boost::shared_ptr<V4L2Image> empty_image(int buffer_index);
  void list_controls();
  int print_control(struct v4l2_query_ext_ctrl *query_ext_ctrl);
  void enumerate_menu(const struct v4l2_query_ext_ctrl *query_ext_ctrl);
  void enumerate_integer_menu(const struct v4l2_query_ext_ctrl *query_ext_ctrl);
  int query_menu(struct v4l2_querymenu *menu);
  void list_formats();
  int print_format(struct v4l2_fmtdesc *fmtdesc);
};

}  // namespace v4l2_cam

#endif  // V4L2_CAM_CAMERA_H
