// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("pointcloud");
  auto node = new rclcpp::Node("pointcloud");

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", qos);

  // Points are 32 bytes here, so twice the size of Adam's points.
  // his 1M points -> my .5M
  const uint32_t w = 724;
  const uint32_t h = 724;
  sensor_msgs::msg::PointCloud2 m;
  m.height = h;
  m.width = w;
  sensor_msgs::PointCloud2Modifier mm(m);
  mm.setPointCloud2FieldsByString(2, "xyz", "rgb");
  mm.reserve(w * h);

  // Define some raw data we'll put in the PointCloud2
  const float point_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  const uint8_t color_data[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
  const auto ndata = sizeof (point_data) / sizeof (point_data[0]);
  // Define the iterators. When doing so, you define the Field you would like to iterate upon and
  // the type of you would like returned: it is not necessary the type of the PointField as sometimes
  // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
  sensor_msgs::PointCloud2Iterator<float> iter_x(m, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(m, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(m, "z");
  // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
  // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
  // and RGBA as A,R,G,B)
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(m, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(m, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(m, "b");
  // Fill the PointCloud2
  for(size_t i = 0; i < w * h; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    *iter_x = point_data[(3*i+0) % ndata];
    *iter_y = point_data[(3*i+1) % ndata];
    *iter_z = point_data[(3*i+2) % ndata];
    *iter_r = color_data[(3*i+0) % ndata];
    *iter_g = color_data[(3*i+1) % ndata];
    *iter_b = color_data[(3*i+2) % ndata];
  }

  rclcpp::Clock clock;
  const uint64_t tstart = clock.now().nanoseconds();
  uint64_t tprint = tstart + 1000000000;
  uint32_t count = 0;
  while (rclcpp::ok())
  {
    pub->publish(m);
    count++;

    const uint64_t tnow = clock.now().nanoseconds();
    if (tnow >= tprint)
    {
      RCLCPP_INFO(node->get_logger(), "%.3f %u", (double) (tnow - tstart) / 1e9, count);
      tprint += 1000000000;
      if (tprint < tnow)
        tprint = tnow + 1000000000;
      count = 0;
    }
  };

  rclcpp::shutdown();
  return 0;
}
