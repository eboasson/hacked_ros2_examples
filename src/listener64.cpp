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

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "hacked_demo/msg/test.hpp"

void print_usage()
{
  printf("Usage for listener64 app:\n");
  printf("listener64 [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
}

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener64 : public rclcpp::Node
{
public:
  explicit Listener64(const std::string & topic_name)
  : Node("listener")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const hacked_demo::msg::Test::SharedPtr msg) -> void
      {
        uint64_t now = clock_.now().nanoseconds();
        if (prev_)
        {
          uint32_t pseq = (prev_ >> 48);
          uint32_t nseq = (msg->info >> 48);
          lost += ((nseq <= pseq) ? 32768 : 0) + nseq - pseq - 1;
          rcvd++;
          uint64_t dt = ((int64_t) (now << 16) - (msg->info << 16)) >> 16;
          if (dt < minlat) minlat = dt;
          latsum += dt;
        }
        prev_ = msg->info;
        if (tprint + 1000000000 < now)
        {
          RCLCPP_INFO(this->get_logger(), "rcvd %llu lost %llu minlat %.3fus avglat %.3fus", rcvd, lost, (double) minlat / 1e3, (double) latsum / (double) rcvd / 1e3);
          rcvd = lost = latsum = 0;
          minlat = UINT64_MAX;
          tprint = now;
        }
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<hacked_demo::msg::Test>(topic_name, 10, callback);
  }

private:
  rclcpp::Subscription<hacked_demo::msg::Test>::SharedPtr sub_;
  rclcpp::Clock clock_;
  uint64_t prev_ = 0;
  uint64_t lost = 0;
  uint64_t rcvd = 0;
  uint64_t latsum = 0;
  uint64_t minlat = UINT64_MAX;
  uint64_t tprint = 0;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("chatter64");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }

  // Create a node.
  auto node = std::make_shared<Listener64>(topic);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
