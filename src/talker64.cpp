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

#include "std_msgs/msg/int64_multi_array.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for talker64 app:\n");
  printf("talker64 [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker64 : public rclcpp::Node
{
public:
  explicit Talker64(const std::string & topic_name, size_t size)
  : Node("talker64")
  {
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this, size]() -> void
      {
        uint64_t now = clock_.now().nanoseconds();
        msg_ = std::make_unique<std_msgs::msg::Int64MultiArray>();
        msg_->data.clear ();
        msg_->data.push_back ((int64_t) ((((uint64_t) count_ & 0x7fff) << 48) | (now & 0xffffffffffff)));
        msg_->data.resize (size);
        count_++;
        pub_->publish(std::move(msg_));
      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(topic_name, qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1ms, publish_message);
  }

private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::Int64MultiArray> msg_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;
};

int main(int argc, char * argv[])
{
  size_t size = 1;
  
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
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

  cli_option = rcutils_cli_get_option(argv, argv + argc, "-z");
  if (nullptr != cli_option) {
    size = (size_t) std::stoi(cli_option);
  }
  
  // Create a node.
  auto node = std::make_shared<Talker64>(topic, size);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
