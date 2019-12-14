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
#include <unistd.h>

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
class Roundtrip64 : public rclcpp::Node
{
public:
  explicit Roundtrip64(const std::string topic_name, size_t size, bool isping)
    : Node(isping ? "roundtrip64A" : "roundtrip64B")
  {
    // Create a function for when messages are to be sent.
    auto do_ping =
      [this, size]() -> void {
        uint64_t now = clock_.now().nanoseconds();
        msg_ = std::make_unique<std_msgs::msg::Int64MultiArray>();
        msg_->data.clear ();
        msg_->data.push_back ((int64_t) ((((uint64_t) count_ & 0x7fff) << 48) | (now & 0xffffffffffff)));
        msg_->data.resize (size);
        count_++;
        pub_->publish(std::move(msg_));
      };

    auto callback_ping =
      [this, do_ping](const std_msgs::msg::Int64MultiArray::SharedPtr msg) -> void {
        uint64_t now = clock_.now().nanoseconds();
        int64_t data = msg->data[0];
        if (prev_)
        {
          uint32_t pseq = (prev_ >> 48);
          uint32_t nseq = (data >> 48);
          lost += ((nseq <= pseq) ? 32768 : 0) + nseq - pseq - 1;
          rcvd++;
          uint64_t dt = ((int64_t) (now << 16) - (data << 16)) >> 16;
          if (dt < minlat) minlat = dt;
          latsum += dt;
        }
        prev_ = data;
        if (tprint + 1000000000 < now)
        {
          RCLCPP_INFO(this->get_logger(), "rcvd %llu lost %llu minlat %.3fus avglat %.3fus", rcvd, lost, (double) minlat / 1e3, (double) latsum / (double) rcvd / 1e3);
          rcvd = lost = latsum = 0;
          minlat = UINT64_MAX;
          tprint = now;

          auto names = this->get_node_names ();
          for (auto&&n : names) { RCLCPP_INFO(this->get_logger(), "%s", n.c_str ()); }
        }
        do_ping ();
      };
    
    auto callback_pong =
      [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) -> void {
        pub_->publish(*msg);
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    if (isping)
      sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(topic_name + "A", 10, callback_ping);
    else
      sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(topic_name + "B", 10, callback_pong);

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(topic_name + (isping ? "B" : "A"), qos);

    // Use a timer to schedule periodic message publishing.
    //timer_ = this->create_wall_timer(1ms, publish_message);
    if (isping) { sleep (1); do_ping (); }
  }

private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::Int64MultiArray> msg_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_;
  uint64_t prev_ = 0;
  uint64_t lost = 0;
  uint64_t rcvd = 0;
  uint64_t latsum = 0;
  uint64_t minlat = UINT64_MAX;
  uint64_t tprint = 0;
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

  bool isping = rcutils_cli_option_exist(argv, argv + argc, "-p");
  printf ("isping = %d\n", (int) isping);
  
  // Create a node.
  auto node = std::make_shared<Roundtrip64>(topic, size, isping);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
