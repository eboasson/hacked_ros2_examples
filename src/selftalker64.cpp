// Copyright 2014 Open Source Robotics Foundation, Inc.
// Copyright 2020 Erik Boasson
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
  printf("Usage for selftalker64 app:\n");
  printf("selftalker64 [-t topic_name] [-s] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
  printf("-s : Publish pre-serialized messages.\n");
}

class Talker64 : public rclcpp::Node
{
public:
  explicit Talker64(const std::string & topic_name, size_t size, bool sermsg)
  : Node("selftalker")
  {
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this, size, sermsg]() -> void
      {
        uint64_t now = clock_.now().nanoseconds();
        msg_ = std::make_unique<std_msgs::msg::Int64MultiArray>();
        msg_->data.clear ();
        msg_->data.push_back ((int64_t) ((((uint64_t) count_ & 0x7fff) << 48) | (now & 0xffffffffffff)));
        msg_->data.resize (size);
        count_++;
        if (!sermsg)
          pub_->publish(std::move(msg_));
        else
        {
          rcl_serialized_message_t sermsg = rmw_get_zero_initialized_serialized_message ();
          auto allocator = rcutils_get_default_allocator();
          auto ret = rmw_serialized_message_init(&sermsg, 0, &allocator);
          if (ret != RCL_RET_OK) {
            throw std::runtime_error("failed to initialize serialized message");
          }
          auto ts =
            rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::Int64MultiArray>();
          // Given the correct typesupport, we can convert our ROS2 message into
          // its binary representation (serialized_msg)
          ret = rmw_serialize(msg_.get(), ts, &sermsg);
          if (ret != RMW_RET_OK) {
            fprintf(stderr, "failed to serialize serialized message\n");
            return;
          }
          pub_->publish(sermsg);
          ret = rmw_serialized_message_fini(&sermsg);
          if (ret != RCL_RET_OK) {
            fprintf(stderr, "could not clean up memory for serialized message");
          }
        }
      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(topic_name, qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::Int64MultiArray> msg_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;
};

class Listener64 : public rclcpp::Node
{
public:
  explicit Listener64(const std::string & topic_name)
  : Node("listener")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) -> void
      {
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
        }
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(topic_name, 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_;
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

  rclcpp::executors::MultiThreadedExecutor exec;
  auto ln = std::make_shared<Listener64>(topic);
  bool sermsg = rcutils_cli_option_exist(argv, argv + argc, "-s");
  printf("sermsg = %d\n", (int) sermsg);
  auto tn = std::make_shared<Talker64>(topic, 1, sermsg);
  exec.add_node (ln);
  exec.add_node (tn);
  
  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  exec.spin();

  exec.remove_node (tn);
  exec.remove_node (ln);
  rclcpp::shutdown();
  return 0;
}
