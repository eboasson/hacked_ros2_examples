// Copyright 2021 Erik Boasson
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
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  auto c1 = rclcpp::Context::make_shared();
  auto co1 = rclcpp::InitOptions();
  co1.set_domain_id(1);
  c1->init(argc, argv, co1);
  auto no1 = rclcpp::NodeOptions();
  no1.context(c1);
  auto n1 = std::make_shared<rclcpp::Node>("kwik","",no1);
  
  auto c2 = rclcpp::Context::make_shared();
  rclcpp::InitOptions co2;
  co2.set_domain_id(1);
  c2->init(argc, argv, co2);
  auto no2 = rclcpp::NodeOptions();
  no2.context(c2);
  auto n2 = std::make_shared<rclcpp::Node>("cadmium","",no2);

  auto c3 = rclcpp::Context::make_shared();
  rclcpp::InitOptions co3;
  co3.set_domain_id(3);
  c3->init(argc, argv, co3);
  auto no3 = rclcpp::NodeOptions();
  no3.context(c3);
  auto n3 = std::make_shared<rclcpp::Node>("zink","",no3);

  std::this_thread::sleep_for(1s);
  return 0;
}
