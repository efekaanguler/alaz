// Copyright 2026 SDC Team
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

#include <memory>

#include "my_vehicle_interface/vehicle_interface_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node =
    std::make_shared<my_vehicle_interface::VehicleInterfaceNode>(options);

  RCLCPP_INFO(
    node->get_logger(),
    "SDC 2026 Kart Vehicle Interface starting...");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
