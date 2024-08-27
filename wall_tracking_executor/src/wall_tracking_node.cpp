// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "wall_tracking_executor/wall_tracking_executor.hpp"

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
  	auto node = std::make_shared<WallTracking::WallTracking>();
  	rclcpp::spin(node);
  	rclcpp::shutdown();
  	return 0;
}
