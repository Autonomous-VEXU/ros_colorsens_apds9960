#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <thread>

#include "driver_apds9960.h"
#include "i2c/i2c.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  //   rclcpp::spin(std::make_shared<LSM6DSV16XNode>());
  rclcpp::shutdown();
}
