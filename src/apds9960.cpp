#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <thread>

#include "apds9960/msg/color_proximity.hpp"
#include "apds9960_cfg.hpp"
#include "driver_apds9960.h"
#include "i2c/i2c.h"
#include "rclcpp/rclcpp.hpp"

#define DRIVER_CONFIG_ERR_HANDLE(conf_func, fail_msg)                          \
  do {                                                                         \
    uint8_t res = conf_func;                                                   \
    if (res != 0) {                                                            \
      RCLCPP_FATAL(this->get_logger(), "Couldn't " fail_msg ", Error = 0x%x",  \
                   res);                                                       \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)

#define DRIVER_SET_CONF(conf_name, enable, fail_msg)                           \
  DRIVER_CONFIG_ERR_HANDLE(                                                    \
      apds9960_set_conf(&driver_handle, APDS9960_CONF_##conf_name,             \
                        static_cast<apds9960_bool_t>(enable)),                 \
      fail_msg)

namespace {

template <std::unsigned_integral T> constexpr float uint_to_0_1_float(T val) {
  return static_cast<float>(val) /
         static_cast<float>(std::numeric_limits<T>::max());
}

} // namespace

class APDS9960Node : public rclcpp::Node {
public:
  APDS9960Node() : Node("apds9960_node") {

    this->declare_parameter<std::string>("dev_path", "/dev/i2c-7");
    this->declare_parameter<std::string>("frame_id", "color_sensor");
    this->declare_parameter<std::string>("topic", "/color_sensor");
    this->declare_parameter<uint8_t>("i2c_addr", 0x39);

    this->get_parameter("dev_path", I2C_DEVICE_PATH);
    this->get_parameter("i2c_addr", I2C_DEVICE_ADDR);
    this->get_parameter("frame_id", ROS_FRAME_ID);
    this->get_parameter("topic", ROS_TOPIC_NAME);

    // display set parameters
    RCLCPP_INFO(this->get_logger(), "dev_path: %s", I2C_DEVICE_PATH.c_str());
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", ROS_FRAME_ID.c_str());
    RCLCPP_INFO(this->get_logger(), "topic: %s", ROS_TOPIC_NAME.c_str());
    RCLCPP_INFO(this->get_logger(), "i2c_addr: %X\n", I2C_DEVICE_ADDR);

    configure_driver();

    ros_publisher = this->create_publisher<apds9960::msg::ColorProximity>(
        ROS_TOPIC_NAME, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Started apds9960 Node");

    ros_timer = this->create_wall_timer(PUBLISH_PERIOD, [this] {
      if (auto x = make_message(); x != nullptr) {
        ros_publisher->publish(std::move(x));
      }
    });
  }

  ~APDS9960Node() {
    apds9960_deinit(&driver_handle);
    RCLCPP_INFO(this->get_logger(), "Successfully deinit'ed APDS9960");
  }

private:
  I2CDevice i2c_dev;
  apds9960_handle_t driver_handle;
  rclcpp::TimerBase::SharedPtr ros_timer;
  rclcpp::Publisher<apds9960::msg::ColorProximity>::SharedPtr ros_publisher;

  std::string I2C_DEVICE_PATH;
  uint8_t I2C_DEVICE_ADDR;
  std::string ROS_FRAME_ID;
  std::string ROS_TOPIC_NAME;

  void configure_driver() {
    DRIVER_APDS9960_LINK_INIT(&driver_handle, apds9960_handle_t);
    DRIVER_APDS9960_LINK_IIC_INIT(&driver_handle, driver_i2c_init);
    DRIVER_APDS9960_LINK_IIC_DEINIT(&driver_handle, driver_i2c_deinit);
    DRIVER_APDS9960_LINK_IIC_READ(&driver_handle, driver_i2c_read);
    DRIVER_APDS9960_LINK_IIC_WRITE(&driver_handle, driver_i2c_write);
    DRIVER_APDS9960_LINK_DELAY_MS(&driver_handle, driver_delay_ms);
    DRIVER_APDS9960_LINK_DEBUG_PRINT(&driver_handle, driver_debug_print);
    DRIVER_APDS9960_LINK_RECEIVE_CALLBACK(&driver_handle,
                                          driver_interrupt_callback);
    DRIVER_APDS9960_LINK_CONTEXT(&driver_handle, this);

    if (uint8_t res = apds9960_init(&driver_handle); res != 0) {
      RCLCPP_FATAL(this->get_logger(), "Couldn't init APDS9960: Error = 0x%x",
                   res);
      exit(1);
    }

    DRIVER_SET_CONF(POWER_ON, false, "power chip off for reset");
    DRIVER_SET_CONF(POWER_ON, true, "power chip on");
    DRIVER_SET_CONF(WAIT_ENABLE, false, "disable wait");
    DRIVER_SET_CONF(PROXIMITY_DETECT_ENABLE, true, "enable prox detection");
    DRIVER_SET_CONF(ALS_ENABLE, true, "enable color/ambient light sensor");
    DRIVER_SET_CONF(ALS_INTERRUPT_ENABLE, false, "disable ALS interrupt");
    DRIVER_SET_CONF(PROXIMITY_INTERRUPT_ENABLE, false, "disable prox intr");
    DRIVER_SET_CONF(GESTURE_ENABLE, false, "disable gesture recognition");

    uint8_t adc_int_time_reg;
    DRIVER_CONFIG_ERR_HANDLE(
        apds9960_adc_integration_time_convert_to_register(
            &driver_handle, ADC_INTEGRATION_TIME.count(), &adc_int_time_reg),
        "convert adc integration time to reg");
    DRIVER_CONFIG_ERR_HANDLE(
        apds9960_set_adc_integration_time(&driver_handle, adc_int_time_reg),
        "set adc integration time");

    DRIVER_CONFIG_ERR_HANDLE(
        apds9960_set_als_color_gain(&driver_handle, COLOR_SENSOR_GAIN),
        "set color sensor gain");

    // Let it settle
    // TODO: Maybe replace with a loop to check for AVALID & PVALID
    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "Successfully configured APDS9960");
  }

  std::unique_ptr<apds9960::msg::ColorProximity> make_message() {
    uint8_t apds_status;
    apds9960_get_status(&driver_handle, &apds_status);
    if (!(apds_status & (1 << APDS9960_STATUS_AVALID))) {
      RCLCPP_WARN(this->get_logger(), "AVALID is NOT set: Status = 0x%x",
                  apds_status);
      return nullptr;
    }

    auto ret = std::make_unique<apds9960::msg::ColorProximity>();
    ret->header.stamp = this->get_clock()->now();
    ret->header.frame_id = ROS_FRAME_ID;

    uint16_t r, g, b, c;
    if (auto res = apds9960_read_rgbc(&driver_handle, &r, &g, &b, &c);
        res != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Aborting publish: Couldn't read RGBC, Error = 0x%x", res);
      return nullptr;
    }

    uint8_t proximity;
    if (auto res = apds9960_read_proximity(&driver_handle, &proximity);
        res != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Aborting publish: Couldn't read prox, Error = 0x%x", res);
      return nullptr;
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "raw data: r=%u, g=%u, b=%u, c=%u, prox=%u", r, g, b, c,
                 proximity);

    ret->color.r = uint_to_0_1_float(r);
    ret->color.g = uint_to_0_1_float(g);
    ret->color.b = uint_to_0_1_float(b);
    ret->color.a = uint_to_0_1_float(c);
    ret->proximity = uint_to_0_1_float(proximity);

    return ret;
  }

  static uint8_t driver_i2c_init(void* ctx) { 
    auto* this_ = reinterpret_cast<APDS9960Node*>(ctx);

    int i2c_fd = i2c_open(this_->I2C_DEVICE_PATH.c_str());
    if (i2c_fd < 0) {
      RCLCPP_FATAL(this_->get_logger(), "Could not open i2c device");
      exit(1);
      return 1;
    }

    this_->i2c_dev.bus = i2c_fd;
    this_->i2c_dev.addr = this_->I2C_DEVICE_ADDR;
    this_->i2c_dev.iaddr_bytes = 1;
    this_->i2c_dev.page_bytes = 256;

    return 0;
  }

  static uint8_t driver_i2c_deinit(void* ctx) {
    auto* this_ = reinterpret_cast<APDS9960Node*>(ctx);

    i2c_close(this_->i2c_dev.bus);

    return 0;
  }

  static uint8_t driver_i2c_read(void* ctx, [[maybe_unused]] uint8_t addr, uint8_t reg,
                                 uint8_t *buf, uint16_t len) {
    auto* this_ = reinterpret_cast<const APDS9960Node*>(ctx);

    if (i2c_ioctl_read(&this_->i2c_dev, reg, buf, len) < 0) {
      return 1;
    }

    return 0;
  }

  static uint8_t driver_i2c_write(void* ctx, [[maybe_unused]] uint8_t addr, uint8_t reg,
                                  uint8_t *buf, uint16_t len) {
    auto* this_ = reinterpret_cast<const APDS9960Node*>(ctx);

    if (i2c_ioctl_write(&this_->i2c_dev, reg, buf, len) < 0) {
      return 1;
    }

    return 0;
  }

  static void driver_delay_ms([[maybe_unused]] void* ctx, uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  static void driver_debug_print(void* ctx, const char *const fmt, ...) {
    // Because this stupid driver doesn't give context to callbacks, we
    // can't get the ROS logger so just have to printf it... stupid stupid
    auto* this_ = reinterpret_cast<const APDS9960Node*>(ctx);
    
    constexpr size_t MAX_SIZE = 1024;
    std::string buf;
    buf.reserve(MAX_SIZE);
    
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf.data(), MAX_SIZE, fmt, args);
    va_end(args);

    RCLCPP_INFO(this_->get_logger(), "%s", buf.data());
  }

  static void driver_interrupt_callback([[maybe_unused]] void *ctx, [[maybe_unused]] uint8_t type) {
    // Not using interrupts, so shouldn't have to implement this
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APDS9960Node>());
  rclcpp::shutdown();
}
