#include "driver_apds9960.h"
#include <chrono>
#include <cstdint>
using namespace std::chrono_literals;

/**
 * @brief How often the node will read and publish the IMU data
 */
constexpr std::chrono::duration PUBLISH_PERIOD = 100ms;

/**
 * @brief The duration over which the color reading ADCs will be
 * integrated. If the readings arent sensitive enough, turn this
 * value up. If the readings are oversensitive and saturating,
 * turn this value down.
 *
 * Range is between 2.78ms-712ms.
 */
constexpr std::chrono::milliseconds ADC_INTEGRATION_TIME = 100ms;

/**
 * @brief The gain applied to the ALS color sensor readings. Range
 * is between 1X and 64X.
 */
constexpr apds9960_als_color_gain_t COLOR_SENSOR_GAIN =
    APDS9960_ALS_COLOR_GAIN_64X;
