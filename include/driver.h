#pragma once
#include "zephyr/device.h"
#include "zephyr/toolchain.h"
#include <stdint.h>
#define DT_DRV_COMPAT pendulum
#define PERIOD 1999980
#define MAX_PWM 2.0f
#define MIN_PWM 1.0f
struct sensor_state {
  int32_t pendulumAngle;
  int32_t cartPosition;
  int16_t buttons[2];
};

typedef int (*reset_fn)(const struct device *);
typedef int (*read_fn)(const struct device *, struct sensor_state *);
typedef int (*update_fn)(const struct device *, int32_t);

__subsystem struct pendulum_controller_driver_api {
  reset_fn reset;
  read_fn read;
  update_fn update_threshold;
};

static inline float clamp(float val, float min, float max) {
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

static inline int pd_pwm_cfg(const struct device *dev, int32_t val) {
  const struct pendulum_controller_driver_api *api =
      DEVICE_API_GET(pendulum_controller, dev);
  return api->update_threshold(dev, val);
}

static inline void pd_read_api(const struct device *dev,
                               struct sensor_state *sstate) {
  const struct pendulum_controller_driver_api *api =
      (const struct pendulum_controller_driver_api *)dev->api;
  api->read(dev, sstate);
}

static inline void pd_update_threshold_api(const struct device *dev,
                                           float val) {
  float duty_cycle =(1.0f / 20.0f) * val + (MAX_PWM + MIN_PWM) / 2.0f;
  int32_t pwm = (int32_t)clamp(duty_cycle, MAX_PWM, MIN_PWM) / 20. * PERIOD;
  const struct pendulum_controller_driver_api *api =
      (const struct pendulum_controller_driver_api *)dev->api;
  api->update_threshold(dev, pwm);
}

static inline void pd_reset_api(const struct device *dev) {
  const struct pendulum_controller_driver_api *api =
      (const struct pendulum_controller_driver_api *)dev->api;
  api->reset(dev);
}
