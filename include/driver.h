#pragma once
#include "zephyr/device.h"
#include "zephyr/toolchain.h"
#include <stdint.h>
#define DT_DRV_COMPAT pendulum
struct sensor_state {
  int32_t pendulumAngle;
  int32_t cartPosition;
  int16_t buttons[2];
};

typedef int(*reset_fn)(const struct device*);
typedef int(*read_fn)(const struct device*, struct sensor_state*);
typedef int(*update_fn)(const struct device*, int32_t);

__subsystem struct pendulum_controller_driver_api{
    reset_fn reset;
    read_fn read;
    update_fn update_threshold;
};

#define CALL_API(name, dev) \
    const struct pendulum_controller_driver_api* api = (const struct pendulum_controller_driver_api*) dev->api; \
    api->##name##(dev, sstate); 

static inline void pd_read_api(const struct device* dev, struct sensor_state* sstate){
    const struct pendulum_controller_driver_api* api = (const struct pendulum_controller_driver_api*) dev->api;
    api->read(dev, sstate);
}

static inline void pd_update_threshold_api(const struct device* dev, int32_t val){
    const struct pendulum_controller_driver_api* api = (const struct pendulum_controller_driver_api*) dev->api;
    api->update_threshold(dev, val);
}

static inline void pd_reset_api(const struct device* dev){
    const struct pendulum_controller_driver_api* api = (const struct pendulum_controller_driver_api*) dev->api;
    api->reset(dev);
}
