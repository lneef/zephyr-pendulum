#pragma once
#include <zephyr/kernel.h>
#include "driver.h"
struct pendulum_state {
  float x;      // cart position
  float dx;     // cart velocity
  float theta;  // pendulum angle
  float dtheta; // pendulum angular velocity

  float x_avg;      // averaged cart position
  float dx_avg;     // averaged cart velocity
  float theta_avg;  // averaged pendulum angle
  float dtheta_avg; // averaged pendulum angular velocity
  uint32_t k;       // length of data
};

struct control_interval {
  uint32_t su;
  uint32_t ct;
  float weight;
  float sx;
  float xdist;
};

void wait(const struct device *dev, struct sensor_state *sstate,
          struct pendulum_state *pstate, float dt, float force);

int32_t calibrate(const struct device* pd, const struct device* odrive, struct sensor_state* sstate);

void control(const struct device *pd, const struct device *odrive,
             struct pendulum_state *pstate, struct sensor_state *sstate,
             struct control_interval *con, uint32_t control_freq);
