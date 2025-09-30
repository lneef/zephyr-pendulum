#pragma once
#include "driver.h"
#include "zsl/vectors.h"
#include <zephyr/kernel.h>
#include <zsl/zsl.h>
#include <zsl/matrices.h>

#define WIDTH 4
typedef float vec4f[WIDTH];
typedef float mat4f[WIDTH][WIDTH];
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
  float su;
  uint32_t ct;
  float weight;
  float sx;
  float xdist;
};

struct matrix {
    vec4f k_hac;
    vec4f k_hpc;
    mat4f p_hac;
    mat4f p_hpc;
};

void init_matrix(struct matrix **mat);

void delete_matrix(struct matrix **mat);

void go_to(const struct device *pd, struct sensor_state *sstate,
           int32_t position);

void wait(const struct device *dev, struct sensor_state *sstate,
          struct pendulum_state *pstate, float dt, float force);

int32_t calibrate(const struct device *pd, struct sensor_state *sstate);

void control(const struct device *pd, struct pendulum_state *pstate,
             struct sensor_state *sstate, struct control_interval *con,
             uint32_t control_freq, float swing_up_str, float swing_up_smoother);
