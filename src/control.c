#include "control.h"
#include "driver.h"
#include "zephyr/device.h"
#include "zephyr/drivers/uart.h"
#include "zephyr/kernel.h"
#include "zephyr/sys_clock.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys_clock.h>

#define M_PI 3.14159265358979323846f
#define PULSES_PER_MIN (2048 * 4)
#define TO_RADIANS (2 * M_PI / PULSES_PER_MIN)
typedef float vec4f[4];

static const float to_radians(int32_t value) { return value * TO_RADIANS; }
static const int32_t normalize(int32_t value) {
  int32_t threshold = PULSES_PER_MIN >> 1;
  value = value % PULSES_PER_MIN;
  if (value > threshold) {
    value -= PULSES_PER_MIN;
  } else if (value <= -threshold) {
    value += PULSES_PER_MIN;
  }
  return value;
}

static float dot(const vec4f *v1, const vec4f *v2) {
  float out = 0.0;
  for (int i = 0; i < 4; ++i) {
    out += (*v1)[i] * (*v2)[i];
  }
  return out;
}

static void pendulum_update(struct pendulum_state *pstate,
                            const struct sensor_state *state, const float dt,
                            const float force) {
  float position = to_radians(state->cartPosition);
  float velocity = (position - pstate->k) / dt;
  float angle = to_radians(normalize(state->pendulumAngle));
  float angular_velocity = (angle - pstate->theta) / dt;

  pstate->x = position;
  pstate->dx = velocity;
  pstate->theta = angle;
  pstate->dtheta = angular_velocity;
}

static float compute_control(struct pendulum_state *pstate, const float force,
                             vec4f *m) {
  vec4f temp_x = {pstate->x, pstate->dx, pstate->theta, pstate->dtheta};
  return dot(&temp_x, m) * 0.014f;
}

static int sign(const float x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}

static float get_swing_up(struct pendulum_state *pstate, float swing_up_str) {
  return swing_up_str *
         (((1.0f / 6) * 0.0374f * pstate->dtheta * pstate->dtheta) +
          0.7443f * (1 - cosf(pstate->theta))) *
         sign(pstate->dtheta * cosf(pstate->theta));
}

static float chooseController(struct pendulum_state *pstate, float force,
                              vec4f *k_hac, vec4f *k_hpc, float swing_up_str,
                              struct control_interval *con) {

  if (fabsf(pstate->theta) < con->sx) {
    if (fabsf(pstate->x) < con->xdist) {
      return compute_control(pstate, force, k_hpc);
    } else {
      return compute_control(pstate, force, k_hac);
    }
  } else {
    return get_swing_up(pstate, swing_up_str);
  }
}

static void uart_write(const struct device *odrive, const char *buf,
                       size_t len) {
  size_t i;
  for (i = 0; i < len; ++i) {
    uart_poll_out(odrive, buf[i]);
  }
}

static void init_odrive(const struct device *odrive) {
  const char calibration[] = "w axis0.requested_state 3\n";
  const char loop_control[] = "w axis0.requested_state 8\n";
  uart_write(odrive, calibration, sizeof(calibration));
  uart_write(odrive, loop_control, sizeof(loop_control));
}

static void set_torque(const struct device *odrive, float torque) {
  char buf[128];
  snprintf(buf, sizeof(buf), "c 0 %f\n", (double)torque);
  uart_write(odrive, buf, strlen(buf));
}

static void set_max_vel(const struct device *odrive, float value) {
  char buf[128];
  snprintf(buf, sizeof(buf), "w axis0.controller.config.vel_limit %f\n",
           (double)value);
  uart_write(odrive, buf, strlen(buf));
}

void go_to(const struct device *pd, const struct device *odrive,
           struct sensor_state *sstate, int32_t position) {
  do {
    pd_read_api(pd, sstate);
    if (sstate->cartPosition > position) {
      set_torque(odrive, -0.2);
    } else if (sstate->cartPosition < position) {
      set_torque(odrive, 0.2);
    }
  } while (abs(sstate->cartPosition - position) > 16);
  set_torque(odrive, 0.0);
}

// for init
// do not call from main control loop
void wait(const struct device *dev, struct sensor_state *sstate,
          struct pendulum_state *pstate, float dt, float force) {
  {
    float old_theta = 90000;
    while (true) {
      k_timeout_t timeout = K_USEC(1000000);
      if (old_theta == pstate->theta) {
        break;
      }
      old_theta = pstate->theta;
      pd_read_api(dev, sstate);
      k_sleep(timeout);
      pendulum_update(pstate, sstate, dt, force);
    }
  }
}

static bool close(int32_t v1, int32_t v2) {
#define threshold 128
  return abs(v1 - v2) < threshold;
}

int32_t calibrate(const struct device *pd, const struct device *odrive,
                  struct sensor_state *sstate) {
  init_odrive(odrive);
  do {
    pd_read_api(pd, sstate);
    set_torque(odrive, 0.2);
  } while (!sstate->buttons[0]);
  set_torque(odrive, 0.0);
  int32_t cart_position = sstate->cartPosition;

  do {
    pd_read_api(pd, sstate);
    set_torque(odrive, -0.2);
  } while (!sstate->buttons[1]);
  set_torque(odrive, 0.0);
  int32_t cart_position2 = sstate->cartPosition;
  int32_t center = (cart_position + cart_position2) >> 1ll;

  do {
    pd_read_api(pd, sstate);
    set_torque(odrive, 0.2);
  } while (!close(center, sstate->cartPosition));
  set_torque(odrive, 0.0);
  return center;
}

void control(const struct device *pd, const struct device *odrive,
             struct pendulum_state *pstate, struct sensor_state *sstate,
             struct control_interval *con, uint32_t control_freq) {
  vec4f K_HAC = {5.8, 1.6, -855.0, -30.0};
  vec4f K_HPC = {5.8, 2.0, -285.0, -22.0};
  float u = 0.0, swing_up_str = 0.181, elapsed = control_freq, maxVel = 8.0;
  k_timeout_t timer, wait;
  k_timepoint_t left;
  set_max_vel(odrive, maxVel);
  set_torque(odrive, 1.0);
  timer = K_USEC(control_freq);
  while (1) {
    left = sys_timepoint_calc(timer);
    wait = sys_timepoint_timeout(left);
    k_sleep(wait);
    timer = K_USEC(control_freq);
    pd_read_api(pd, sstate);
    pendulum_update(pstate, sstate, elapsed, u);
    if(sstate->buttons[0] || sstate->buttons[1])
        break;
    u = chooseController(pstate, u, &K_HAC, &K_HPC, swing_up_str, con);
    set_torque(odrive, u);
  }
}
