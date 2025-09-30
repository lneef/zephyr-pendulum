#include "control.h"
#include "driver.h"
#include "zephyr/device.h"
#include "zephyr/kernel.h"
#include "zephyr/sys_clock.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys_clock.h>

#define M_PI 3.14159265358979323846f
#define PULSES_PER_MIN (2048 * 4)
#define TO_RADIANS (2 * M_PI / PULSES_PER_MIN)
#define MOTOR_RADIUS 0.0205f

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
  float position = to_radians(state->cartPosition) * MOTOR_RADIUS;
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
  return dot(&temp_x, m) * MOTOR_RADIUS;
}

static bool close(int32_t v1, int32_t v2) {
#define threshold 128
  return abs(v1 - v2) < threshold;
}

static float quadform(const vec4f *v, const mat4f *m){
    float sum = 0.0f;
    for(int i = 0; i < WIDTH; ++i){
        for(int j = 0; j < WIDTH; ++j){
            sum += (*v)[i] * (*m)[i][j] * (*v)[j];
        }
    }
    return sum;
}

static int sign(const float x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}

static float get_swing_up(struct pendulum_state *pstate, float swing_up_str, float swing_up_smoother) {
    return swing_up_str *
             (((0.06875f * pstate->dtheta * pstate->dtheta) + 2.69775f * (1.0f - cosf(pstate->theta))) *
              sign(pstate->dtheta * cosf(pstate->theta))) -
         swing_up_smoother * pstate->dtheta;

}

static float chooseController(struct pendulum_state *pstate, float force, struct matrix *m,
                              uint32_t* control_freq, float swing_up_str, float swing_up_smoother,
                              struct control_interval *con) {
   const vec4f tempState = {pstate->x, pstate->dx, pstate->theta, pstate->dtheta}; 
   if (fabsf(tempState[2]) > 0.6f) {
       *control_freq = con->su;
    return get_swing_up(pstate, swing_up_str, swing_up_smoother);
  } else if (quadform(&tempState, &m->p_hpc) < 1) {
    *control_freq = con->ct;
    return compute_control(pstate, force, &m->k_hpc);
  } else if (quadform(&tempState, &m->p_hac) < 1) {
    *control_freq = con->ct;
    return compute_control(pstate, force, &m->k_hac);
  } else {
    *control_freq = con->su;
    return get_swing_up(pstate, swing_up_str, swing_up_smoother);
  }
}

void go_to(const struct device *pd, struct sensor_state *sstate, int32_t position) {
  do {
    pd_read_api(pd, sstate);
    if (sstate->cartPosition > position) {
      pd_update_threshold_api(pd, -0.2);  
    } else if (sstate->cartPosition < position) {
      pd_update_threshold_api(pd, 0.2);
    }
  } while (abs(sstate->cartPosition - position) > 16);
  pd_update_threshold_api(pd, 0.0);
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

void init_matrix(struct matrix **mat){
    *mat = malloc(sizeof(struct matrix));
    if(*mat == NULL){
        printk("Failed to allocate memory for matrix\n");
        exit(1);
    }
    (*mat)->k_hac[0] = 65.0;
    (*mat)->k_hac[1] = 112.0;
    (*mat)->k_hac[2] = -370.2;
    (*mat)->k_hac[3] = -38.5;
    (*mat)->k_hpc[0] = 58.37;
    (*mat)->k_hpc[1] = 80.74;
    (*mat)->k_hpc[2] = -319.45;
    (*mat)->k_hpc[3] = -27.29;
    
    (*mat)->p_hac[0][0] = 8.15119469;
    (*mat)->p_hac[0][1] = 2.8266167;
    (*mat)->p_hac[0][2] = -7.57358622;
    (*mat)->p_hac[0][3] = -0.96052152;

    (*mat)->p_hac[1][0] = 2.8266167;
    (*mat)->p_hac[1][1] = 3.7290751;
    (*mat)->p_hac[1][2] = -9.89565939;
    (*mat)->p_hac[1][3] = -1.2430829;

    (*mat)->p_hac[2][0] = -7.57358622;
    (*mat)->p_hac[2][1] = -9.89565939;
    (*mat)->p_hac[2][2] = 31.67548976;
    (*mat)->p_hac[2][3] = 3.36267636;

    (*mat)->p_hac[3][0] = -0.96052152;
    (*mat)->p_hac[3][1] = -1.2430829;
    (*mat)->p_hac[3][2] = 3.36267636;
    (*mat)->p_hac[3][3] = 0.43045142;

    (*mat)->p_hpc[0][0] = 2.71217664e+02;
    (*mat)->p_hpc[0][1] = 1.17166609e+01;
    (*mat)->p_hpc[0][2] = -4.22353808e+01;
    (*mat)->p_hpc[0][3] = -3.65636321e+00;

    (*mat)->p_hpc[1][0] = 1.17166609e+01;
    (*mat)->p_hpc[1][1] = 1.99733763e+00;
    (*mat)->p_hpc[1][2] = -7.12305739e+00;
    (*mat)->p_hpc[1][3] = -5.98773670e-01;

    (*mat)->p_hpc[2][0] = -4.22353808e+01;
    (*mat)->p_hpc[2][1] = -7.12305739e+00;
    (*mat)->p_hpc[2][2] = 2.94670562e+01;
    (*mat)->p_hpc[2][3] = 2.21976362e+00;

    (*mat)->p_hpc[3][0] = -3.65636321e+00;
    (*mat)->p_hpc[3][1] = -5.98773670e-01;
    (*mat)->p_hpc[3][2] = 2.21976362e+00;
    (*mat)->p_hpc[3][3] = 1.98118654e-01;

}

void delete_matrix(struct matrix **mat){
    free(*mat);
    *mat = NULL;
}

int32_t calibrate(const struct device *pd, struct sensor_state *sstate) {
  do {
    pd_read_api(pd, sstate);
    pd_update_threshold_api(pd, 0.2);
  } while (!sstate->buttons[0]);
  pd_update_threshold_api(pd, 0.0);
  int32_t cart_position = sstate->cartPosition;

  do {
    pd_read_api(pd, sstate);
    pd_update_threshold_api(pd, -0.2);
  } while (!sstate->buttons[1]);
  pd_update_threshold_api(pd, 0.0);
  int32_t cart_position2 = sstate->cartPosition;
  int32_t center = (cart_position + cart_position2) >> 1ll;

  do {
    pd_read_api(pd, sstate);
    pd_update_threshold_api(pd, 0.2);
  } while (!close(center, sstate->cartPosition));
  pd_update_threshold_api(pd, 0.0);
  return center;
}

void control(const struct device *pd, struct pendulum_state *pstate, struct sensor_state *sstate,
             struct control_interval *con, uint32_t control_freq, float swing_up_str, float swing_up_smoother) {
  float u = 0.0, elapsed = control_freq;
  struct matrix *m;
  init_matrix(&m);
  k_timeout_t timer, wait;
  k_timepoint_t left;
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
    u = chooseController(pstate, u,  m, &control_freq, swing_up_str, swing_up_smoother, con);
    pd_update_threshold_api(pd, u);
  }
    delete_matrix(&m);
}
