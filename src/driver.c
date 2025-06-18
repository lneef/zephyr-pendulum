#include "driver.h"
#include "zephyr/arch/arm/cortex_a_r/sys_io.h"
#include "zephyr/sys/device_mmio.h"
#include "zephyr/sys/sys_io.h"
#include <arm_acle.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define PD_ENC_O_VAL 0
#define PD_ENC_0_RST 4
#define PD_ENC_1_VAL 8
#define PD_ENC_1_RST 12
#define PD_ENC_PWM 16
#define PD_BUT_L 24
#define PD_BUT_R 26

struct pd_data {
  DEVICE_MMIO_RAM
  struct device *dev;
  mm_reg_t mmio;
};

struct pd_config {
  DEVICE_MMIO_RAM
  int32_t rst[2];
};

static int pd_reset(const struct device *dev) {
  struct pd_data* data = (struct pd_data*)dev->data;  
  struct pd_config *cfg = (struct pd_config *)dev->config;
  sys_write32(cfg->rst[0], data->mmio + PD_ENC_0_RST);
  sys_write32(cfg->rst[0], data->mmio + PD_ENC_1_RST);
  return 0;
}

static int pd_read(const struct device *dev, struct sensor_state *sstate) {
  struct pd_data* data = (struct pd_data*)dev->data;
  sstate->cartPosition = sys_read32(data->mmio + PD_ENC_O_VAL);
  sstate->pendulumAngle = sys_read32(data->mmio + PD_ENC_1_VAL);
  sstate->buttons[0] = sys_read32(data->mmio + PD_BUT_L);
  sstate->buttons[1] = sys_read32(data->mmio + PD_BUT_R);
  return 0;
}

static int pd_update_threshold(const struct device *dev, int32_t threshold) {
  struct pd_data* data = (struct pd_data*)dev->data;
  sys_write32(threshold, data->mmio + PD_ENC_PWM);
  return 0;
}

static int pd_init(const struct device *dev) {
  struct pd_data* data = (struct pd_data*) dev->data;  
  DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
  data->mmio = DEVICE_MMIO_GET(dev);
  sys_write32(0, data->mmio + PD_ENC_0_RST);
  sys_write32(0, data->mmio + PD_ENC_1_RST);
  return 0;
}

static struct pendulum_controller_driver_api pd_api = {.reset = pd_reset,
                                                       .read = pd_read,
                                                       .update_threshold =
                                                           pd_update_threshold};
static struct pd_data pd_data = {0};
static struct pd_config pd_config = {
    .rst = {0, 0},
};
DEVICE_DT_DEFINE(DT_NODELABEL(pendulum0), pd_init, NULL, &pd_data, &pd_config,
                 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &pd_api);
