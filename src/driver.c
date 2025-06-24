#include "driver.h"
#include "zephyr/arch/arm/cortex_a_r/sys_io.h"
#include <arm_acle.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(vnd_sensor, LOG_LEVEL_DBG);

#define PD_ENC_O_VAL 0
#define PD_ENC_0_RST 4
#define PD_ENC_1_VAL 8
#define PD_ENC_1_RST 12
#define PD_ENC_PWM 16
#define PD_BUT_L 24
#define PD_BUT_R 26

struct pd_data {
  DEVICE_MMIO_RAM;
};

struct pd_config {
  DEVICE_MMIO_ROM;
};

static int pd_reset(const struct device *dev) {
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);
  sys_write32(0, mmio + PD_ENC_0_RST);
  sys_write32(0, mmio + PD_ENC_1_RST);
  return 0;
}

static int pd_read(const struct device *dev, struct sensor_state *sstate) {
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);
  sstate->cartPosition = sys_read32(mmio + PD_ENC_O_VAL);
  sstate->pendulumAngle = sys_read32(mmio + PD_ENC_1_VAL);
  sstate->buttons[0] = sys_read32(mmio + PD_BUT_L);
  sstate->buttons[1] = sys_read32(mmio + PD_BUT_R);
  LOG_INF("Reading from %lx: %d %d %d %d", mmio, sstate->cartPosition,
          sstate->pendulumAngle, sstate->buttons[0], sstate->buttons[1]);
  return 0;
}

static int pd_update_threshold(const struct device *dev, int32_t threshold) {
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);
  sys_write32(threshold, mmio + PD_ENC_PWM);
  return 0;
}

static int pd_init(const struct device *dev) {
  DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);
  LOG_INF("Allocating MMIO Region at %lx\\nn", mmio);
  sys_write32(0, mmio + PD_ENC_0_RST);
  sys_write32(0, mmio + PD_ENC_1_RST);
  return 0;
}

static struct pendulum_controller_driver_api pd_api = {.reset = pd_reset,
                                                       .read = pd_read,
                                                       .update_threshold =
                                                           pd_update_threshold};
static struct pd_data pd_data = {};
static struct pd_config pd_config = {
    DEVICE_MMIO_ROM_INIT(DT_NODELABEL(pendulum0)),
};
DEVICE_DT_DEFINE(DT_NODELABEL(pendulum0), pd_init, NULL, &pd_data, &pd_config,
                 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &pd_api);
