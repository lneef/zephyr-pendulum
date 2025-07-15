#include "driver.h"
#include "zephyr/app_memory/mem_domain.h"
#include "zephyr/arch/arm/mpu/arm_mpu.h"
#include "zephyr/sys/printk.h"
#include <arm_acle.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>

#define PD_ENC_O_VAL 0
#define PD_ENC_0_RST 4
#define PD_ENC_1_VAL 8
#define PD_ENC_1_RST 12
#define PD_BUT_L 16
#define PD_BUT_R 18
#define PD_UART_CMD_PORT 20

struct pd_data {
  DEVICE_MMIO_RAM;
};

struct pd_config {
  DEVICE_MMIO_ROM;
};

static int pd_write_uart(const struct device *dev, const char *cmd,
                         size_t len) {
  unsigned i;
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);

  // cmd must be null terminated
  printk("Writing Cmd: %s", cmd);
  for (i = 0; i < len; ++i) {
    sys_write8(cmd[i], mmio + PD_UART_CMD_PORT);
  }
  return 0;
}

static int pd_reset(const struct device *dev) {
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);
  printk("Resetting Encoders\n");
  sys_write32(0, mmio + PD_ENC_0_RST);
  sys_write32(0, mmio + PD_ENC_1_RST);
  printk("Reset Done\n");
  return 0;
}

static int pd_read(const struct device *dev, struct sensor_state *sstate) {
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);
  sstate->cartPosition = sys_read32(mmio + PD_ENC_O_VAL);
  sstate->pendulumAngle = sys_read32(mmio + PD_ENC_1_VAL);
  sstate->buttons[0] = sys_read32(mmio + PD_BUT_L);
  sstate->buttons[1] = sys_read32(mmio + PD_BUT_R);
  return 0;
}

static int pd_update_threshold(const struct device *dev, int32_t threshold) {
  (void)dev;
  (void)threshold;
  return 0;
}

static int pd_init(const struct device *dev) {
  DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
  mm_reg_t mmio = DEVICE_MMIO_GET(dev);
  printk("Allocating MMIO Region at %lx\n", mmio);
  return 0;
}

static struct pendulum_controller_driver_api pd_api = {.write = pd_write_uart,
                                                       .reset = pd_reset,
                                                       .read = pd_read,
                                                       .update_threshold =
                                                           pd_update_threshold};
static struct pd_data pd_data = {};
static struct pd_config pd_config = {
    DEVICE_MMIO_ROM_INIT(DT_NODELABEL(pendulum0)),
};
DEVICE_DT_DEFINE(DT_NODELABEL(pendulum0), pd_init, NULL, &pd_data, &pd_config,
                 PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &pd_api);

