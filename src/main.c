#include "driver.h"
#include "control.h"
#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include "zephyr/sys/printk.h"
#include <stdint.h>


int main() { 
    printk("Pendulum Controller on %s started\n", CONFIG_BOARD_TARGET);
    const struct device* pd = DEVICE_DT_GET(DT_NODELABEL(pendulum0));
    printk("Drivers: %p\n", (void*)pd);
    struct sensor_state sstate;
    struct pendulum_state pstate = {.k = 1 };
    struct control_interval con = {.ct = 20000, .su = 2000, .weight = 0.8, .xdist = 0.3, .sx = 0.4};
    float swing_up_str = 0.038f;
    float swing_up_smoother = 0.03f;
    uint32_t control_freq = 40000;
    pd_reset_api(pd);
    int32_t center = calibrate(pd, &sstate);
    wait(pd, &sstate, &pstate, 0.02, 0);
    pd_reset_api(pd);
    while(1){
        control(pd,&pstate, &sstate, &con, control_freq, swing_up_str, swing_up_smoother);
        printk("Button was hit\n");
        go_to(pd, &sstate, center);
    }
    return 0; 
}
