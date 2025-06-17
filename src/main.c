#include "driver.h"
#include "control.h"
#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include <stdint.h>


int main() { 
    const struct device* pd = DEVICE_DT_GET(DT_NODELABEL(pendulum0));
    const struct device* odrive = DEVICE_DT_GET(DT_NODELABEL(uart1));
    struct sensor_state sstate;
    struct pendulum_state pstate = {.k = 1 };
    struct control_interval con = {.ct = 1000, .su = 1000, .weight = 0.997, .xdist = 4.00, .sx = 0.4};
    uint32_t control_freq = 100;
    int32_t center = calibrate(pd, odrive, &sstate);
    wait(pd, &sstate, &pstate, 0.02, 0);
    pd_reset_api(pd);
    while(1){
        control(pd, odrive, &pstate, &sstate, &con, control_freq);
        go_to(pd, odrive, &sstate, center);
    }

    return 0; 
}
