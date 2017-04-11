#include "ftm_driver.h"
#include "adc_driver.h"

typedef struct camera_driver {
    int pixcnt;
    int capcnt;
    int scan[128];
    int wbuffer[128];
    int clkval;
    uint8_t newscan;
    ftm_driver *ftm;
    adc_driver *adc;
} camera_driver;

extern camera_driver camera;

void mvg_average(int *data, int size, int n);
