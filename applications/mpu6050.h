#include "sensor_inven_mpu6xxx.h"
#define DBG_TAG           "MPU6050"
#define DBG_LVL           DBG_INFO
#include <rtdbg.h>

extern struct rt_sensor_data data_acce;
extern struct rt_sensor_data data_gyro;

#ifdef __cplusplus
extern "C"{
#endif

void measure_acceleration();

void measure_gryoscope();

#ifdef __cplusplus
}
#endif