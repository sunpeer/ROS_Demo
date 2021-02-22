#include "mpu6050.h"

rt_device_t gyro_mpu_sensor;
rt_device_t acce_mpu_sensor;

int rt_hw_mpu6xxx_port()
{
    struct rt_sensor_config cfg;
    cfg.intf.user_data = (void *)MPU6XXX_ADDR_DEFAULT;
    cfg.intf.dev_name = "i2c1";
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_mpu6xxx_init("mpu", &cfg);

    gyro_mpu_sensor = rt_device_find("gyro_mpu");
    acce_mpu_sensor = rt_device_find("acce_mpu");

    if (rt_device_open(gyro_mpu_sensor, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        LOG_E("gyro_mpu open failed!");
        return -1;
    }
    if (rt_device_open(acce_mpu_sensor, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        LOG_E("acce_mpu open failed!");
        return -1;
    }

    return 0;
}
INIT_APP_EXPORT(rt_hw_mpu6xxx_port);


struct rt_sensor_data data_acce;
struct rt_sensor_data data_gyro;

//获取加速度，单位为mg
void measure_acceleration()
{
    rt_device_read(acce_mpu_sensor,0,&data_acce,1);
}

//获取角速度，单位为deg/ms
void measure_gyroscope()
{
    rt_device_read(gyro_mpu_sensor,0,&data_gyro,1);
    data_gyro.data.acce.x/=10;
    data_gyro.data.acce.y/=10;
    data_gyro.data.acce.z/=10;
}