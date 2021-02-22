/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author      Notes
 * 2020-03-10     MyGuo       the first version
 */

#include "mag.h"


rt_device_t mag;
struct hmc5883_3axes valueOffset={57,61,-378};

struct rt_sensor_data mag_data;

static int mag_thread_init()
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)HMC5883_ADDR;

    rt_hw_hmc5883_init("hmc", &cfg);
    mag = rt_device_find("mag_hmc");

    rt_device_open(mag, RT_DEVICE_FLAG_RDONLY);
    rt_device_control(mag, RT_SENSOR_CTRL_SET_RANGE, (void *)HMC5883_RANGE_1_9Ga);
    //设置完量程后，要再读取一下才能生效
    rt_thread_mdelay(10);
    rt_device_read(mag,0,&mag_data,1);
    return 0;
}
INIT_APP_EXPORT(mag_thread_init);

double mag_read()
{
    rt_device_read(mag, 0, &mag_data, 1);
    mag_data.data.mag.x-=valueOffset.x;
    mag_data.data.mag.y-=valueOffset.y;
    mag_data.data.mag.z-=valueOffset.z;

    double mag_y=(double)mag_data.data.mag.y;
    double mag_x=(double)mag_data.data.mag.x;
    //计算航偏角
    //function reference http://c.biancheng.net/ref/atan2.html
    double deg=atan2(mag_y,mag_x)*RAD_TO_DEG;
    //在第三，第四象限
    if(deg<0)
    deg+=360;
    return deg;
}

static int mag_single_read()
{
    rt_device_read(mag, 0, &mag_data, 1);
    mag_data.data.mag.x-=valueOffset.x;
    mag_data.data.mag.y-=valueOffset.y;
    mag_data.data.mag.z-=valueOffset.z;

    rt_kprintf("%d,%d,%d\n", mag_data.data.mag.x, mag_data.data.mag.y, mag_data.data.mag.z);
    double mag_y=(double)mag_data.data.mag.y;
    double mag_x=(double)mag_data.data.mag.x;
    //计算航偏角
    //function reference http://c.biancheng.net/ref/atan2.html
    double deg=atan2(mag_y,mag_x)*RAD_TO_DEG;
    //在第三，第四象限
    if(deg<0)
    deg+=360;
    rt_kprintf("deg=%f\n",deg);
    return 0;
}
MSH_CMD_EXPORT(mag_single_read, read hmc5883 once);

