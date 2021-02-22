#ifndef __MAG_H__
#define __MAG_H__

#include <rtthread.h>
#include <rtdbg.h> // LOG_E
#include "hmc5883.h"
#include "sensor_intf_hmc5883.h"
#include "math.h"
#include "math_def.h"

double mag_read();

#endif