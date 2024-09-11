/*
 * 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-20     SimpleInit   the first version
 */

#ifndef SENSOR_MAXIM_MAX31865_H__
#define SENSOR_MAXIM_MAX31865_H__

#include <rtthread.h>
#include <rtdef.h>
#include <rtdevice.h>

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif

#endif

#define MAX31865_CFIG_24WIRE         0x00   /* RTD wire: 1 = 3-wire; 0 = 2-wire or 4-wire */
#define MAX31865_CFIG_3WIRE          0x10   /* RTD wire: 1 = 3-wire; 0 = 2-wire or 4-wire */
#define MAX31865_CFIG_FILTER_60HZ    0x00   /* 60Hz filtering, single conversion completion time 52ms */
#define MAX31865_CFIG_FILTER_50HZ    0x01   /* 50Hz filtering, single conversion completion time 62.5ms */

int rt_hw_max31865_init(const char *name, struct rt_sensor_config *cfg);

#endif



