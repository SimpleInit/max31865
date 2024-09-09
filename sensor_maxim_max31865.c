/*
 * 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-20     SimpleInit   the first version
 */

#include "sensor_maxim_max31865.h"
#include "max31865.h"

//#define DRV_DEBUG
#define DBG_TAG              "sensor.maxim.max31865"
  #ifdef DRV_DEBUG
    #define DBG_LVL          DBG_LOG
  #else
    #define DBG_LVL          DBG_INFO
  #endif /* DRV_DEBUG */
#include <rtdbg.h>

#define SENSOR_TEMP_RANGE_MAX (800)
#define SENSOR_TEMP_RANGE_MIN (-200)


#ifndef RT_SENSOR_VENDOR_MAXIM
#define RT_SENSOR_VENDOR_MAXIM RT_SENSOR_VENDOR_UNKNOWN
#endif

static RT_SIZE_TYPE _max31865_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    float temperature;
    struct max31865_device *max31865_dev = sensor->parent.user_data;

    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        temperature = max31865_read_temperature(max31865_dev);
        data->data.temp = (rt_int32_t)(temperature * 10);
        data->timestamp = rt_sensor_get_ts();
    }else 
        return 0;
    return 1;
}

static RT_SIZE_TYPE max31865_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _max31865_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t _max31865_self_test(struct rt_sensor_device *sensor)
{
    return max31865_detection_fault(sensor->parent.user_data);
}

static rt_err_t max31865_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    
    switch (cmd)
    {
        case RT_SENSOR_CTRL_SELF_TEST:
            result = _max31865_self_test(sensor);
            break;
        default:
            return -RT_ERROR;
    }
    
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    max31865_fetch_data,
    max31865_control
};

int rt_hw_max31865_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_temp = RT_NULL; 
    struct max31865_device *max31865_dev;

#ifdef PKG_USING_MAX31865

    max31865_dev = max31865_init(cfg->intf.dev_name, \
                                 ((rt_uint32_t)cfg->intf.user_data >> 4) & 0x0F, \
                                 (rt_uint32_t)cfg->intf.user_data & 0x0F);
    if (max31865_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    /* temperature sensor register */
    sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_temp == RT_NULL)
        return -RT_ERROR;
    
    sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
    sensor_temp->info.vendor     = RT_SENSOR_VENDOR_MAXIM;
    sensor_temp->info.model      = "max31865";
    sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor_temp->info.intf_type  = RT_SENSOR_INTF_SPI;
    sensor_temp->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    sensor_temp->info.range_min  = SENSOR_TEMP_RANGE_MIN;
    sensor_temp->info.period_min = 50;

    rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
    sensor_temp->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDONLY, max31865_dev);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        goto __exit;
    }

#endif

    return RT_EOK;

__exit:
    if (sensor_temp)
        rt_free(sensor_temp);
    return -RT_ERROR;
}




