/*
 * 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-20     SimpleInit   the first version
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <finsh.h>

#include "max31865.h"

//#define DRV_DEBUG
#define DBG_TAG              "max31865"
  #ifdef DRV_DEBUG
    #define DBG_LVL          DBG_LOG
  #else
    #define DBG_LVL          DBG_INFO
  #endif /* DRV_DEBUG */
#include <rtdbg.h>

#ifndef RT_USING_SPI
#error "Please eable RT_USING_SPI"
#endif

/**
 * @brief Define default parameters
 */
#ifndef MAX31865_REF_RES
    #define MAX31865_REF_RES 400000     /* Reference resistance     unit: m次 */
#endif

#if defined(MAX31865_PT100)
    #define MAX31865_RTD_RES 100000     /* RTD resistance           unit: m次 */
#elif defined(MAX31865_PT1000)
    #define MAX31865_RTD_RES 1000000    /* RTD resistance           unit: m次 */
#else
    #define MAX31865_RTD_RES 100000     /* RTD resistance           unit: m次 */
#endif
/* a precalculated coefficient */
#define PRECALC  (((float)MAX31865_REF_RES) / ((float)MAX31865_RTD_RES) / 327.68f)


/**
 * @brief Read registers' values by SPI protocol
 *
 * @param[in] dev     dev the pointer of device driver structure
 * @param[in]   reg   a register number (see the enum 'MAX31865_REG_*' above)
 * @param[out]  ptr   a pointer to a byte value buffer
 * @param[in]   num   a required registers number to read (must be <= 8)
 */
static rt_err_t max31865_read_registers(struct rt_spi_device *dev,
                                        rt_uint8_t reg,
                                        rt_uint8_t *ptr,
                                        rt_uint8_t num)
{
    struct rt_spi_message msg1, msg2;

    if (num > 8 || (reg + num) > MAX31865_REG_08_END)
    {
        LOG_D("Read register address/number error");
        return -RT_ERROR;
    }

    msg1.send_buf   = &reg;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 1;
    msg1.cs_take    = 1;
    msg1.cs_release = 0;
    msg1.next       = &msg2;

    msg2.send_buf   = RT_NULL;
    msg2.recv_buf   = ptr;
    msg2.length     = num;
    msg2.cs_take    = 0;
    msg2.cs_release = 1;
    msg2.next       = RT_NULL;
    
    if(rt_spi_transfer_message(dev, &msg1) != RT_NULL)
        return -RT_ERROR;
    
    return RT_EOK;
}

/**
 * @brief Write registers' values by SPI protocol
 * 
 * @param[in] dev     dev the pointer of device driver structure
 * @param[in] reg     a register number (see the enum 'MAX31865_REG_*')
 * @param[in] data    a register values
 * @param[in] num     a required registers number to write (must be <= 4)
 */
static rt_err_t max31865_write_registers(struct rt_spi_device *dev,
                                         rt_uint8_t reg,
                                         rt_uint8_t const *data,
                                         rt_uint8_t num)
{
    static rt_uint8_t const WMASKS[] =
    {
        0xD1U,                          /* MAX31865_REG_00_CONF */
        0x00U,                          /* MAX31865_REG_01_RTD_MSB */
        0x00U,                          /* MAX31865_REG_02_RTD_LSB */
        0xFFU,                          /* MAX31865_REG_03_RTDHIGH_MSB */
        0xFEU,                          /* MAX31865_REG_04_RTDHIGH_LSB */
        0xFFU,                          /* MAX31865_REG_05_RTDLOW_MSB */
        0xFEU,                          /* MAX31865_REG_06_RTDLOW_LSB */
        0x00U                           /* MAX31865_REG_07_STATUS */
    };

    rt_int8_t i;  
    rt_uint8_t buf[4];
    struct rt_spi_message msg1, msg2;

    if (num > 4 || (reg + num) > MAX31865_REG_08_END)
    {
        LOG_D("Number of registers written >4");
        return -RT_ERROR;
    }

    *buf = 0x80U | reg;
    
    msg1.send_buf   = buf;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 1;
    msg1.cs_take    = 1;
    msg1.cs_release = 0;
    msg1.next       = &msg2;

    msg2.send_buf   = data;
    msg2.recv_buf   = buf;
    msg2.length     = num;
    msg2.cs_take    = 0;
    msg2.cs_release = 1;
    msg2.next       = RT_NULL;
    
    if(rt_spi_transfer_message(dev, &msg1) != RT_NULL)
        return -RT_ERROR;

    if(max31865_read_registers(dev, reg, buf, num) != RT_EOK)
        return -RT_ERROR;

    for (i = 0; i < num; i++)
    {
        if ((data[i] & WMASKS[reg + i]) != (buf[i] & WMASKS[reg + i]))
        {
            LOG_D("Write self-check error!");
            return -RT_ERROR;
        }
    }

    return RT_EOK;
}

/**
 * @brief This function Fault Status Clear (D1)
 *
 * @param dev the pointer of device driver structure
 *
 * @return none
 */
static void max31865_clear_fault(struct rt_spi_device *dev)
{
    rt_uint8_t byte;

    max31865_read_registers(dev, MAX31865_REG_00_CONF, &byte, 1);
    SET_BIT(byte, 1, 1);
    SET_BIT(byte, 2, 0);
    SET_BIT(byte, 3, 0);
    SET_BIT(byte, 5, 0);
    max31865_write_registers(dev, MAX31865_REG_00_CONF, &byte, 1);
}

/**
 * @brief Initialize and perform power-on self-test (POST) procedures
 */
static rt_err_t sensor_init(max31865_device_t dev)
{
    /* goto into automatic (continuous) conversion mode, ~17msec */
    uint8_t byte = MAX31865_CFIG_D7_VBIAS_ON | MAX31865_CFIG_D6_CONV_AUTO;
    uint8_t result;
    
    if(dev->nwire_type == 1)
        byte |= MAX31865_CFIG_D4_3WIRE;
    if(dev->filter == 1)
        byte |= MAX31865_CFIG_D0_FILTER_50HZ;

    rt_thread_mdelay(500);
    result = max31865_write_registers(dev->spi, MAX31865_REG_00_CONF, &byte, 1);
    rt_thread_mdelay(100);
    
    return result;
}

/**
 * @brief This function manual fault detection
 *
 * @param dev the pointer of device driver structure
 *
 * @return none
 */
rt_err_t max31865_detection_fault(max31865_device_t dev)
{
    rt_uint8_t byte;
    rt_err_t result;
    result = rt_mutex_take(dev->lock, RT_WAITING_FOREVER);
    if(result == RT_EOK)
    {
        if(max31865_read_registers(dev->spi, MAX31865_REG_00_CONF, &byte, 1) != RT_EOK)
        {
            result = -RT_EIO;
            goto __exit;
        }
        SET_BIT(byte, 1, 0);
        SET_BIT(byte, 2, 0);
        SET_BIT(byte, 3, 1);
        SET_BIT(byte, 5, 0);
        SET_BIT(byte, 6, 0);
        SET_BIT(byte, 7, 1);
        /* write 100X100Xb DETECT C1 */
        if(max31865_write_registers(dev->spi, MAX31865_REG_00_CONF, &byte, 1) != RT_EOK)
        {
            result = -RT_EIO;
            goto __exit;
        }
        rt_hw_us_delay(250);
        
        /* write 100X110Xb DETECT C2 */
        SET_BIT(byte, 2, 1);
        max31865_write_registers(dev->spi, MAX31865_REG_00_CONF, &byte, 1);
        rt_hw_us_delay(250);
        
        /* write 100X000Xb END FAULT DETECTION CYCLE */
        SET_BIT(byte, 2, 0);
        SET_BIT(byte, 3, 0);
        max31865_write_registers(dev->spi, MAX31865_REG_00_CONF, &byte, 1);
        
        /* Read fault code */
        max31865_read_registers(dev->spi, MAX31865_REG_07_STATUS, &byte, 1);
        result = byte & 0xFC;
        
        sensor_init(dev);
    }
    else
    {
        LOG_E("Failed to fault detection.Please try again");
    }
    
__exit:
    rt_mutex_release(dev->lock);
    
    return result;
}


/**
 * @brief Read the RTD resistance registers value (15bits)
 */
static rt_int16_t max31865_read_rtd(max31865_device_t dev)
{
    rt_uint8_t buf[2];
    rt_uint16_t val;
    rt_int16_t result;

    RT_ASSERT(dev);
    
    result = rt_mutex_take(dev->lock, RT_WAITING_FOREVER);
    if(result == RT_EOK)
    {
        if(max31865_read_registers(dev->spi, MAX31865_REG_01_RTD_MSB, buf, 2) != RT_EOK)
        {
            result = -RT_EIO;
            goto __exit;
        }

        val = (rt_uint16_t)((buf[0] << 8) + buf[1]);
        
        if (val & MAX31865_RTD_01_FAULT)
        {
            max31865_clear_fault(dev->spi);
            result = -RT_ERROR;
        }
    }
    else
    {
        LOG_E("Failed to reading tempurature.Please try again");
    }
    
__exit:
    rt_mutex_release(dev->lock);

    return (result == RT_EOK ? (val >> 1) : result);
}


#pragma inline
static float calc_iteration(float rnorm, float t) {
  return rnorm / (0.39083f - 0.00005775f * t);
}

/**
 * @brief Scale a plain resistance registers value to physical units
 *
 * @param[in]   rtd       resistance registers value (RTD, 15bits)
 * @param[out]  val       a physical temperature value (as float!)
 * @param[in]   unit      a physical unit (see 'MAX31865_UNIT_*')
 *
 */
static void max31865_calc_rtd(rt_uint16_t rtd,
                          float *val,
                          rt_uint8_t unit)
{
    float rnorm;

    rnorm = ((float) rtd) * PRECALC - 100.0f;

    *val = calc_iteration(rnorm, ((float) rtd) / 32 - 256);

    if (rtd > 13000)
    {
        *val = rtd > 21000 ?
            calc_iteration(rnorm, calc_iteration(rnorm, *val)) :
            calc_iteration(rnorm, *val);
    }

    if (unit == MAX31865_UNIT_00_FAHRENHEIT)
    {
        *val = *val * 1.8f + 32;
    }
}

/**
 * @brief This function reads temperature by max31865 sensor measurement
 *
 * @param dev the pointer of device driver structure
 *
 * @return the relative temperature converted to data. Return -300 on error
 */
float max31865_read_temperature(max31865_device_t dev)
{
    rt_int16_t row;
    float t;
    
    row = max31865_read_rtd(dev);
    
    if(row < 0)
        return -300;
    else
        max31865_calc_rtd(row,&t,MAX31865_UNIT_01_CELSIUS);
    
    return (t + (t >= 0 ? 0.5 : -0.5));        
}

/**
 * @brief This function initializes max31865 registered device driver
 *
 * @param dev the name of max31865 device
 *
 * @return the max31865 device.
 */
max31865_device_t max31865_init(const char *spi_device_name, rt_uint8_t nwire, rt_uint8_t filter)
{
    max31865_device_t dev;

    RT_ASSERT(intf);

    dev = rt_calloc(1, sizeof(struct max31865_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for max31865 device on '%s' ", spi_device_name);
        rt_free(dev);
        return RT_NULL;
    }
    
    dev->spi = SPI_DEVICE(rt_device_find(spi_device_name));
    if (dev->spi == RT_NULL)
    {
        LOG_E("Can't find max31865 device on '%s'", spi_device_name);
        rt_free(dev);
        return RT_NULL;
    }
    
    struct rt_spi_configuration cfg;
    
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.max_hz = 5 * 1000 * 1000;  /* 5M */
    rt_spi_configure(dev->spi, &cfg);

    dev->lock = rt_mutex_create("mutex_max31865", RT_IPC_FLAG_FIFO);
    if (dev->lock == RT_NULL)
    {
        LOG_E("Can't create mutex for max31865 device on '%s'", spi_device_name);
        rt_free(dev);
        return RT_NULL;
    }
    
    dev->nwire_type = nwire;
    dev->filter = filter;
    
    if(sensor_init(dev) != RT_EOK)
        LOG_E("sensor init error");

    return dev;
}

/**
 * @brief This function releases memory and deletes mutex lock
 *
 * @param dev the pointer of device driver structure
 */
void max31865_deinit(max31865_device_t dev)
{
    RT_ASSERT(dev);

    rt_mutex_delete(dev->lock);

    rt_free(dev);
}

#ifdef DRV_DEBUG
void max31865(int argc, char *argv[])
{
    static max31865_device_t dev = RT_NULL;

    if (argc > 1)
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (argc > 2)
            {
                /* initialize the sensor when first probe */
                if (!dev || strcmp(dev->spi->parent.parent.name, argv[2]))
                {
                    /* deinit the old device */
                    if (dev)
                    {
                        max31865_deinit(dev);
                    }
                    dev = max31865_init(argv[2],0,0);
                    rt_kprintf("max31865 probe to %s ok\n",argv[2]);
                }
            }
            else
            {
                rt_kprintf("max31865 probe <dev_name>  - probe sensor by given name\n");
            }
        }
        else if (!strcmp(argv[1], "read"))
        {
            if (dev)
            {
                float temp_data;

                /* read the sensor */
                temp_data = max31865_read_temperature(dev);
                rt_kprintf("max31865 read : %d.%d\n", (rt_int16_t)temp_data, ((rt_uint16_t)(temp_data * 100)) % 100);
            }
            else
            {
                rt_kprintf("Please using 'max31865 probe <dev_name>' first\n");
            }
        }
        else if (!strcmp(argv[1], "detect"))
        {
            if (dev)
            {
                rt_err_t err;

                /* detect the sensor */
                err = max31865_detection_fault(dev);
                rt_kprintf("max31865 error code : %d\n",err);
            }
            else
            {
                rt_kprintf("Please using 'max31865 probe <dev_name>' first\n");
            }
        }
        else
        {
            rt_kprintf("Unknown command. Please enter 'max31865' for help\n");
        }
    }
    else
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("max31865 probe <dev_name>  - probe sensor by given name\n");
        rt_kprintf("max31865 read              - read sensor max31865 data\n");
        rt_kprintf("max31865 detect            - max31865 self-test\n");
    }
}
MSH_CMD_EXPORT(max31865, max31865 sensor function);
#endif


