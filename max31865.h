/*
 * 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-20     SimpleInit   the first version
 */

#ifndef __MAX31865_H
#define __MAX31865_H

#include <rthw.h>
#include <rtthread.h>

#define GET_BIT(byte, pos) (((byte) >> (pos)) & 1U)
#define SET_BIT(byte, pos, flag) \
  (byte = (flag) ? ((byte) | (1U << (pos))) : ((byte) & ~(1U << (pos))))


/**
 * @brief Register address definition
 */
enum
{
    MAX31865_REG_00_CONF        = 0x00U,      /* Configuration [R/W] */
    MAX31865_REG_01_RTD_MSB     = 0x01U,      /* RTD MSBs [R] */
    MAX31865_REG_02_RTD_LSB     = 0x02U,      /* RTD LSBs [R] */
    MAX31865_REG_03_RTDHIGH_MSB = 0x03U,      /* High Fault Threshold MSB [R/W] */
    MAX31865_REG_04_RTDHIGH_LSB = 0x04U,      /* High Fault Threshold LSB [R/W] */
    MAX31865_REG_05_RTDLOW_MSB  = 0x05U,      /* Low Fault Threshold MSB [R/W] */
    MAX31865_REG_06_RTDLOW_LSB  = 0x06U,      /* Low Fault Threshold LSB [R/W] */
    MAX31865_REG_07_STATUS      = 0x07U,      /* Fault Status [R] */
    MAX31865_REG_08_END         = 0x08U,      /* end */
};

/**
 * @brief Register configuration definition
 */
enum
{
    MAX31865_CFIG_D0_FILTER_60HZ   = 0x00U,      /* 60Hz filtering, single conversion completion time 52ms */
    MAX31865_CFIG_D0_FILTER_50HZ   = 0x01U,      /* 50Hz filtering, single conversion completion time 62.5ms */
    MAX31865_CFIG_D1_CLEARSTATUS   = 0x02U,      /* Fault Status Clear (Auto reset) */
    MAX31865_CFIG_D3D2_DETECT_NO   = 0x00U,      /* Fault Detection (No action) */
    MAX31865_CFIG_D3D2_DETECT_AUTO = 0x04U,      /* Fault Detection Cycle Control (Automatic) */
    MAX31865_CFIG_D3D2_DETECT_C1   = 0x08U,      /* Fault Detection Cycle Control (manual delay cycle 1) */
    MAX31865_CFIG_D3D2_DETECT_C2   = 0x0CU,      /* Fault Detection Cycle Control (manual delay cycle 2) */
    MAX31865_CFIG_D4_24WIRE        = 0x00U,      /* RTD wire: 1 = 3-wire; 0 = 2-wire or 4-wire */
    MAX31865_CFIG_D4_3WIRE         = 0x10U,      /* RTD wire: 1 = 3-wire; 0 = 2-wire or 4-wire */  
    MAX31865_CFIG_D5_1SHOT         = 0x20U,      /* Start a measurement conversion (Auto reset) */
    MAX31865_CFIG_D6_CONV_MANUAL   = 0x00U,      /* Conversion mode (Manual) */
    MAX31865_CFIG_D6_CONV_AUTO     = 0x40U,      /* Conversion mode (Automatic) */
    MAX31865_CFIG_D7_VBIAS_OFF     = 0x00U,      /* VBIAS OFF */
    MAX31865_CFIG_D7_VBIAS_ON      = 0x80U,      /* VBIAS ON */
};

enum
{
    MAX31865_RTD_01_FAULT = 0x01U,
};
/**
 * @brief Register fault status definition
 */
enum
{
    MAX31865_STATUS_04_VOLTAGE      = 0x04U,
    MAX31865_STATUS_08_RTDIN_FORCE  = 0x08U,  
    MAX31865_STATUS_10_REFIN_FORCE  = 0x10U,
    MAX31865_STATUS_20_REFIN        = 0x20U,
    MAX31865_STATUS_40_RTDLOW       = 0x40U,
    MAX31865_STATUS_80_RTDHIGH      = 0x80U,
};

enum
{
    MAX31865_UNIT_00_FAHRENHEIT = 0x00U,
    MAX31865_UNIT_01_CELSIUS    = 0x01U,
};


struct max31865_device
{
    struct rt_spi_device *spi;
    rt_uint8_t nwire_type:4;  /* RTD wire:   2/3/4 */
    rt_uint8_t filter:4;      /* filter: 1=50Hz; 0=60Hz */
    rt_mutex_t lock;
};
typedef struct max31865_device *max31865_device_t;

/**
 * @brief This function manual fault detection
 */
rt_err_t max31865_detection_fault(max31865_device_t dev);

/**
 * @brief This function reads temperature by max31865 sensor measurement
 */
float max31865_read_temperature(max31865_device_t dev);

/**
 * @brief This function initializes max31865 registered device driver
 */
max31865_device_t max31865_init(const char *spi_device_name, rt_uint8_t nwire, rt_uint8_t filter);



#endif
