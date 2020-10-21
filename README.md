# MAX31865 软件包

## 1、简介

MAX31865 软件包提供了使用温度传感器 [MAX31865](http://www.maximintegrated.com/datasheet/index.mvp/id/7900) 基本功能，并且本软件包新的版本已经对接到了 Sensor 框架，通过 Sensor 框架，开发者可以快速的将此传感器驱动起来。

[MAX31865](http://www.maximintegrated.com/datasheet/index.mvp/id/7900) 是简单易用的热敏电阻至数字输出转换器，优化用于铂电阻温度检测器(RTD)。外部电阻设置RTD灵敏度，高精度Δ-Σ ADC将RTD电阻与基准电阻之比转换为数字输出。[MAX31865](http://www.maximintegrated.com/datasheet/index.mvp/id/7900) 输入具有高达±45V的过压保护，提供RTD及电缆开路、短路检测。

| 功能 | 量程 | 精度 |
| ---- | ---- | ---- |
| 温度 | `-200℃ - 800℃` |`±0.5℃`|

## 2、支持情况

| 包含设备     | 温度 |
| ------------ | ---- |
| **通信接口** |      |
| SPI          | √    |
| **工作模式** |      |
| 轮询         | √    |
| 中断         |      |
| FIFO         |      |
| **电源模式** |      |
| **数据输出速率** |      |
| **测量范围** | 由RTD决定 |
| **自检** | √    |

## 3、使用说明

### 3.1、依赖

- RT-Thread 4.0.0+
- Sensor 组件
- SPI 驱动：max31865 设备使用 SPI 进行数据通讯，需要系统 SPI 驱动支持

### 3.2、获取软件包

使用 max31865 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
          max31865: a package of digital temperature sensor.
            (400000) MAX31865 reference resistor. Unit: Milliohm
                RTD type (PT100)  --->
                Version (latest)  --->
```

**Version**：软件包版本选择，默认选择最新版本。

### 3.3、使用软件包

max31865 软件包初始化函数如下所示：

```c
int rt_hw_max31865_init(const char *name, struct rt_sensor_config *cfg)；
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息配置接口设备）；
- 注册相应的传感器设备，完成 max31865 设备的注册；

#### 使用示例

```c
#include "sensor_maxim_max31865.h"

#define MAX31865_SPI_DEVICE_NAME  "spi10"

int rt_hw_max31865_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name  = MAX31865_SPI_DEVICE_NAME;
    cfg.intf.user_data = (void *)(MAX31865_CFIG_FILTER_50HZ | MAX31865_CFIG_24WIRE);
    
    rt_hw_max31865_init("max31865", &cfg);

    return RT_EOK;
}
INIT_ENV_EXPORT(rt_hw_max31865_port);

static void max_test(int argc, char **argv)
{
    rt_device_t dev;
    struct rt_sensor_data data;
    rt_err_t result = RT_EOK;

    /* 查找传感器设备 */
    dev = rt_device_find("temp_max");
    /* 以只读及轮询模式打开传感器设备 */
    rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);

    if (rt_device_read(dev, 0, &data, 1) == 1)
    {
        rt_kprintf("temp: temp:%3d.%dC, timestamp:%5d\n", data.data.temp / 10, (rt_uint32_t)data.data.temp % 10, data.timestamp);
    }
    
    result = rt_device_control(dev, RT_SENSOR_CTRL_SELF_TEST, RT_NULL);
    
    rt_kprintf("err code: 0x%02X\n",result);
    
    rt_device_close(dev);
    
}
MSH_CMD_EXPORT(max_test, max test);
```

## 4、注意事项

- 当传感器检测到故障后，读取的温度为固定值`-300`。
- 使用`user_data`给`max31865_dev`传入的配置信息，配置选项支持下列参数：

#### RTD类型配置 2/3/4线
```c
#define MAX31865_CFIG_24WIRE         0x00   /* RTD wire: 1 = 3-wire; 0 = 2-wire or 4-wire */
#define MAX31865_CFIG_3WIRE          0x10   /* RTD wire: 1 = 3-wire; 0 = 2-wire or 4-wire */
```
#### MAX31865滤波周期
```c
#define MAX31865_CFIG_FILTER_60HZ    0x00   /* 60Hz filtering, single conversion completion time 52ms */
#define MAX31865_CFIG_FILTER_50HZ    0x01   /* 50Hz filtering, single conversion completion time 62.5ms */
```
## 5、联系人信息

维护人:

- 维护：[SimpleInit](https://github.com/SimpleInit)
- 主页：https://github.com/SimpleInit/max31865