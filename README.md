# INS5T8025 RTC 驱动（RT-Thread 适配）

## 项目介绍

本驱动是针对 **INS5T8025 实时时钟芯片** 开发的 RT-Thread 适配驱动，基于 I2C 通信协议实现，支持时间读写、闹钟配置、中断触发等核心功能。

驱动已在 **HC32F460** 平台验证通过，兼容 RT-Thread 4.0+ 版本，提供标准化设备框架接口和 Shell 调试命令，可直接集成到物联网、工业控制等嵌入式项目中，稳定性高、易用性强。

## 芯片特性

- 支持 I2C 接口通信（7 位从机地址 `0x32`，读地址 `0x33`）
- 完整时间维度：秒 / 分 / 时 / 日 / 月 / 年 / 星期（24 小时制）
- 灵活闹钟功能：支持「星期闹钟」和「日闹钟」两种模式
- 中断输出：闹钟中断（INT 引脚），支持信号量通知机制
- 低功耗设计，适用于电池供电设备
- 内置电压检测、振荡器停止标志等硬件保护

## 驱动功能

✅ 初始化 I2C 总线与 RTC 芯片

✅ 读取 / 设置系统时间（支持 `struct tm` 标准格式）

✅ 配置闹钟（分 / 时 / 日 / 星期匹配，两种闹钟模式）

✅ 闹钟中断触发（信号量 + 线程处理，避免中断耗时操作）

✅ 清除闹钟标志、启用 / 禁用闹钟中断

✅ RT-Thread Shell 命令调试（时间读写、闹钟配置）

✅ RT-Thread 设备框架适配（支持 `rt_device` 标准接口）

✅ 内置星期计算（基姆拉尔森公式）和 BCD 码转换

## 快速上手

### 1. 环境依赖

- 操作系统：RT-Thread 4.0+
- 硬件平台：支持 I2C 总线的 MCU（已验证 HC32F460）
- 编译工具：RT-Thread Studio / GCC
- 硬件连接：INS5T8025 芯片（VCC/GND/SDA/SCL/INT）

### 2. 硬件接线

|INS5T8025 引脚|单片机引脚|说明|
|---|---|---|
|VCC|3.3V|电源（建议 3.3V，避免 5V）|
|GND|GND|地|
|SDA|I2C_SDA|I2C 数据引脚（需上拉）|
|SCL|I2C_SCL|I2C 时钟引脚（需上拉）|
|INT|PE.4（默认）|中断输出引脚（可修改）|

> 注意：I2C 引脚需外接 4.7KΩ 上拉电阻，INT 引脚默认配置为 `PE.4`，可在代码中修改 `INS5T8025_INT_PIN` 宏定义适配实际硬件。

### 3. 工程集成

#### 3.1 启用驱动配置

在 RT-Thread 工程的 `rtconfig.h` 中添加宏定义，启用驱动：

```c
#define BOARD_USING_INS5T8025
```

#### 3.2 添加文件到工程

将以下文件添加到工程 `drivers` 目录下：

- `ins5t8025.c`（驱动核心文件）
- （可选）`ins5t8025.h`（如需外部调用驱动接口，可自行提取头文件）

#### 3.3 编译工程

确认 RT-Thread 已启用 I2C 驱动框架（`RT_USING_I2C` 和 `RT_USING_I2C_DEVICE` 已定义），编译工程无报错即可。

## 使用说明

### 1. 基础操作（Shell 命令）

驱动内置 Shell 命令，可直接在终端调试，无需编写额外代码：

|命令|功能描述|示例|
|---|---|---|
|`rtc_init_cmd <i2c_bus>`|初始化 RTC（指定 I2C 总线名）|`rtc_init_cmd i2c2_sw`（软件 I2C 2）|
|`rtc_set_cmd <年 月 日 时 分 秒>`|设置系统时间|`rtc_set_cmd 2025 11 20 15 30 00`|
|`rtc_get_cmd`|读取并打印当前时间|`rtc_get_cmd`（输出格式：年 - 月 - 日 时：分: 秒 星期 X）|
|`rtc_set_alarm <时 分 值 类型>`|设置闹钟|`rtc_set_alarm 8 30 1 0`（每周一 8:30）|
|`rtc_alarm_status_cmd`|查看闹钟状态（模式、标志位）|`rtc_alarm_status_cmd`|
|`rtc_wait_alarm_cmd`|阻塞等待闹钟触发|`rtc_wait_alarm_cmd`|
|`rtc_user_demo`|运行驱动测试 Demo（连续读时间）|`rtc_user_demo`|

#### 闹钟设置说明（`rtc_set_alarm`）

- 格式：`rtc_set_alarm <hour> <min> <value> <type>`
- 参数说明：
    - `hour`：小时（0-23）
    - `min`：分钟（0-59）
    - `value`：星期（0-6，0 = 周日）或日期（1-31）
    - `type`：0 = 星期闹钟，1 = 日闹钟
- 示例：
    - 每周三 12:00 触发：`rtc_set_alarm 12 0 3 0`
    - 每月 20 日 9:30 触发：`rtc_set_alarm 9 30 20 1`

### 2. 编程接口调用

若需在代码中调用驱动功能，可直接使用以下核心接口（需包含驱动头文件）：

#### 2.1 初始化与时间操作


```c
// 初始化 RTC（指定 I2C 总线）
rt_err_t ins5t8025_init(const char *i2c_bus_name);

// 设置时间（struct tm 格式）
rt_err_t ins5t8025_set_time(struct tm *time);

// 获取时间（struct tm 格式）
rt_err_t ins5t8025_get_time(struct tm *time);
```

#### 2.2 闹钟操作


```c
// 设置闹钟（hour:小时, min:分钟, day_or_week:日/星期, is_day_alarm:是否日闹钟）
rt_err_t ins5t8025_set_alarm(uint8_t hour, uint8_t min, uint8_t day_or_week, rt_bool_t is_day_alarm);

// 启用/禁用闹钟中断
rt_err_t ins5t8025_enable_alarm_int(rt_bool_t enable);

// 清除闹钟中断标志
rt_err_t ins5t8025_clear_alarm_flag(void);

// 检查闹钟标志是否触发
rt_bool_t ins5t8025_check_alarm_flag(void);
```

#### 2.3 RT-Thread 设备框架接口

驱动已适配 RT-Thread 标准设备框架，可通过 `rt_device` 接口操作：


```c
// 查找 RTC 设备
rt_device_t rtc = rt_device_find("INS5T8025");

// 打开设备
rt_device_open(rtc, RT_DEVICE_OFLAG_RDWR);

// 设置时间
struct tm tm = {0};
tm.tm_year = 2025 - 1900; // 1900年偏移
tm.tm_mon = 11 - 1;       // 0-11 月份
tm.tm_mday = 20;
tm.tm_hour = 15;
tm.tm_min = 30;
tm.tm_sec = 0;
rt_device_control(rtc, RT_DEVICE_CTRL_RTC_SET_TIME, &tm);

// 获取时间
rt_device_control(rtc, RT_DEVICE_CTRL_RTC_GET_TIME, &tm);
```

### 3. 闹钟中断处理

驱动采用「中断 + 信号量 + 线程」的处理机制，避免在中断中执行耗时操作：

1. 闹钟触发时，INT 引脚产生中断，中断函数仅发布信号量；
2. 后台线程 `alarm_task` 阻塞等待信号量，触发后执行业务逻辑；
3. 可在 `rtc_alarm_task_entry` 函数中添加自定义回调（如点亮 LED、发送消息等）。

示例：添加闹钟触发后的自定义逻辑


```c
void rtc_alarm_task_entry(void *param) {
    while (1) {
        if (rt_sem_take(alarm_sem, RT_WAITING_FOREVER) == RT_EOK) {
            rt_kprintf("Alarm triggered! User logic here!\n");
            // 自定义逻辑：点亮 LED
            rt_pin_write(LED_PIN, PIN_HIGH);
            rt_thread_mdelay(1000);
            rt_pin_write(LED_PIN, PIN_LOW);
            
            // 清除闹钟标志
            ins5t8025_clear_alarm_flag();
        }
    }
}
```

## 驱动细节说明

### 1. 寄存器映射（关键）

|寄存器地址|名称|功能|
|---|---|---|
|0x00|INS5T8025_REG_SEC|秒寄存器（BCD 格式，bit7 为 OSF 标志）|
|0x01|INS5T8025_REG_MIN|分寄存器（BCD 格式）|
|0x02|INS5T8025_REG_HOUR|时寄存器（BCD 格式，24 小时制）|
|0x03|INS5T8025_REG_WEEK|星期寄存器（bit0-6 对应周日 - 周六）|
|0x08|INS5T8025_REG_ALARM_MIN|闹钟分寄存器（bit7 为 AE 使能位）|
|0x09|INS5T8025_REG_ALARM_HOUR|闹钟时寄存器|
|0x0A|INS5T8025_REG_ALARM_DAY_WEEK|日 / 星期闹钟寄存器|
|0x0D|INS5T8025_REG_EXT|扩展寄存器（WADA 位选择闹钟模式）|
|0x0E|INS5T8025_REG_FLAG|标志寄存器（AF 为闹钟触发标志）|
|0x0F|INS5T8025_REG_CTL|控制寄存器（AIE 为闹钟中断使能位）|

### 2. 关键宏定义（可配置）

|宏定义|功能|默认值|
|---|---|---|
|`INS5T8025_ADDR`|I2C 从机写地址|0x32|
|`INS5T8025_INT_PIN`|中断引脚|rt_pin_get("PE.4")|
|`ALARM_AE_BIT`|闹钟使能位（bit7）|(1 << 7)|
|`CTL_AIE_BIT`|闹钟中断使能位|(1 << 3)|

## 注意事项

1. 电源稳定性：INS5T8025 对电源敏感，建议使用 3.3V 稳定电源，避免电压波动导致时间不准；
2. I2C 总线：确保 I2C 引脚上拉电阻正常，总线无冲突（避免多个设备地址重复）；
3. 闹钟模式：设置闹钟时需区分「星期闹钟」和「日闹钟」，`is_day_alarm` 参数错误会导致闹钟不触发；
4. 中断引脚：若修改 INT 引脚，需同步调整 `INS5T8025_INT_PIN` 宏定义和引脚模式配置；
5. 时间格式：`struct tm` 结构体中，年份为「1900 年以来的偏移」，月份为「0-11」，需注意转换。

## 已知问题与优化方向

- 目前仅支持单次闹钟，如需循环闹钟，可在 `rtc_alarm_task_entry` 中注销闹钟失能；
- 时间校验仅做简化处理（未判断闰年、每月天数差异），可根据需求扩展；


## 许可证

本项目基于 MIT 许可证开源，可自由用于商业和非商业项目，修改后需保留原版权声明。

## 联系方式

维护：龚金华
单位: 上海威派格智慧水务股份有限公司
邮箱：782730309@qq.com
