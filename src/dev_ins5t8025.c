/**
 * @file   drv_ins5t8025.c
 * @brief  RT-Thread RTC driver for INS5T8025 IIC RTC/Alarm chip
 * @author GKoSon
 * @date   2025-12-22
 */
#include "rtconfig.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <time.h>  

#define DEVICE_CTRL_RTC_GET_TIME 1
#define DEVICE_CTRL_RTC_SET_TIME 2

/* ====================================================================== */
/*  INS5T8025 register map                                                */
/* ====================================================================== */
#define INS5T8025_ADDR                0x32    /* I2C 从机地址（7位，写地址，读地址为0x33） */
/* INS5T8025 实时时钟核心寄存器定义 */
#define INS5T8025_REG_SEC             0x00    /* 秒寄存器 */
#define INS5T8025_REG_MIN             0x01    /* 分寄存器 */
#define INS5T8025_REG_HOUR            0x02    /* 时寄存器 */
#define INS5T8025_REG_WEEK            0x03    /* 星期寄存器 */
#define INS5T8025_REG_DAY             0x04    /* 日寄存器 */
#define INS5T8025_REG_MONTH           0x05    /* 月寄存器 */
#define INS5T8025_REG_YEAR            0x06    /* 年寄存器 */
#define INS5T8025_REG_CTL1            0x0E    /* 控制寄存器1 */
#define INS5T8025_REG_CTL2            0x0F    /* 控制寄存器2 */
/* INS5T8025 闹钟相关寄存器定义（手册第11页） */
#define INS5T8025_REG_ALARM_MIN       0x08    /* 分闹钟寄存器 */
#define INS5T8025_REG_ALARM_HOUR      0x09    /* 时闹钟寄存器 */
#define INS5T8025_REG_ALARM_DAY_WEEK  0x0A    /* 日/星期闹钟寄存器 */
/* INS5T8025 控制/状态类寄存器定义 */
#define INS5T8025_REG_EXT             0x0D    /* 扩展寄存器 */
#define INS5T8025_REG_FLAG            0x0E    /* 标志寄存器 */
#define INS5T8025_REG_CTL             0x0F    /* 控制寄存器 */

/* INS5T8025 扩展寄存器位定义（手册第15页） */
#define EXT_WADA_BIT               (1 << 6) /* WADA位：0=星期闹钟，1=日闹钟 */  
#define EXT_USEL_BIT               (1 << 5) /* 更新中断选择位 */  
#define EXT_TE_BIT                 (1 << 4) /* 定时器使能位 */  

/* INS5T8025 标志寄存器位定义（手册第15页） */
#define FLAG_VDET_BIT              (1 << 0) /* 电压检测标志位 */  
#define FLAG_VLF_BIT               (1 << 1) /* 电压低标志位 */  
#define FLAG_AF_BIT                (1 << 3) /* 闹钟标志位 */  
#define FLAG_TF_BIT                (1 << 4) /* 定时器标志位 */  
#define FLAG_UF_BIT                (1 << 5) /* 更新标志位 */  

/* INS5T8025 控制寄存器位定义（手册第16页） */
#define CTL_RESET_BIT              (1 << 0) /* 复位位 */  
#define CTL_AIE_BIT                (1 << 3) /* 闹钟中断使能位 */  
#define CTL_TIE_BIT                (1 << 4) /* 定时器中断使能位 */  
#define CTL_UIE_BIT                (1 << 5) /* 更新中断使能位 */  

/* INS5T8025 闹钟专用位定义 */
#define ALARM_AE_BIT               (1 << 7) /* 闹钟使能位：0=使能，1=禁用 */


#define INS5T8025_INT_PIN   rt_pin_get(INS5T8025_ALARM_INT_PIN)


/* 软件I2C设备句柄 */
static struct rt_i2c_bus_device *ins5t8025_i2c_bus = RT_NULL;



/**********************************星期单独设计**********************************/ 
// 正向查找:比如星期0真实是星期天对应weekMap[0]对应值是0X01
static const uint8_t weekMap[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};
// 反向查找:比如值是0x40 执行函数找到[6]对应星期六
static int reverse_lookup(uint8_t value) {
    for(int i = 0; i < 7; i++) {
        if(weekMap[i] == value) {
            return i; 
        }
    }
    return -1;
}

//基姆拉尔森公式：根据年月日计算星期几 0=Sunday ... 6=Saturday
int calculate_weekday(int year, int month, int day) {
    if(month < 3)    {
        month += 12;
        year--;
    }
    int week = (day + 1 + 2 * month + 3 * (month + 1) / 5 + year + year / 4 - year / 100 + year / 400) % 7;
    return week;  
}

/**********************************辅助函数**********************************/
/* BCD码转十进制 */
static uint8_t bcd2dec(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

/* 十进制转BCD码 */
static uint8_t dec2bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

/**********************************操作IIC**********************************/

static rt_err_t ins5t8025_write_reg(uint8_t reg, uint8_t data) {
    struct rt_i2c_msg msgs[1];
    uint8_t buf[2] = {reg, data};

    msgs[0].addr  = INS5T8025_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = buf;
    msgs[0].len   = 2;

    if(rt_i2c_transfer(ins5t8025_i2c_bus, msgs, 1) != 1) {
        rt_kprintf("INS5T8025 write reg %02X failed!\n", reg);
        return RT_ERROR;
    }
    return RT_EOK;
}


static rt_err_t ins5t8025_read_reg(uint8_t reg, uint8_t *data) {
    struct rt_i2c_msg msgs[2];
    uint8_t reg_addr = reg;

    /* 第一步：发送要读取的寄存器的地址 */
    msgs[0].addr  = INS5T8025_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg_addr;
    msgs[0].len   = 1;

    /* 第二步：读取数据 */
    msgs[1].addr  = INS5T8025_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = 1;

    if(rt_i2c_transfer(ins5t8025_i2c_bus, msgs, 2) != 2) {
        rt_kprintf("INS5T8025 read reg %02X failed!\n", reg);
        return RT_ERROR;
    }
    return RT_EOK;
}

/**********************************API函数**********************************/
static rt_err_t ins5t8025_init_int(void);

/* 初始化INS5T8025 */
rt_err_t ins5t8025_init(const char *i2c_bus_name) {
    uint8_t ctl1, ctl2;

    ins5t8025_i2c_bus = rt_i2c_bus_device_find(i2c_bus_name);
    if(ins5t8025_i2c_bus == RT_NULL) {
        rt_kprintf("Can't find I2C bus %s!\n", i2c_bus_name);
        return RT_ERROR;
    }

    if(ins5t8025_read_reg(INS5T8025_REG_CTL1, &ctl1) != RT_EOK)
        return RT_ERROR;
    if(ins5t8025_read_reg(INS5T8025_REG_CTL2, &ctl2) != RT_EOK)
        return RT_ERROR;

    /* 初始化控制寄存器：关闭中断，启用振荡器，默认24小时制 */
    ctl1 = 0x00;  /* 禁用中断，普通模式 */
    ctl2 = 0x00;  /* 禁用定时器，默认频率输出 */
    if(ins5t8025_write_reg(INS5T8025_REG_CTL1, ctl1) != RT_EOK)
        return RT_ERROR;
    if(ins5t8025_write_reg(INS5T8025_REG_CTL2, ctl2) != RT_EOK)
        return RT_ERROR;

    
    
     /* 初始化闹钟中断功能:闹钟原理 写寄存器配置闹钟时间-时间到PIN脚下降沿中断-应用查到中断写寄存器清初*/
    if (ins5t8025_init_int() != RT_EOK) {
        rt_kprintf("Init interrupt failed!\n");
        return RT_ERROR;
    }   
    
   
    rt_kprintf("ins5t8025_init success!\n");
    return RT_EOK;
}




/* 设置RTC时间 */
rt_err_t ins5t8025_set_time(struct tm *time) {
    if(time == RT_NULL)
        return RT_ERROR;


    // 校验struct tm的参数（根据struct tm的特性）
    if(time->tm_sec > 59 || time->tm_min > 59 || time->tm_hour > 23 ||
            time->tm_mday < 1 || time->tm_mday > 31 ||
            time->tm_mon < 0 || time->tm_mon > 11 ||  // struct tm的月份是0-11
            time->tm_year < 100 || time->tm_year > 200) { // 100对应2000年（1900+100）
        rt_kprintf("Invalid time parameters!\n");
        return RT_ERROR;
    }


    rt_kprintf("ins5t8025_set_time week:%d  %d\n", time->tm_wday, dec2bcd(time->tm_wday));
    /* 写入时间寄存器（秒、分、时、星期、日、月、年） */
    ins5t8025_write_reg(INS5T8025_REG_SEC,  dec2bcd(time->tm_sec));
    ins5t8025_write_reg(INS5T8025_REG_MIN,  dec2bcd(time->tm_min));
    ins5t8025_write_reg(INS5T8025_REG_HOUR, dec2bcd(time->tm_hour));  /* 24小时制 */
    ins5t8025_write_reg(INS5T8025_REG_WEEK, weekMap[time->tm_wday]);
    ins5t8025_write_reg(INS5T8025_REG_DAY,  dec2bcd(time->tm_mday));
    ins5t8025_write_reg(INS5T8025_REG_MONTH, dec2bcd(time->tm_mon));
    ins5t8025_write_reg(INS5T8025_REG_YEAR, dec2bcd(time->tm_year));  /* 年份取后两位（00-99） */

    return RT_EOK;
}




/* 获取RTC时间 */
rt_err_t ins5t8025_get_time(struct tm *time) {
    uint8_t sec, min, hour, week, day, month, year;

    if(time == RT_NULL)
        return RT_ERROR;

    /* 读取时间寄存器 */
    ins5t8025_read_reg(INS5T8025_REG_SEC,   &sec);
    ins5t8025_read_reg(INS5T8025_REG_MIN,   &min);
    ins5t8025_read_reg(INS5T8025_REG_HOUR,  &hour);
    ins5t8025_read_reg(INS5T8025_REG_WEEK,  &week);
    ins5t8025_read_reg(INS5T8025_REG_DAY,   &day);
    ins5t8025_read_reg(INS5T8025_REG_MONTH, &month);
    ins5t8025_read_reg(INS5T8025_REG_YEAR,  &year);

    /* BCD转十进制并赋值 */
    time->tm_sec  = bcd2dec(sec & 0x7F);  /* 屏蔽秒寄存器的OSF位（振荡器停止标志） */
    time->tm_min  = bcd2dec(min & 0x7F);
    time->tm_hour = bcd2dec(hour & 0x3F); /* 24小时制，屏蔽高位 */
    time->tm_wday = reverse_lookup(week) ;
    time->tm_mday = bcd2dec(day & 0x3F);
    time->tm_mon  = bcd2dec(month & 0x1F);
    time->tm_year = bcd2dec(year);  /* 年份后两位（00-99） */

    rt_kprintf("ins5t8025_get_time week:%d %d\n", week, time->tm_wday);
    return RT_EOK;
}



/* 设置闹钟时间 */
rt_err_t ins5t8025_set_alarm(uint8_t hour, uint8_t min, uint8_t day_or_week, rt_bool_t is_day_alarm) {
    uint8_t ext_reg;

    /* 读取扩展寄存器 */
    if (ins5t8025_read_reg(INS5T8025_REG_EXT, &ext_reg) != RT_EOK)
        return RT_ERROR;

    /* 设置WADA位选择日闹钟或星期闹钟 */
    if (is_day_alarm) {
        ext_reg |= EXT_WADA_BIT;  // 日闹钟模式
    } else {
        ext_reg &= ~EXT_WADA_BIT; // 星期闹钟模式
    }
    
    if (ins5t8025_write_reg(INS5T8025_REG_EXT, ext_reg) != RT_EOK)
        return RT_ERROR;

    /* 写入分闹钟寄存器（AE=0使能闹钟） */
    if (ins5t8025_write_reg(INS5T8025_REG_ALARM_MIN, dec2bcd(min) & 0x7F) != RT_EOK)
        return RT_ERROR;

    /* 写入时闹钟寄存器（AE=0使能闹钟） */
    if (ins5t8025_write_reg(INS5T8025_REG_ALARM_HOUR, dec2bcd(hour) & 0x7F) != RT_EOK)
        return RT_ERROR;

    /* 写入日/星期闹钟寄存器 */
    if (is_day_alarm) {
        /* 日闹钟：写入日值（1-31），BCD格式，AE=0 */
        if (ins5t8025_write_reg(INS5T8025_REG_ALARM_DAY_WEEK, dec2bcd(day_or_week) & 0x7F) != RT_EOK)
            return RT_ERROR;
    } else {
        /* 星期闹钟：写入星期值（0-6），使用weekMap映射，AE=0 */
        if (day_or_week < 7) {
            if (ins5t8025_write_reg(INS5T8025_REG_ALARM_DAY_WEEK, weekMap[day_or_week] & 0x7F) != RT_EOK)
                return RT_ERROR;
        } else {
            rt_kprintf("Invalid weekday value! Must be 0-6\n");
            return RT_ERROR;
        }
    }

    rt_kprintf("Alarm set: %02d:%02d %s=%d\n", hour, min, is_day_alarm ? "Day" : "Week", day_or_week);
    return RT_EOK;
}

/* 使能闹钟中断 */
rt_err_t ins5t8025_enable_alarm_int(rt_bool_t enable) {
    uint8_t ctl_reg;

    if (ins5t8025_read_reg(INS5T8025_REG_CTL, &ctl_reg) != RT_EOK)
        return RT_ERROR;

    if (enable) {
        ctl_reg |= CTL_AIE_BIT;  // 使能闹钟中断
    } else {
        ctl_reg &= ~CTL_AIE_BIT; // 禁用闹钟中断
    }

    return ins5t8025_write_reg(INS5T8025_REG_CTL, ctl_reg);
}

/* 清除闹钟标志 */
rt_err_t ins5t8025_clear_alarm_flag(void) {
    uint8_t flag_reg;

    if (ins5t8025_read_reg(INS5T8025_REG_FLAG, &flag_reg) != RT_EOK)
        return RT_ERROR;

    /* 写0清除AF位（手册第15页） */
    flag_reg &= ~FLAG_AF_BIT;

    return ins5t8025_write_reg(INS5T8025_REG_FLAG, flag_reg);
}

/* 检查闹钟标志状态 */
rt_bool_t ins5t8025_check_alarm_flag(void) {
    uint8_t flag_reg;

    if (ins5t8025_read_reg(INS5T8025_REG_FLAG, &flag_reg) != RT_EOK)
        return RT_FALSE;

    return (flag_reg & FLAG_AF_BIT) ? RT_TRUE : RT_FALSE;
}

/* 获取扩展寄存器值（用于调试） */
rt_err_t ins5t8025_get_ext_reg(uint8_t *value) {
    return ins5t8025_read_reg(INS5T8025_REG_EXT, value);
}

/**********************************闹钟功能**********************************/
/* 新增全局变量 */
static rt_sem_t alarm_sem = RT_NULL;  /* 闹钟信号量 */

/* 中断处理函数 */
static void ins5t8025_int_handler(void *args) {
    rt_sem_release(alarm_sem);
}
void rtc_alarm_task_entry(void *param) {
    while (1) {
        /* 永久等待闹钟事件 */
        if (rt_sem_take(alarm_sem, RT_WAITING_FOREVER) == RT_EOK) {
            /*********** 以下代码都在线程上下文，安全打印、回调 ***********/
            rt_kprintf("Alarm triggered! thread handler!\n");

             /* 读取标志寄存器 */
             uint8_t flag_reg;
            if (ins5t8025_read_reg(INS5T8025_REG_FLAG, &flag_reg) == RT_EOK) {
            /* 检查闹钟标志 */
                if (flag_reg & FLAG_AF_BIT) {
/***************************************************************************/                
                    rt_kprintf("Alarm triggered! Flag register: 0x%02X\n", flag_reg);                
                    /* 清除闹钟标志 */
                    ins5t8025_clear_alarm_flag();                
                    /* TODO: 这里可以加用户自定义回调函数 */
/***************************************************************************/                     
                }
            }
        }
    }
}


/* 初始化中断功能 */
static rt_err_t ins5t8025_init_int(void) {
    /* 创建闹钟信号量 */
    alarm_sem = rt_sem_create("alarm_sem", 0, RT_IPC_FLAG_FIFO);
    if (alarm_sem == RT_NULL) {
        rt_kprintf("Create alarm_sem failed!\n");
        return RT_ERROR;
    }
    
    /* 配置中断引脚（假设INT引脚连接到PA0，请根据实际硬件调整） */
    rt_pin_mode(INS5T8025_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(INS5T8025_INT_PIN, PIN_IRQ_MODE_FALLING, ins5t8025_int_handler, RT_NULL);
    rt_pin_irq_enable(INS5T8025_INT_PIN, PIN_IRQ_ENABLE);
    
    
    //创建对应的任务 当信号alarm_sem发布的时候 去执行读寄存器等动作
    rt_thread_t tid = rt_thread_create("alarm_task",
                           rtc_alarm_task_entry,
                           RT_NULL,
                           1024,
                           20,
                           20);
    if (tid)
        rt_thread_startup(tid);
 
    /* 使能闹钟中断 */
    if (ins5t8025_enable_alarm_int(RT_TRUE) != RT_EOK) {
        rt_kprintf("Enable alarm interrupt failed!\n");
        return RT_ERROR;
    }
    
    rt_kprintf("Alarm interrupt initialized\n");
    return RT_EOK;
}


/**********************************Shell命令**********************************/

/* Shell命令：设置闹钟 

rtc_set_alarm_cmd 9 59   12 1
rtc_set_alarm_cmd 10 0   5  0 
*/  
static void rtc_set_alarm_cmd(int argc, char **argv) {  
    if (argc != 5) {  
        rt_kprintf("Usage: rtc_set_alarm_cmd <hour> <min> <value> <type>\n");  
        rt_kprintf("  type: 0=Weekday (0-6), 1=Day (1-31)\n");  
        rt_kprintf("Example: rtc_set_alarm_cmd 8 30   1 0   # 每周一8:30   [最后TYPE是0表示周 那么前面1  表示周一]\n");  
        rt_kprintf("         rtc_set_alarm_cmd 12 0   15 1  # 每月15日12:00[最后TYPE是1表示天 那么前面15 表示周月15日的这一天]\n");  
        return;  
    }  
    uint8_t hour = atoi(argv[1]);  
    uint8_t min = atoi(argv[2]);  
    uint8_t value = atoi(argv[3]);  
    uint8_t onedDayInWeek  =0;
    uint8_t onedDayInMonth =0;
    if(0==atoi(argv[4])){
        onedDayInWeek=1 ;
        if (value > 6) {  
            rt_kprintf("Invalid weekday value! Must be 0-6\n");  
            return;  
        }  
    }else{
        onedDayInMonth=1;
        if (value < 1 || value > 31) {  
            rt_kprintf("Invalid day value! Must be 1-31\n");  
            return;  
        }
    }

    if (hour > 23 || min > 59) {  
        rt_kprintf("Invalid time parameters!\n");  
        return;  
    }  


    if (ins5t8025_set_alarm(hour, min, value, onedDayInMonth) == RT_EOK) {  
        rt_kprintf("Alarm set successfully.\n");  
    } else {  
        rt_kprintf("Failed to set alarm.\n");  
    }  
}  
MSH_CMD_EXPORT(rtc_set_alarm_cmd, Set alarm: rtc_set_alarm hour-min-value type);  

/* Shell命令：检查闹钟状态 */
static void rtc_alarm_status_cmd(int argc, char **argv) {
    uint8_t ext_reg, flag_reg;
    
    if (ins5t8025_get_ext_reg(&ext_reg) != RT_EOK) {
        rt_kprintf("Read ext reg failed!\n");
        return;
    }
    
    if (ins5t8025_read_reg(INS5T8025_REG_FLAG, &flag_reg) != RT_EOK) {
        rt_kprintf("Read flag reg failed!\n");
        return;
    }
    
    rt_kprintf("Alarm status:\n");
    rt_kprintf("  WADA bit: %d (%s)\n", (ext_reg & EXT_WADA_BIT) ? 1 : 0, (ext_reg & EXT_WADA_BIT) ? "Day alarm" : "Week alarm");
    rt_kprintf("  AF flag: %d\n", (flag_reg & FLAG_AF_BIT) ? 1 : 0);
    rt_kprintf("  AIE enabled: Need to read CTL register\n");
}
MSH_CMD_EXPORT(rtc_alarm_status_cmd, Show alarm status);

/* Shell命令：等待闹钟触发 */
static void rtc_wait_alarm_cmd(int argc, char **argv) {
    rt_kprintf("Waiting for alarm trigger...\n");
    
    if (rt_sem_take(alarm_sem, RT_WAITING_FOREVER) == RT_EOK) {
        rt_kprintf("Alarm received!\n");
    } else {
        rt_kprintf("Wait alarm failed!\n");
    }
}
MSH_CMD_EXPORT(rtc_wait_alarm_cmd, Wait for alarm trigger);



/**********************************独立API***********************************/

/* 全局变量：标记RTC是否已初始化 */
static rt_bool_t ins5t8025_inited = RT_FALSE;

/*
 * shell命令：初始化RTC（参数为I2C总线名，如"i2c1"）
 */
static void rtc_init_cmd(int argc, char **argv) {
    if(argc != 2) {
        rt_kprintf("useage : rtc_init_cmd <i2c_bus_name>\n");
        rt_kprintf("example: rtc_init_cmd i2c2_sw\n");
        return;
    }

    rt_err_t ret = ins5t8025_init(argv[1]);
    if(ret == RT_EOK) {
        ins5t8025_inited = RT_TRUE;
    } else {
        rt_kprintf("INS5T8025 Fail: %d\n", ret);
    }
}
MSH_CMD_EXPORT(rtc_init_cmd, rtc_init_cmd);




int param_check(uint16_t year,uint16_t month, uint16_t day,uint16_t hour, uint16_t min, uint16_t sec,uint16_t weekday){ 
    if(year < 2000 || year > 2100) {         // 限制合理年份范围  
        rt_kprintf("年份不合法!请输入2000-2100之间的年份0X[%X]\n",year);  
        return 1;  
    }  
    if(month < 1 || month > 12) {            // 月份1-12  
        rt_kprintf("月份不合法!请输入1-12之间的月份\n");  
        return 1; 
    }  
    if(day < 1 || day > 31) {                // 日期1-31（简化校验，可根据月份细化）  
        rt_kprintf("日期不合法!请输入1-31之间的日期\n");  
        return 1; 
    }  
    if((int16_t)hour < 0 || hour > 23) {              // 小时0-23  
        rt_kprintf("小时不合法!请输入0-23之间的小时\n");  
        return 1; 
    }  
    if((int16_t)min < 0 || min > 59) {                // 分钟0-59  
        rt_kprintf("分钟不合法!请输入0-59之间的分钟\n");  
        return 1; 
    }  
    if((int16_t)sec < 0 || sec > 59) {                // 秒钟0-59  
        rt_kprintf("秒钟不合法!请输入0-59之间的秒钟\n");  
        return 1; 
    } 
    return 0;
} 

/* 
 * shell命令：设置时间（参数为 年 月 日 时 分 秒） 
 * 示例：rtc_set_cmd 2025 11 19 14 35 00 
 */  
static void rtc_set_cmd(int argc, char **argv) {  
    struct tm time;  
    rt_err_t ret;  
    uint16_t year, month, day, hour, min, sec;
    uint16_t weekday;  
    if(!ins5t8025_inited) {  
        rt_kprintf("need rtc_init first!\n");  
        return;  
    }  

    if(argc != 7) {
        rt_kprintf("useage : rtc_set <年> <月> <日> <时> <分> <秒>\n");
        rt_kprintf("example: rtc_set 2024 10 25 09 30 00\n");
        return;
    }

    // 解析命令行参数
    year = atoi(argv[1]);
    month = atoi(argv[2]);
    day = atoi(argv[3]);
    hour = atoi(argv[4]);
    min = atoi(argv[5]);
    sec = atoi(argv[6]);  
    if(param_check(year,month,day,hour,min,sec,weekday) != 0){
        return;
    }
    // 关键：计算星期（0=周日，1=周一，...，6=周六）  
    weekday = calculate_weekday(year, month, day);  
    // 转换为struct tm结构体的格式  
    time.tm_year = year - 1900;  // struct tm的年份是1900年以来的偏移（2024-1900=124）  
    time.tm_mon  = month - 1;    // struct tm的月份是0-11（10月对应9）  
    time.tm_mday = day;  
    time.tm_hour = hour;  
    time.tm_min  = min;  
    time.tm_sec  = sec;  
    time.tm_wday = weekday;      // 星期计算  
    // 调用设置时间函数  
    ret = ins5t8025_set_time(&time);  
    if(ret == RT_EOK) {  
        rt_kprintf("A时间设置成功: %s", asctime(&time));  // asctime自动格式化时间字符串  
        rt_kprintf("B时间设置成功: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, min, sec);  
    } else {  
        rt_kprintf("时间设置失败!错误码: %d\n", ret);  
    }  
}  
MSH_CMD_EXPORT(rtc_set_cmd, rtc_set_cmd);






/*
 * shell命令：读取并打印当前时间
 * 示例：rtc_get_cmd
 */
static void rtc_get_cmd(int argc, char **argv) {
    struct tm time;
    rt_err_t ret;

    if(!ins5t8025_inited) {
        rt_kprintf("请先执行 rtc_init 初始化设备!\n"); 
        return;
    }

    ret = ins5t8025_get_time(&time);
    if(ret == RT_EOK) {
        // 格式化输出时间（struct tm的年份需要+1900，月份+1）
        int weekday = (time.tm_wday == 0) ? 7 : time.tm_wday;
        rt_kprintf("当前时间: %04d-%02d-%02d %02d:%02d:%02d 星期%d\n",
                   time.tm_year + 1900,   // 转换为实际年份
                   time.tm_mon + 1,       // 转换为1-12月
                   time.tm_mday,
                   time.tm_hour,
                   time.tm_min,
                   time.tm_sec,
                   weekday);  // 正确显示星期
    } else {
        rt_kprintf("时间读取失败!错误码: %d\n", ret);  
    }
}
MSH_CMD_EXPORT(rtc_get_cmd, rtc_get_cmd);






























/********************************************************************/
/*                       RT-Thread 设备框架二次封装                  */
/********************************************************************/
typedef struct {
    struct rt_device          parent;      /* 继承 rt_device */
    rt_mutex_t                lock;        /* 保护 I2C 操作 */
} ins5t8025_dev_t;

static ins5t8025_dev_t  ins5t8025_dev;   /* 唯一实例 */



static rt_err_t _ins5t8025_rtc_get_time(struct tm *tm) {
    return ins5t8025_get_time(tm);
}

static rt_err_t _ins5t8025_rtc_set_time(struct tm *tm) {
    return ins5t8025_set_time(tm);
}


static rt_err_t ins5t8025_dev_open(rt_device_t dev, rt_uint16_t oflag) {
    return ins5t8025_init(INS5T8025_I2C_BUS);
}

static rt_err_t ins5t8025_dev_close(rt_device_t dev) { return RT_EOK; }

static rt_ssize_t ins5t8025_dev_read(rt_device_t dev,
                                     rt_off_t pos,
                                     void *buffer,
                                     rt_size_t size) {
    struct tm *tm = (struct tm *)buffer;
    if (size < sizeof(struct tm)) return 0;

    if (_ins5t8025_rtc_get_time(tm) != RT_EOK) return 0;
    return sizeof(struct tm);
}

static rt_ssize_t ins5t8025_dev_write(rt_device_t dev,
                                      rt_off_t pos,
                                      const void *buffer,
                                      rt_size_t size) {
    const struct tm *tm = (const struct tm *)buffer;
    if (size < sizeof(struct tm)) return 0;

    return (_ins5t8025_rtc_set_time((struct tm *)tm) == RT_EOK) ? sizeof(struct tm) : 0;
}

static rt_err_t ins5t8025_dev_control(rt_device_t dev,
                                      int cmd,
                                      void *args) {
    switch (cmd) {
    case DEVICE_CTRL_RTC_GET_TIME:
        return _ins5t8025_rtc_get_time((struct tm *)args);

    case DEVICE_CTRL_RTC_SET_TIME:
        return _ins5t8025_rtc_set_time((struct tm *)args);

    default:
        return RT_EINVAL;
    }
}

#ifdef RT_USING_DEVICE_OPS
// 先定义全局的设备操作结构体（适配旧版本）
static const struct rt_device_ops ops =
{
    RT_NULL,               // init 函数
    ins5t8025_dev_open,    // open 函数
    ins5t8025_dev_close,   // close 函数
    ins5t8025_dev_read,    // read 函数
    ins5t8025_dev_write,   // write 函数
    ins5t8025_dev_control  // control 函数
};

#endif


static int ins5t8025_dev_register(void) {
    ins5t8025_dev_t *d = &ins5t8025_dev;

    rt_memset(d, 0, sizeof(*d));
    d->lock = rt_mutex_create("INS5T8025", RT_IPC_FLAG_FIFO);
    RT_ASSERT(d->lock != RT_NULL);

    d->parent.type    = RT_Device_Class_RTC;
    
#ifdef RT_DEVICE_USING_OPS  // 旧版本（有rt_device_ops的情况）
    d->parent.ops = &ops; 
#else                       // 新版本（无ops，直接赋值函数）
   
    d->parent.init    = RT_NULL;
    d->parent.open    = ins5t8025_dev_open;
    d->parent.close   = ins5t8025_dev_close;
    d->parent.read    = ins5t8025_dev_read;
    d->parent.write   = ins5t8025_dev_write;
    d->parent.control = ins5t8025_dev_control;
#endif
    
    return rt_device_register(&d->parent, "INS5T8025", RT_DEVICE_FLAG_RDWR);
}
INIT_ENV_EXPORT(ins5t8025_dev_register);



static void rtc_user_demo_entry(void *param) {
    rt_device_t rtc;
    struct tm   tm;
    rt_err_t    ret;
    int         i;

    /* 1. 查找设备 因为提前有ins5t8025_dev_register*/
    rtc = rt_device_find("INS5T8025");
    if (rtc == RT_NULL) {
        rt_kprintf("[RTC_TEST] device 'rtc' not found!\n");
        return;
    }

    /* 2. 打开设备（非必须，control 接口可直接用） */
    ret = rt_device_open(rtc, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("[RTC_TEST] open rtc fail, %d\n", ret);
        return;
    }

    /* 3. 连续读 5 次，看初始值 */
    rt_kprintf("[RTC_TEST] -------- read 5 times --------\n");
    for (i = 0; i < 5; i++) {
        ret = rt_device_control(rtc, DEVICE_CTRL_RTC_GET_TIME, &tm);
        if (ret == RT_EOK)
            rt_kprintf("[RTC_TEST] read %d: %s", i, asctime(&tm));
        else
            rt_kprintf("[RTC_TEST] read %d err, %d\n", i, ret);
        rt_thread_mdelay(1000);
    }
#if 0
    /* 4. 构造一个目标时间 2025-06-25 18:30:45 */
    tm.tm_year = 2025 - 1900;
    tm.tm_mon  = 6  - 1;
    tm.tm_mday = 25;
    tm.tm_hour = 18;
    tm.tm_min  = 30;
    tm.tm_sec  = 45;
    tm.tm_wday = 3;          /* 2025-06-25 确实是星期三 */
    rt_kprintf("[RTC_TEST] -------- write target: %s", asctime(&tm));
    ret = rt_device_control(rtc, DEVICE_CTRL_RTC_SET_TIME, &tm);
    rt_kprintf("[RTC_TEST] write ret = %d\n", ret);

    /* 5. 再读 3 次，确认写入成功 */
    rt_kprintf("[RTC_TEST] -------- read 3 times after write --------\n");
    for (i = 0; i < 3; i++) {
        ret = rt_device_control(rtc, DEVICE_CTRL_RTC_GET_TIME, &tm);
        if (ret == RT_EOK)
            rt_kprintf("[RTC_TEST] read %d: %s", i, asctime(&tm));
        else
            rt_kprintf("[RTC_TEST] read %d err, %d\n", i, ret);
        rt_thread_mdelay(1000);
    }

    /* 6. 关设备（非必须） */
    rt_device_close(rtc);
#endif    
    rt_kprintf("[RTC_TEST] -------- test finish --------\n");
}


static int rtc_user_demo(void) {
    rt_thread_t tid;
    tid = rt_thread_create("rtc_demo",
                           rtc_user_demo_entry,
                           RT_NULL,
                           2048,
                           RT_THREAD_PRIORITY_MAX / 2,
                           20);
    if (tid)
        rt_thread_startup(tid);
    return RT_EOK;
}

MSH_CMD_EXPORT(rtc_user_demo, rtc_user_demo);
