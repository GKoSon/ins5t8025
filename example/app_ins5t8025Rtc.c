#include "rtconfig.h"
#include <rtthread.h>
#include <time.h>  


extern int param_check(uint16_t year,uint16_t month, uint16_t day,uint16_t hour, uint16_t min, uint16_t sec,uint16_t weekday);
extern int calculate_weekday(int year, int month, int day);
extern rt_err_t ins5t8025_set_time(struct tm *time);
extern rt_err_t ins5t8025_get_time(struct tm *time);
extern rt_err_t ins5t8025_init(const char *i2c_bus_name);

char api_rtc_set_time(uint16_t year,uint16_t month, uint16_t day,uint16_t hour, uint16_t min, uint16_t sec){
    int weekday;  
    struct tm time;  
    rt_err_t ret;  
    if(param_check(year,month,day,hour,min,sec,weekday) != 0){
        return 1;
    }

    weekday = calculate_weekday(year, month, day);

    time.tm_year = year - 1900;  
    time.tm_mon  = month - 1;    
    time.tm_mday = day;
    time.tm_hour = hour;
    time.tm_min  = min;
    time.tm_sec  = sec;
    time.tm_wday = weekday;      

    ret = ins5t8025_set_time(&time);
    if(ret == RT_EOK) {
        rt_kprintf("A时间设置成功: %s", asctime(&time));  // asctime自动格式化时间字符串
        rt_kprintf("B时间设置成功: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, min, sec);
        return 0;
    } else {  
        rt_kprintf("时间设置失败!错误码: %d\n", ret);  
        return 1;
    }  
}

int api_rtc_get_time(uint16_t *year,uint16_t *month, uint16_t *day,uint16_t *hour, uint16_t *min, uint16_t *sec,uint16_t *weekday){
    struct tm time;  
    rt_err_t ret;  
    ret = ins5t8025_get_time(&time);  
    if(ret == RT_EOK) {  
        *year = time.tm_year + 1900;
        *month = time.tm_mon + 1;
        *day = time.tm_mday; 
        *hour = time.tm_hour; 
        *min = time.tm_min; 
        *sec = time.tm_sec; 
        *weekday = (time.tm_wday == 0) ? 7 : time.tm_wday;  
        rt_kprintf("时间读取 %d-%d-%d-%d-%d-%d-%d\n",*year,*month,*day,*hour,*min,*sec,*weekday);         
        return 0;
    } else {  
        rt_kprintf("时间读取失败!错误码: %d\n", ret);  
        return 1;
    }  
}






/*modbus读 那就是函数内部给buf赋值 7个值是*year,*month,*day,*hour,*min,*sec,*weekday*/
int modBus_get_rtc_data(void *buf, int bufsz) {
    uint16_t *ptr = (uint16_t *)buf;
    return api_rtc_get_time(&ptr[0],&ptr[1],&ptr[2],&ptr[3],&ptr[4],&ptr[5],&ptr[6]);
}

/*modbus写 那就是函数内部从buf取值去投入使用*/
int modBus_set_rtc_data(int index, int len, void * buf, int bufsz) {
    uint16_t *ptr = (uint16_t *)buf;
    return api_rtc_set_time(ptr[0],ptr[1],ptr[2],ptr[3],ptr[4],ptr[5]);
}




int ins5t8025_Init(void) {  
    return ins5t8025_init(INS5T8025_I2C_BUS);
}
INIT_COMPONENT_EXPORT(ins5t8025_Init);