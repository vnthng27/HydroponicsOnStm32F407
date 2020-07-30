/**
 * @filename rtc_ds1307.c
 * @description DS1307 I2C Real Time Clock Driver for stm32f4 boards
 * @author Nicholas Shrake, <shraken@gmail.com>
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include "rtc_usart.h"
#include "rtc_i2c.h"
#include "rtc_ds1307.h"

#define BCD2BIN(val) (((val)&15) + ((val)>>4)*10)
#define BIN2BCD(val) ((((val)/10)<<4) + (val)%10)

int g_error_code;
unsigned int g_return_value;

int ds1307_set_rtc_data (char register_value, char data) {
    I2C_start((DS1307_ADDRESS << 1), I2C_Direction_Transmitter);
    I2C_write(register_value);
    I2C_write(data);
    I2C_stop();

    // otherwise success
    return RTC_DS1307_OK;
}

int ds1307_get_rtc_data (char register_value, char register_mask,
    unsigned int *return_value) {
    volatile uint8_t out_buff[1];
    volatile uint8_t in_buff[1];
    volatile uint32_t status;

    I2C_start((DS1307_ADDRESS << 1), I2C_Direction_Transmitter);
    I2C_write(register_value);
    I2C_stop();

    I2C_start((DS1307_ADDRESS << 1), I2C_Direction_Receiver);
    in_buff[0] = I2C_read_nack();
    
    // otherwise success
    *return_value = BCD2BIN(in_buff[0] & register_mask);
    return RTC_DS1307_OK;
}

void ds1307_set_rtc_second (unsigned int value) {
    ds1307_set_rtc_data(SECOND_REGISTER, (BIN2BCD(value) & SECOND_MASK));
}

void ds1307_set_rtc_minute (unsigned int value) {
    ds1307_set_rtc_data(MINUTE_REGISTER, (BIN2BCD(value) & MINUTE_MASK));
}

void ds1307_set_rtc_hour (unsigned int value) {
    ds1307_set_rtc_data(HOUR_REGISTER, (BIN2BCD(value) & HOUR_MASK));
}

void ds1307_set_rtc_day (unsigned int value) {
    ds1307_set_rtc_data(DAY_REGISTER, (BIN2BCD(value) & DAY_MASK));
}

void ds1307_set_rtc_date (unsigned int value) {
    ds1307_set_rtc_data(DATE_REGISTER, (BIN2BCD(value) & DATE_MASK));
}
void ds1307_set_rtc_month (unsigned int value) {
    ds1307_set_rtc_data(MONTH_REGISTER, (BIN2BCD(value) & MONTH_MASK));
}

void ds1307_set_rtc_year (unsigned int value) {
    ds1307_set_rtc_data(YEAR_REGISTER, (BIN2BCD(value) & YEAR_MASK));
}

int ds1307_set_rtc_datetime (rtc_ds1307_datetime_t* datetime) {
    // Set RTC datetime, the write format is documented on ds1307 pg. 13
    // byte0: word address = 0x00
    // byte1: (seconds)
    // byte2: (minutes)
    // 3: day
    // 4: date (day of month)
    // 5: month
    // 6: year (0-99)
    I2C_start((DS1307_ADDRESS << 1), I2C_Direction_Transmitter);
    I2C_write(0x00);
    I2C_write(BIN2BCD(datetime->seconds) & SECOND_MASK);
    I2C_write(BIN2BCD(datetime->minutes) & MINUTE_MASK);
    I2C_write(BIN2BCD(datetime->hour) & HOUR_MASK);
    I2C_write(BIN2BCD(datetime->day) & DAY_MASK);
    I2C_write(BIN2BCD(datetime->date) & DATE_MASK);
    I2C_write(BIN2BCD(datetime->month) & MONTH_MASK);
    I2C_write(BIN2BCD(datetime->year) & YEAR_MASK);
    I2C_stop();

    // otherwise success
    return RTC_DS1307_OK;
}


int ds1307_get_rtc_second (void) {
    return ds1307_get_rtc_data(SECOND_REGISTER, SECOND_MASK, &g_return_value);
}

int ds1307_get_rtc_minute (void) {
    return ds1307_get_rtc_data(MINUTE_REGISTER, MINUTE_MASK, &g_return_value);
}

int ds1307_get_rtc_hour (void) {
    return ds1307_get_rtc_data(HOUR_REGISTER, HOUR_MASK, &g_return_value);
}

int ds1307_get_rtc_day (void) {
    return ds1307_get_rtc_data(DAY_REGISTER, DAY_MASK, &g_return_value);
}

int ds1307_get_rtc_date (void) {
    return ds1307_get_rtc_data(DATE_REGISTER, DATE_MASK, &g_return_value);
}

int ds1307_get_rtc_month (void) {
    return ds1307_get_rtc_data(MONTH_REGISTER, MONTH_MASK, &g_return_value);
}

int ds1307_get_rtc_year (void) {
    return ds1307_get_rtc_data(YEAR_REGISTER, YEAR_MASK, &g_return_value);
}

int ds1307_get_rtc_datetime (rtc_ds1307_datetime_t *datetime)
{
    // seconds
    if (ds1307_get_rtc_second() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->seconds = g_return_value;

    // minutes
    if (ds1307_get_rtc_minute() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->minutes = g_return_value;

    // hours
    if (ds1307_get_rtc_hour() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->hour = g_return_value;

    // day
    if (ds1307_get_rtc_day() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->day = g_return_value;

    // date
    if (ds1307_get_rtc_date() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->date = g_return_value;

    // month
    if (ds1307_get_rtc_month() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->month = g_return_value;

    // year
    if (ds1307_get_rtc_year() != RTC_DS1307_OK)
        return RTC_DS1307_BAD;
    else
        datetime->year = g_return_value;

    // otherwise, success
    return RTC_DS1307_OK;
}

int ds1307_init_rtc (int first_run_flag) {
    // build the default (zero) starting state for the RTC
    // datetime structure.
    rtc_ds1307_datetime_t init_datetime = {
        .year = 0,
        .month = 0,
        .date = 0,
        .day = 0,
        .hour = 0,
        .minutes = 0,
        .seconds = 0
    };

    if (ds1307_set_rtc_data(CONTROL_REGISTER, 0) == -1) {
        return RTC_DS1307_BAD;
    }


    if (first_run_flag) {
        return ds1307_set_rtc_datetime(&init_datetime);
    }

    // otherwise return 0 to indicate success
    return RTC_DS1307_OK;
}
