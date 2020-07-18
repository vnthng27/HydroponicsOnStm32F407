/**
 * @filename rtc_i2c.h
 * @description I2C support library for stm32f4 board
 * @author Nicholas Shrake, <shraken@gmail.com>
 */

#ifndef RTC_I2C_H
#define RTC_I2C_H
#include "stm32f4xx.h" 
/**
 * CONSTANTS
 */

/**
 * PROTOTYPES
 */

void I2C_start(uint8_t address, uint8_t direction);
void I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);
void I2C_stop(void);

#endif
