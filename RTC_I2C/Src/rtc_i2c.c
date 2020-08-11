#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include "stm32f4xx_gpio.h"
#include "rtc_i2c.h"
#include "stm32f4xx_rcc.h"



/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 *      I2Cx --> the I2C peripheral e.g. I2C1
 *      address --> the 7 bit slave address
 *      direction --> the tranmission direction can be:
 *                      I2C_Direction_Tranmitter for Master transmitter mode
 *                      I2C_Direction_Receiver for Master receiver
 */
void I2C_start(uint8_t address, uint8_t direction) {
    // wait until I2C1 is not busy anymore
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
  
    // Send I2C1 START condition 
    I2C_GenerateSTART(I2C2, ENABLE);
      
    // wait for I2C1 EV5 --> Slave has acknowledged start condition
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
        
    // Send slave Address for write 
    I2C_Send7bitAddress(I2C2, address, direction);
      
    /* wait for I2C1 EV6, check if 
     * either Slave has acknowledged Master transmitter or
     * Master receiver mode, depending on the transmission
     * direction
     */ 
    if(direction == I2C_Direction_Transmitter){
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if(direction == I2C_Direction_Receiver){
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

/* This function transmits one byte to the slave device
 * Parameters:
 *      I2Cx --> the I2C peripheral e.g. I2C1 
 *      data --> the data byte to be transmitted
 */
void I2C_write(uint8_t data) {
    I2C_SendData(I2C2, data);
    // wait for I2C1 EV8_2 --> byte has been transmitted
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(void) {
    // enable acknowledge of recieved data
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    uint8_t data = I2C_ReceiveData(I2C2);
    return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(void) {
    // disabe acknowledge of received data
    // nack also generates stop condition after last byte received
    // see reference manual for more info
		uint16_t timeout = 0;
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) ) {
			timeout++;
			if (timeout == 0xFFFE) 
				break;
		}
    // read data from I2C data register and return data byte
    uint8_t data = I2C_ReceiveData(I2C2);
    return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(void) {
    // Send I2C1 STOP Condition 
    I2C_GenerateSTOP(I2C2, ENABLE);
}
