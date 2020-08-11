#include "ds18b20_mflib.h"

extern void mydelayus(unsigned int time); // Delay us by Time2 ( Description in main.h )
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;            
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

/**
 * Initialises communication with the DS18B20 device.
 *
 * @param 	None.
 * @return 	1 if succeed, 0 if failed.
 */
uint8_t ds18b20_init_seq(void)
{
	//Set_Pin_Output(TXRX_PORT,TXRX_PIN);
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET);
	mydelayus(600);
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET);
	//Set_Pin_Input(TXRX_PORT,TXRX_PIN);
	mydelayus(100);
	if (GPIO_ReadInputDataBit(TXRX_PORT, TXRX_PIN) == Bit_RESET) {
		mydelayus(500);
		return 1;
	} else {
		mydelayus(500);
		return 0;
	}
}

/**
 * Sends the read rom command.
 *
 * @param 	None.
 * @return 	The lasered rom code.
 */
uint64_t ds18b20_read_rom_cmd(void)
{
	uint8_t crc, family_code;
	uint64_t lasered_rom_code, serial_num;

	ds18b20_write_byte(READ_ROM_CMD_BYTE);

	family_code = ds18b20_read_byte();
	serial_num = ds18b20_read_byte();
	serial_num |= (ds18b20_read_byte() << 8);
	serial_num |= (ds18b20_read_byte() << 16);
	crc = ds18b20_read_byte();

	lasered_rom_code = (crc << 24) | (serial_num << 8) |  family_code;

	return lasered_rom_code;

}

/**
 * Sends the function command.
 *
 * @param 	cmd: Number of the function command
 * @return  None.
 */
void ds18b20_send_function_cmd(uint8_t cmd)
{
	ds18b20_write_byte(cmd);
	//Set_Pin_Input(TXRX_PORT,TXRX_PIN);
	if (cmd == CONVERT_T_CMD) {
		while(GPIO_ReadInputDataBit(TXRX_PORT, TXRX_PIN) == Bit_RESET) { 
		/* wait for end of conversion */
		}
	}
}

/**
 * Sends the rom command.
 *
 * @param 	cmd: Number of the rom command
 * @return  None.
 */
void ds18b20_send_rom_cmd(uint8_t cmd)
{
	ds18b20_write_byte(cmd);
}

/**
 * Read the temperature.
 *
 * @param 	None.
 * @return  Float value of the last measured temperature.
 */
float ds18b20_read_temp(void)
{
	int8_t k;
	uint8_t temp_LSB, temp_MSB;
	uint16_t u16_temp, mask = 1;
	float temperature = 0.0;

	temp_LSB = ds18b20_read_byte();
	temp_MSB = ds18b20_read_byte();

	u16_temp = ((temp_MSB << 8) | temp_LSB);

	for (k = -4; k < 7; k++) {
		if (u16_temp & mask) {
			temperature += powf(2,k);
		}
		mask = mask << 1;
	}
	return temperature;
}

/**
 * Write byte to DS18B20.
 *
 * @param 	data: Data to be written.
 * @return  None.
 */
void ds18b20_write_byte(uint8_t data)
{
	//Set_Pin_Output(TXRX_PORT,TXRX_PIN);
	uint8_t i, mask = 0b00000001;
	uint8_t data_bit = data & mask;
 	for (i = 0; i < 8; i++) {
		if (data_bit != 0) {
			GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET);
			mydelayus(3);
			GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET);
			mydelayus(90);
		} else {
			GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET);
			mydelayus(90);
			GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET);
			mydelayus(3);
		}
		mask = mask << 1;
		data_bit = data & mask;
	}
}

/**
 * Read one bit from DS18B20.
 *
 * @param 	None.
 * @return  1 if succeed, 0 if failed.
 */
uint8_t ds18b20_read_bit(void)
{
	//Set_Pin_Output(TXRX_PORT,TXRX_PIN);
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET);
	mydelayus(2);
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET);
	mydelayus(10);
	//Set_Pin_Input(TXRX_PORT,TXRX_PIN);
	if (GPIO_ReadInputDataBit(TXRX_PORT, TXRX_PIN) == Bit_RESET) {
		mydelayus(58);
		return 0;
	} else {
		mydelayus(58);
		return 1;
	}
}

/**
 * Read one byte from DS18B20.
 *
 * @param 	None.
 * @return  One byte of data read from DS18B20.
 */
uint8_t ds18b20_read_byte(void)
{
	uint8_t i, data = 0;
	for (i = 0; i < 8; i++)
		data |= (ds18b20_read_bit() << i);
	return data;
}



