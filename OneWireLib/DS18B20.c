#include "ds18b20.h"
/*******************************************************************************
Noi Dung    : Cau hinh trang thai cua mot chan I/O.
Tham Bien   : None
Tra Ve      : Khong.
********************************************************************************/

static void Set_Pin_Output ()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;             
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
}

static void Set_Pin_Input ()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
}
/*-----------------------------------------------------------------------------
Noi Dung    :   Doc 1 byte du lieu tu DS18B20 ra ngoai. 
Tham Bien   :   Khong. 
Tra Ve      :   Byte du lieu can doc.
  -----------------------------------------------------------------------------*/ 
  
uint8_t DS18B20_ReadByte(void)
{   
    uint8_t i=0;
    uint8_t data=0;      
      
    for(i=8;i>0;i--)
    {    
				Set_Pin_Output ();	// Cau hinh chan DQ la OUPUT      
				GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET);  // Keo chan B2 xuong muc '0'
        data>>=1;
        GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET);  // Keo chan B2 len muc '1'     
				Set_Pin_Input ();		// Cau hinh chan DQ la INPUT
        if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)) data|=0x80;   // Nhan du lieu tra ve tu DS18B20
        mydelayus(120);                                                 
    }
    return(data);    
}	

/*-----------------------------------------------------------------------------
Noi Dung    :   Viet 1 byte du lieu vao DS18B20. 
Tham Bien   :   data: Byte du lieu can viet vao. 
Tra Ve      :   Khong.
  -----------------------------------------------------------------------------*/
  		
void DS18B20_WriteByte(uint8_t data)
{
    uint8_t i=0;
		Set_Pin_Output ();  
		// Cau hinh chan DQ la OUTPUT
    for (i=8;i>0;i--)
    {
        GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET);  // Keo chan B2 xuong muc '0'
				GPIO_WriteBit(GPIOB,GPIO_Pin_2,(data&0x01)); 
        mydelayus(60);
        GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET);  // Keo chan B2 len muc '1'  
        data>>=1;
    }    
}

/*-----------------------------------------------------------------------------
Noi Dung    :   Khoi tao DS18B20. 
Tham Bien   :   Khong. 
Tra Ve      :   Gia tri tra ve khi khoi tao xong cua DS18B20 (bit).
  -----------------------------------------------------------------------------*/
  	
void DS18B20_Init(void)
{
		Set_Pin_Output ();  // Cau hinh chan DQ la OUTPUT
    GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET);  // Keo chan B2 xuong muc '0'
    mydelayus(500);
		Set_Pin_Input ();	  // Cau hinh chan DQ la INPUT trong khoang 480us
    mydelayus(500);         
}

/*-----------------------------------------------------------------------------
Noi Dung    :   Ghi 1 byte lenh chuc nang vao DS18B20. 
Tham Bien   :   byte_func: byte lenh chuc nang cua DS18B20. 
Tra Ve      :   Khong.
  -----------------------------------------------------------------------------*/
  
void DS18B20_WriteFunc(uint8_t byte_func)
{
    DS18B20_Init();                 // Khoi tao DS18B20
    DS18B20_WriteByte(DS1820_CMD_SKIPROM);  // Truy cap thang den cac lenh chuc nang bo nho cua DS18B20
    DS18B20_WriteByte(byte_func);   // Viet ma lenh chuc nang
}

/*-----------------------------------------------------------------------------
Noi Dung    :   Cau hinh cho DS18B20. 
Tham Bien   :   temp_low: Gia tri nhiet do thap nhat de dua ra canh bao. 
                temp_high: Gia tri nhiet do cao nhat de dua ra canh bao.   
                resolution: Do phan giai cho DS18B20.(1|R1|R0|1|1|1|1|1)
Tra Ve      :   Khong.
  -----------------------------------------------------------------------------*/
  
void DS18B20_Config(uint8_t temp_low, uint8_t temp_high, uint8_t resolution)
{   
  resolution=(resolution<<5)|0x1f;  
  DS18B20_WriteFunc(DS1820_CMD_WRITESCRPAD);        // Cho phep ghi 3 byte vao bo nho nhap:    
	DS18B20_WriteByte(temp_high);   // byte 2: Th
	DS18B20_WriteByte(temp_low);    // byte 3: Tl 
	DS18B20_WriteByte(resolution);  // byte 4: configuration register
  DS18B20_WriteFunc(DS1820_CMD_COPYSCRPAD);        // Ghi vao EEPROM
}	                   

/*-----------------------------------------------------------------------------
Noi Dung    :   Doc gia tri nhiet do do duoc cua DS18B20. 
Tham Bien   :   Khong. 
Tra Ve      :   Gia tri nhiet do do duoc.
  -----------------------------------------------------------------------------*/
  											
float DS18B20_ReadTemp(void)
{
    float temp;
	uint8_t buff_temp1,buff_temp2; 
    
	DS18B20_WriteFunc(DS1820_CMD_CONVERTTEMP);  // Khoi dong qua trinh do va chuyen doi nhiet do ra so nhi phan
  mydelayus(200);
	DS18B20_WriteFunc(DS1820_CMD_READSCRPAD);   // Doc du lieu tu bo nho DS18b20 
    
	buff_temp1 = DS18B20_ReadByte(); 
	temp=((float)(buff_temp1&0x0f))/16;		    // Lay phan thuc cua gia tri nhiet do
	buff_temp2 = DS18B20_ReadByte(); 				
	buff_temp1 =((buff_temp1&0xf0)>>4)|((buff_temp2&0x0f)<<4) ;	// Lay phan nguyen cua gia tri nhiet do
	temp=temp+buff_temp1;
	return temp;	   
}
