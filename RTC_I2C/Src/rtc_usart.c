#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include "rtc_usart.h"

/* This function is used to transmit a string of characters via 
 * the USART specified in USARTx.
 * 
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 * 
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 * 
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(volatile char *s) {
    while(*s){
		  // wait until data register is empty
		  while( !(USART1->SR & 0x00000040) ) ;
		      USART_SendData(USART1, *s);
		  *s++;
    }
}
