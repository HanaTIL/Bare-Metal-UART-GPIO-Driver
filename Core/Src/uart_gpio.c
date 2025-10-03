/*
 * uart_gpio.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Hana TILOUCHE
 */
/* Includes ------------------------------------------------------------------*/

#include "uart.h"
#include "Time_Out.h"
#include <stdint.h>


volatile uint32_t *RCC_GPIO_Clock = (uint32_t *)(0x40023800 + 0x30) ;
volatile uint32_t *RCC_UART_Clock = (uint32_t *)(0x40023800 + 0x40) ;


/**
 * @brief  Enable clock for the uart_x peripheral.
 * @param  uart_x: where x can be (2..5) to select the uart peripheral.
 * @retval None
 */

void UART_ClockEnable (USART_TypeDef * uart_x)
{
	 if (uart_x ==  USART2)
	 {
		 *RCC_UART_Clock |= (1U<<17);
	 }

	 else if (uart_x ==  USART3)
	 {
		 *RCC_UART_Clock |= (1U<<18);

	 }
	 else if (uart_x == UART4)
	 {
		 *RCC_UART_Clock |= (1U<<19);
	 }

	 else
	 {
		 if (uart_x == UART5)
	     {
			 *RCC_UART_Clock |= (1U<<20);

	     }
	 }

}

/**
 * @brief  Enable clock for the gpio_x peripheral.
 * @param  gpio_x: where x can be (A..G) to select the GPIO peripheral.
 * @retval None
 */

void GPIO_ClockEnable (GPIO_TypeDef * gpio_x)
{
	 if (gpio_x == GPIOA)
	 {
		 *RCC_GPIO_Clock |= (1U<<0);
	 }

	 else if (gpio_x == GPIOB)
	 {
		 *RCC_GPIO_Clock |= (1U<<1);

	 }
	 else if (gpio_x == GPIOC)
	 {
		 *RCC_GPIO_Clock |= (1U<<2);
	 }

	 else if (gpio_x == GPIOD)
	 {
		*RCC_GPIO_Clock |= (1<<3);
	 }

	 else if (gpio_x == GPIOE)
	 {
	 	*RCC_GPIO_Clock |= (1U<<4);

	 }
	 else if (gpio_x == GPIOF)
	 {
	 	 *RCC_GPIO_Clock |= (1U<<5);

	 }
	 else
	 {
		 if (gpio_x == GPIOG)
	     {
			 *RCC_GPIO_Clock |= (1U<<6);

	     }
     }
}

 /**
 * @brief  Configure the gpio_x
 * @param  gpio_x: where x can be (A..G) to select the GPIO peripheral.
 * @param  Mode: can be INPUT, OUTPUT, AF or AN
 * @param  typeOutput: can be PP or OD
 * @param  typeInput: can be NPUPD, PU, PD, R
 * @param  pin: can be 0...15
 * @retval None
 */


void GPIO_Init(GPIO_TypeDef * gpio_x, char Mode, char typeInput, char typeOutput, short int pin)
{
	 unsigned int masque = ~(0x03 << (pin*2));
	 gpio_x->MODER &= masque;
	 gpio_x->MODER |= (Mode << (pin*2));
	 gpio_x->PUPDR &= masque;
	 gpio_x->PUPDR |= (typeInput << (pin*2));
	 if(typeOutput == PP)
	 {
		 gpio_x->OTYPER &= ~(0x1 << pin);
	 }
	 else
	 {
		 gpio_x->OTYPER |= (0x1 << pin);
	 }

}
/**
* @brief  Set alternate function
* @param  gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param  pin: can be 0...15
* @param  Af: can be AF0 .... AF15
* @retval None
*/

void GPIO_SetAlternate(GPIO_TypeDef *gpio_x, uint8_t pin, uint8_t Af)
{
	if (pin < 8){
		unsigned int shift = pin*4;
		unsigned int masque = ~(0xF << (shift));
		gpio_x->AFRL &= masque;
		gpio_x->AFRL |= (Af << (shift));
	}
	else{
		unsigned int shift = (pin-8)*4;
		unsigned int masque = ~(0xF << (shift));
		gpio_x->AFRH &= masque;
		gpio_x->AFRH |= (Af << (shift));
	}


}


/**
 * @brief  Sets or clears the selected data port bit.
 * @param  gpio_x: where x can be (A..G) to select the GPIO peripheral.
 * @param  GPIO_Pin: specifies the port bit to be written.
 *   This parameter can be one of GPIO_Pin_x where x can be (0..15).
 * @param  BitVal: specifies the value to be written to the selected bit.
 *   This parameter can be one of the BitAction enum values:
 *     @arg Bit_RESET: to clear the port pin
 *     @arg Bit_SET: to set the port pin
 * @retval None
 */
 void GPIO_WriteBit(GPIO_TypeDef * gpio_x, unsigned  short int GPIO_Pin, char BitVal)
 {
	  if(BitVal != 0x0)
	  {
		  gpio_x->ODR |= GPIO_Pin;

	  }
	  else
	  {
		  gpio_x->ODR &= ~GPIO_Pin;

	  }
  }

/**
 * @brief  Enable USART, TX, RX
 * @param  uart_x: where x can be (2..5) to select the UART peripheral.
 * @param  baud: can be 9600 or 115200
 * @param  Mode: can be TE, RE.
 * @retval None
 */

void UART_Init(USART_TypeDef * uart_x, unsigned int baud, char Mode)
{
	 uint32_t pclk = 8000000;
	 uint32_t usartdiv = pclk / (16 * baud);
	 uint32_t fraction = ( (pclk % (16 * baud)) * 16 ) / (16 * baud);
	 uint32_t brr = (usartdiv << 4) | (fraction & 0xF);

	 uart_x->BRR = brr;

	 if(Mode & TE){
		 uart_x->CR1 |= USART_CR1_UE | USART_CR1_TE;
	 }
	 if (Mode & RE){
		 uart_x->CR1 |= USART_CR1_UE | USART_CR1_RE;
	 }

}




/**
 * @brief  Send data
 * @param  uart_x: where x can be (2..5) to select the UART peripheral.
 * @param  buffer: Pointer to data buffer.
 * @param  length: Amount of data elements to be sent.
 * @retval None
 */

void UART_SendBuffer(USART_TypeDef * uart_x,  uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        while (!(uart_x->SR & USART_SR_TXE)); // wait TX buffer empty
        uart_x->DR = buffer[i];               // send byte
    }
    while (!(uart_x->SR & USART_SR_TC)); // wait transmission complete
       // Blink LED here when finished
    GPIO_WriteBit(GPIOD, GPIO_Pin_12, 0x01);
}

/**
 * @brief  Receive data
 * @param  uart_x: where x can be (2..5) to select the UART peripheral.
 * @param  buffer: Pointer to data buffer.
 * @param  length: Amount of data elements to be received.
 * @retval None
 */

TypedefStatus UART_ReceiveBuffer(USART_TypeDef *uart_x, uint8_t *buffer, uint16_t length,  uint32_t timeout)
{
    for (uint16_t i = 0; i < length; i++)
    {
    	uint32_t Tick_start=get_Ticks();
        while (!(uart_x->SR & USART_SR_RXNE))
        {
        	if(get_Ticks() - Tick_start > timeout)
        	{
        		return Error;
        	}
        }
        buffer[i] = (uint8_t)(uart_x->DR & 0xFF);

    }
    return OK;
}

