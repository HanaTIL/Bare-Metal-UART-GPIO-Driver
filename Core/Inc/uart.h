/*
 * uart.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Hana TILOUCHE
 */
/* Includes ------------------------------------------------------------------*/

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "Time_Out.h"



/* Register location */

typedef struct
{
	volatile uint32_t MODER;    // 0x00
	volatile uint32_t OTYPER;   // 0x04
	volatile uint32_t OSPEEDR;  // 0x08
	volatile uint32_t PUPDR;    // 0x0C
	volatile uint32_t IDR;      // 0x10
	volatile uint32_t ODR;      // 0x14
	volatile uint32_t BSRR;     // 0x18
	volatile uint32_t LCKR;     // 0x1C
	volatile uint32_t AFRL;     // 0x20
	volatile uint32_t AFRH;     // 0x24
} GPIO_TypeDef;

/* @brief General Purpose I/O */

#define GPIOA ((GPIO_TypeDef *) 0x40020000)
#define GPIOB ((GPIO_TypeDef *) 0x40020400)
#define GPIOC ((GPIO_TypeDef *) 0x40020800)
#define GPIOD ((GPIO_TypeDef *) 0x40020C00)
#define GPIOE ((GPIO_TypeDef *) 0x40021000)
#define GPIOF ((GPIO_TypeDef *) 0x40021400)
#define GPIOG ((GPIO_TypeDef *) 0x40021800)
#define GPIOH ((GPIO_TypeDef *) 0x40021C00)
#define GPIOI ((GPIO_TypeDef *) 0x40022000)


/* USART registers */

typedef struct {
    volatile uint32_t SR;                //Status register
    volatile uint32_t DR;                //Data register
	volatile uint32_t BRR;               //Baud rate register
	volatile uint32_t CR1;               //Control register 1
} USART_TypeDef;

/* @brief USART	*/

#define USART2 ((USART_TypeDef *) 0x40004400)
#define USART3 ((USART_TypeDef *) 0x40004800)
#define UART4 ((USART_TypeDef *) 0x40004C00)
#define UART5 ((USART_TypeDef *) 0x40005000)


/* Pin Number */
#define Pin0                       00
#define Pin1                       01
#define Pin2                       02
#define Pin3                       03
#define Pin4                       04
#define Pin5                       05
#define Pin6                       06
#define Pin7                       07
#define Pin8                       08
#define Pin9                       09
#define Pin10                      10
#define Pin11                      11
#define Pin12                      12
#define Pin13                      13
#define Pin14                      14
#define Pin15                      15

/* GPIO Init structure definition */
/** @defgroup GPIO_pins_define */
#define GPIO_Pin_0                 ((unsigned short)0x0001)  /* Pin 0 selected */
#define GPIO_Pin_1                 ((unsigned short)0x0002)  /* Pin 1 selected */
#define GPIO_Pin_2                 ((unsigned short)0x0004)  /* Pin 2 selected */
#define GPIO_Pin_3                 ((unsigned short)0x0008)  /* Pin 3 selected */
#define GPIO_Pin_4                 ((unsigned short)0x0010)  /* Pin 4 selected */
#define GPIO_Pin_5                 ((unsigned short)0x0020)  /* Pin 5 selected */
#define GPIO_Pin_6                 ((unsigned short)0x0040)  /* Pin 6 selected */
#define GPIO_Pin_7                 ((unsigned short)0x0080)  /* Pin 7 selected */
#define GPIO_Pin_8                 ((unsigned short)0x0100)  /* Pin 8 selected */
#define GPIO_Pin_9                 ((unsigned short)0x0200)  /* Pin 9 selected */
#define GPIO_Pin_10                ((unsigned short)0x0400)  /* Pin 10 selected */
#define GPIO_Pin_11                ((unsigned short)0x0800)  /* Pin 11 selected */
#define GPIO_Pin_12                ((unsigned short)0x1000)  /* Pin 12 selected */
#define GPIO_Pin_13                ((unsigned short)0x2000)  /* Pin 13 selected */
#define GPIO_Pin_14                ((unsigned short)0x4000)  /* Pin 14 selected */
#define GPIO_Pin_15                ((unsigned short)0x8000)  /* Pin 15 selected */

/* Mode */
#define INPUT                      0x00      //Input(reset state)
#define OUTPUT                     0x01      //Output
#define AF                         0x02      //Alternate function
#define AN                         0x03      //Analog

/* typeOutput */
#define PP                         0x00      //Push-Pull
#define OD                         0x01      //Open-Drain

/* typeInput */

#define NPUPD                      0x00      //No pull-up, pull-down
#define PU                         0x01      //Pull-up
#define PD                         0x02      //Pull-down
#define R                          0x03      //Reserved

/*AFR selection*/

#define AF0                        0x00
#define AF1                        0x01
#define AF2                        0x02
#define AF3                        0x03
#define AF4                        0x04
#define AF5                        0x05
#define AF6                        0x06
#define AF7                        0x07
#define AF8                        0x08
#define AF9                        0x09
#define AF10                       0x10
#define AF11                       0x11
#define AF12                       0x12
#define AF13                       0x13
#define AF14                       0x14
#define AF15                       0x15


/* USART_CR1 */
#define USART_CR1_RE               (1U << 2)  //Receiver enable
#define USART_CR1_TE               (1U << 3)  //Transmitter enable
#define USART_CR1_UE               (1U << 13) //USART enable

/* USART_SR */

#define USART_SR_TXE                  (1U << 7)  //Transmit data register empty
#define USART_SR_TC                   (1U << 6)  //Transmission complete
#define USART_SR_RXNE                 (1U << 5)  // Read data register not empty


#define TE 0x01   // Transmitter enable
#define RE 0x02   // Receiver enable


/* UART enable function */

void UART_ClockEnable (USART_TypeDef * uart_x);

/* GPIO enable function */

void GPIO_ClockEnable (GPIO_TypeDef * gpio_x);

/* Config functions */
void GPIO_Init(GPIO_TypeDef * gpio_x, char Mode, char typeInput, char typeOutput, short int pin);
void UART_Init(USART_TypeDef * uart_x, unsigned int baud, char Mode);

void GPIO_SetAlternate(GPIO_TypeDef *gpio_x, uint8_t pin, uint8_t Af);


/* Write function */

void GPIO_WriteBit(GPIO_TypeDef * gpio_x, unsigned  short int GPIO_Pin, char BitVal);

/* Send function */

void UART_SendBuffer(USART_TypeDef * uart_x,  uint8_t *buffer, uint16_t length);

/* Receive function */

TypedefStatus UART_ReceiveBuffer(USART_TypeDef *uart_x, uint8_t *buffer, uint16_t length,  uint32_t timeout);

#endif /* UART_H_ */
