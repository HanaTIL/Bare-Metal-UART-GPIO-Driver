# STM32F4 Bare-Metal UART + GPIO Driver

This repository provides **a bare-metal implementation of UART and GPIO drivers for STM32F4 microcontrollers**, with optional transmit (TX) and receive (RX) modes, and timeout handling based on the **SysTick timer**.

The project demonstrates how to configure and operate USART2 at register level, while also providing a lightweight abstraction layer for GPIO configuration and control.

## Features

UART Driver in **polling mode**

Support for **USART2–USART5**

Configurable **baud rate** (tested with 9600 and 115200)

Selectable TX and RX operation (#define TX / #define RX)

**Timeout** support for receive operations

GPIO Driver

Clock enable for **GPIOA–GPIOG**

Configurable modes: **INPUT, OUTPUT, AF, AN**

Configurable I/O type: **PP (Push-Pull), OD (Open-Drain)**

**Configurable pull: NPUPD, PU, PD**

**Alternate function** configuration (AF0–AF15)

Bit-level write access

**SysTick Support**

System tick counter

Delay in milliseconds

Timeout mechanism for receive operations

LED Indicators

Activity and status feedback via GPIOD pins (PD12–PD15)

## Hardware Requirements


MCU: STM32F4xx (validated on STM32F407)

UART2 Pins:

PA2 → TX

PA3 → RX

LEDs (on GPIOD):

PD12 → Success indicator

PD13, PD15 → Activity indicators

PD14 → Error indicator

## Project Structure
│   ├── Src

│   │   ├── main.c          # Example application using TX/RX with LEDs

│   │   ├── uart_gpio.c     # UART + GPIO driver (register-level)

│   │   ├── Time_Out.c      # SysTick-based delay & timeout

│   ├── Inc

│   │   ├── uart.h          # Register mappings + driver prototypes

│   │   ├── Time_Out.h      # Timeout definitions and function prototypes


## Usage
1. Mode Selection

Define the operation mode in main.c:

//#define TX   // Enable for Transmit mode
#define RX    // Enable for Receive mode


TX Mode: Sends "Hello UART!" repeatedly

RX Mode: Waits for a fixed-length buffer

**Behavior:**

On success → PD12 set (success LED)

On timeout → PD14 set (error LED)

2. API Summary
   
**GPIO**
void GPIO_ClockEnable(GPIO_TypeDef *gpio_x);
void GPIO_Init(GPIO_TypeDef *gpio_x, char Mode, char typeInput, char typeOutput, short int pin);
void GPIO_SetAlternate(GPIO_TypeDef *gpio_x, uint8_t pin, uint8_t Af);
void GPIO_WriteBit(GPIO_TypeDef *gpio_x, unsigned short int GPIO_Pin, char BitVal);

**UART**
void UART_ClockEnable(USART_TypeDef *uart_x);
void UART_Init(USART_TypeDef *uart_x, unsigned int baud, char Mode);
void UART_SendBuffer(USART_TypeDef *uart_x, uint8_t *buffer, uint16_t length);
TypedefStatus UART_ReceiveBuffer(USART_TypeDef *uart_x, uint8_t *buffer, uint16_t length, uint32_t timeout);

**SysTick / Timeout**
void Ticks_Init(uint32_t freq);
uint32_t get_Ticks();
void delay(uint32_t delay_ms);

## Build & Flash

1. Open the project in **STM32CubeIDE**.  

2. In `main.c`, select the board role by defining one of the following:  
   ```c
   //#define TX_IT   // Uncomment for Transmitter
   #define RX_IT     // Uncomment for Receiver
3. Build and flash the firmware to each board.

4. Connect the hardware:

- **PA2 (TX)** of Transmitter → **PA3 (RX)** of Receiver
- **GND** of Transmitter ↔ **GND** of Receiver

## Example Behavior

**TX Mode:**

"Hello UART!" transmitted periodically over PA2

Activity LEDs (PD13, PD15) toggle

**RX Mode:**

Listens on PA3 for fixed-length buffer (12 bytes)

On successful reception → PD12 set (success LED)

On timeout/error → PD14 set (error LED)

Activity LEDs (PD13, PD15) toggle

**Notes**

Designed for bare-metal use (no HAL).

SysTick frequency must match core clock.

Currently fixed to 8 MHz PCLK for baud rate calculation
(adjust pclk in UART_Init() if needed).

**License**

Provided AS-IS for educational and embedded development purposes.



