#include "console.h"
#include "stm32f405xx.h"
#include "pin.h"

// pin connections
// NOTE: this conflicts with the existing pin assignment for i2c temp sensor.
// for debugging the temp sensor IC and the R9 pullup resistor were tossed.
// PB10 = usart3 TX on AF7

#define USART_TX_PIN 10
#define USART_AF_NUM 7

static volatile bool s_console_init_complete = false;
static volatile USART_TypeDef * const s_console_usart = USART3;

void console_init()
{
  s_console_init_complete = true;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  pin_set_alternate_function(GPIOB, USART_TX_PIN, USART_AF_NUM);
  s_console_usart->CR1 &= ~USART_CR1_UE;
  s_console_usart->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  // APB1 speed is sysclock/4 = 42 MHz
  // we want 1 megabit. do this with mantissa=2 and fraction (sixteenths)=10
  // then baud rate = 42 MHz / 2.625 = 1 MHz
  s_console_usart->BRR  = (((uint16_t)2) << 4) | 10;
  s_console_usart->CR1 |=  USART_CR1_UE;
}

void console_send_block(const uint8_t *buf, uint32_t len)
{
  if (!s_console_init_complete)
    console_init();
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(s_console_usart->SR & USART_SR_TXE)) { } // wait for tx buffer to clear
    s_console_usart->DR = buf[i];
  }
  while (!(s_console_usart->SR & USART_SR_TC)) { } // wait for TX to finish
}

