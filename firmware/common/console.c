#include "console.h"
#include "soc.h"
#include "pin.h"

// pin connections
// NOTE: this conflicts with the existing pin assignment for i2c temp sensor.
// for debugging the temp sensor IC and the R9 pullup resistor were tossed.



#if defined(BOARD_blue)

// console TX = USART3_TX on PB10 AF7
#define USART_TX_PIN 10
#define USART_AF_NUM 7
#define USART_GPIO GPIOB
static volatile USART_TypeDef * const s_console_usart = USART3;

#elif defined(BOARD_mini)

// console TX = USART1_TX on PB6 AF7
#define USART_TX_PIN 6
#define USART_AF_NUM 7
#define USART_GPIO GPIOB
static volatile USART_TypeDef * const s_console_usart = USART1;

#endif

static volatile bool s_console_init_complete = false;

void console_init()
{
  s_console_init_complete = true;
  pin_set_alternate_function(USART_GPIO, USART_TX_PIN, USART_AF_NUM);
#if defined(BOARD_blue)
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  // APB1 speed is sysclock/4 = 42 MHz
  // we want 1 megabit. do this with mantissa=2 and fraction (sixteenths)=10
  // then baud rate = 42 MHz / 2.625 = 16 MHz baud clock -> 1 megabit
  const uint32_t brr = (((uint16_t)2) << 4) | 10;
#elif defined(BOARD_mini)
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  // APB2 speed is 168 MHz
  // we want 1 megabit. BRR = 168 / 16 = 10.5 = 10 plus 8/16 (fractional)
  const uint32_t brr = (((uint16_t)10) << 4) | 8;
#endif

  s_console_usart->CR1 &= ~USART_CR1_UE;
  s_console_usart->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  s_console_usart->BRR  = brr;
  s_console_usart->CR1 |=  USART_CR1_UE;
}

void console_send_block(const uint8_t *buf, uint32_t len)
{
  if (!s_console_init_complete)
    console_init();
#if defined(BOARD_blue)
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(s_console_usart->SR & USART_SR_TXE)) { } // wait for tx to clear
    s_console_usart->DR = buf[i];
  }
  while (!(s_console_usart->SR & USART_SR_TC)) { } // wait for TX to finish
#elif defined(BOARD_mini)
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(s_console_usart->ISR & USART_ISR_TXE)) { } // wait for tx to clear
    s_console_usart->TDR = buf[i];
  }
  while (!(s_console_usart->ISR & USART_ISR_TC)) { } // wait for TX to finish
#endif
}
