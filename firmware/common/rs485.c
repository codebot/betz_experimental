/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <stdio.h>
#include "comms.h"
#include "status_led.h"
#include "pin.h"
#include "rs485.h"
#include "soc.h"

#if defined(BOARD_blue)

// pin connections
// PB6 = usart1 TX
// PB7 = usart1 RX
// PA12 = enable termination resistor (via analog switch)

#define RS485_USART_GPIO GPIOB
#define RS485_USART USART1
#define TX_PIN 6
#define RX_PIN 7

#define RS485_DIR_GPIO GPIOD
#define RS485_DIR_PIN 2

#define RS485_TERM_GPIO GPIOA
#define RS485_TERM_PIN 12

#elif defined(BOARD_mini)
  // TODO
#endif

#define RS485_RX_RING_LEN 512
static volatile uint8_t g_rs485_rx_ring[RS485_RX_RING_LEN];
static volatile uint32_t g_rs485_rx_ring_rpos = 0;
static volatile uint32_t g_rs485_rx_ring_wpos = 0;

void rs485_init()
{
#if defined(BOARD_blue)
  pin_set_output(RS485_TERM_GPIO, RS485_TERM_PIN, 0);
  rs485_enable_termination(true);  // todo: look up in param table

  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  pin_set_output(RS485_DIR_GPIO, RS485_DIR_PIN, 0);
  pin_set_alternate_function(RS485_USART_GPIO, TX_PIN, 7);  // AF7 = USART3 TX
  pin_set_alternate_function(RS485_USART_GPIO, RX_PIN, 7);  // AF7 = USART3 RX

  RS485_USART->CR1 &= ~USART_CR1_UE;

  // peripheral clock = 84 MHz

  // 1 Mbit: divisor = 84/16 = 5.25 => mantissa = 5, fraction (16'ths) = 4
  // RS485_USART->BRR = (5 << 4) | 4;

  // 3 Mbit: divisor = 84/48 = 1.75 => mantissa = 1, fraction (16'ths) = 12
  RS485_USART->BRR = (1 << 4) | 12;

  // 6 Mbit: divisor = 84/48 = 1.75 => mantissa = 1, fraction (8'ths) = 6
  // must use oversample-by-8 mode (not 16)
  // 6 MBit looks fine on the oscilloscope but it seems tricky to get
  // FT232H to work at this speed; up to 3 Mbps is easy, beyond is not easy.
  // so for now, we'll stick to 3 Mbit.

  RS485_USART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  RS485_USART->CR1 |= USART_CR1_UE;

  // Unfortunately, we have to set the highest prirority to the comms RX
  // interrupt handler because there is only a 1-byte RX buffer. Otherwise
  // we might drop inbound bytes. This handler has to be super super fast.
  NVIC_SetPriority(USART1_IRQn, 0);  // highest priority (!)
  NVIC_EnableIRQ(USART1_IRQn);
#elif defined(BOARD_mini)
  // TODO
#endif
}

#if defined(BOARD_blue)
void rs485_enable_termination(const bool enable)
{
  if (enable)
    pin_set_output_high(RS485_TERM_GPIO, RS485_TERM_PIN);
  else
    pin_set_output_low(RS485_TERM_GPIO, RS485_TERM_PIN);
}
#endif

void rs485_tx(const uint8_t *data, const uint32_t len)
{
#if defined(BOARD_blue)
  pin_set_output_high(RS485_DIR_GPIO, RS485_DIR_PIN);  // enable transmitter
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(RS485_USART->SR & USART_SR_TXE)) { } // wait for tx to clear
    RS485_USART->DR = data[i];
  }
  while (!(RS485_USART->SR & USART_SR_TC)) { } // wait for TX to finish
  pin_set_output_low(RS485_DIR_GPIO, RS485_DIR_PIN);  // disable transmitter
#elif defined(BOARD_mini)
  // TODO
#endif
}

#if defined(BOARD_blue)
// void usart1_vector() __attribute__((section(".ramfunc")));
void usart1_vector()
{
  volatile uint8_t __attribute__((unused)) sr = USART1->SR;  // clear errors
  volatile uint8_t b = USART1->DR;  // drain
  g_rs485_rx_ring[g_rs485_rx_ring_wpos] = b;
  if (++g_rs485_rx_ring_wpos >= RS485_RX_RING_LEN)
    g_rs485_rx_ring_wpos = 0;
}
#endif

void rs485_tick()
{
  // called during the CPU's idle time
  while (g_rs485_rx_ring_rpos != g_rs485_rx_ring_wpos)
  {
    comms_rx_byte(g_rs485_rx_ring[g_rs485_rx_ring_rpos]);
    if (++g_rs485_rx_ring_rpos >= RS485_RX_RING_LEN)
      g_rs485_rx_ring_rpos = 0;
  }
}
