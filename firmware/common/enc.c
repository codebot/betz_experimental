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

// PB9 = SPI CS  as GPIO
// PC10 = SPI3 CLK  as AF6
// PC11 = SPI3 MISO as AF6
// PC12 = SPI3 MOSI as AF6

#define ENC_SPI SPI3

#define ENC_CS_GPIO   GPIOB
#define ENC_SCLK_GPIO GPIOC
#define ENC_MOSI_GPIO GPIOC
#define ENC_MISO_GPIO GPIOC

#define ENC_CS_PIN    9
#define ENC_SCLK_PIN 10
#define ENC_MISO_PIN 11
#define ENC_MOSI_PIN 12

#include <stdio.h>
#include <math.h>

#include "soc.h"

#include "enc.h"
#include "param.h"
#include "pin.h"
#include "state.h"

volatile int g_encoder_offset = 0;
volatile int g_encoder_dir = 0;

void enc_init()
{
  param_int(
      "encoder_offset",
      &g_encoder_offset,
      100,
      PARAM_PERSISTENT);

  param_int(
      "encoder_dir",
      &g_encoder_dir,
      1,
      PARAM_PERSISTENT);

#if defined(BOARD_blue)
  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

  pin_set_output(ENC_CS_GPIO, ENC_CS_PIN, 1);
  pin_set_alternate_function(ENC_SCLK_GPIO, ENC_SCLK_PIN, 6);
  pin_set_alternate_function(ENC_MISO_GPIO, ENC_MISO_PIN, 6);
  pin_set_alternate_function(ENC_MOSI_GPIO, ENC_MOSI_PIN, 6);

  pin_set_output_speed(ENC_CS_GPIO, ENC_CS_PIN, 1);
  pin_set_output_speed(ENC_SCLK_GPIO, ENC_SCLK_PIN, 1);
  pin_set_output_speed(ENC_MISO_GPIO, ENC_MISO_PIN, 1);
  pin_set_output_speed(ENC_MOSI_GPIO, ENC_MOSI_PIN, 1);

  // APB1 clock is 168/4 = 42 MHz
  // AS5047U max SPI rate is 10 MHz, so we have to use 42/8 = 5.25 MHz

  ENC_SPI->CR1 =
      SPI_CR1_CPHA |  // clock phase: cpha=1, cpol=0
      SPI_CR1_MSTR |  // master mode
      SPI_CR1_DFF  |  // 16 bit data frames
      SPI_CR1_BR_1 |  // set BR[2:0] = 010b  to divide PCLK by 8 = 5.25 MHz
      SPI_CR1_SSM  |  // set software slave select pin management
      SPI_CR1_SSI  ;  // assert software SSI pin

  // todo: set interrupts, if we care in the future...
  ENC_SPI->CR2 =
      SPI_CR2_RXNEIE;  // enable receive interrupt

  ENC_SPI->CR1 |= SPI_CR1_SPE;  // enable SPI

  NVIC_SetPriority(SPI3_IRQn, 3);  // lower than PWM and comms priority
  NVIC_EnableIRQ(SPI3_IRQn);
#elif defined(BOARD_mini)
  // TODO
#endif
}

uint16_t enc_read_pos_blocking()
{
#if defined(BOARD_blue)
  NVIC_DisableIRQ(SPI3_IRQn);
  pin_set_output_low(ENC_CS_GPIO, ENC_CS_PIN);

  ENC_SPI->DR;  // flush RX with a dummy read
  ENC_SPI->DR = 0xffff;
  while (!(ENC_SPI->SR & SPI_SR_RXNE)) { }  // wait for it...

  pin_set_output_high(ENC_CS_GPIO, ENC_CS_PIN);

  uint16_t reading = ENC_SPI->DR & 0x3fff;  // returns the _previous_ read
  if (g_encoder_dir < 0)
    reading = 16383 - reading;

  NVIC_EnableIRQ(SPI3_IRQn);

  return reading;
#elif defined(BOARD_mini)
  // TODO
  return 42;
#endif
}

void enc_blocking_read_to_state()
{
  g_state.enc = enc_read_pos_blocking();
}

void enc_start_nonblocking_read_to_state()
{
#if defined(BOARD_blue)
  pin_set_output_low(ENC_CS_GPIO, ENC_CS_PIN);
  ENC_SPI->DR = 0xffff;
#elif defined(BOARD_mini)
  // TODO
#endif
}

#if defined(BOARD_blue)
void spi3_vector()
{
  pin_set_output_high(ENC_CS_GPIO, ENC_CS_PIN);
  uint16_t position = ENC_SPI->DR & 0x3fff;  // returns the _previous_ read
  g_state.enc = (position + g_encoder_offset) * (float)(2.0 * M_PI / 16384.0);
}
#elif defined(BOARD_mini)
  // TODO
#endif
