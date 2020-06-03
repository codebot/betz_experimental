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

#if defined(BOARD_blue)

// PB9 = SPI CS  as GPIO
// PC10 = SPI3 CLK  as AF6
// PC11 = SPI3 MISO as AF6
// PC12 = SPI3 MOSI as AF6

#define ENC_SPI SPI3
#define ENC_AF 6
#define ENC_IRQ SPI3_IRQn
#define ENC_VECTOR spi3_vector

#define ENC_CS_GPIO   GPIOB
#define ENC_SCLK_GPIO GPIOC
#define ENC_MOSI_GPIO GPIOC
#define ENC_MISO_GPIO GPIOC

#define ENC_CS_PIN    9
#define ENC_SCLK_PIN 10
#define ENC_MISO_PIN 11
#define ENC_MOSI_PIN 12

#elif defined(BOARD_mini)

#define ENC_SPI SPI2
#define ENC_AF 5
#define ENC_IRQ SPI2_IRQn
#define ENC_VECTOR spi2_vector

#define ENC_CS_GPIO   GPIOB
#define ENC_SCLK_GPIO GPIOB
#define ENC_MISO_GPIO GPIOB
#define ENC_MOSI_GPIO GPIOB

#define ENC_CS_PIN   11
#define ENC_SCLK_PIN 13
#define ENC_MISO_PIN 14
#define ENC_MOSI_PIN 15

#endif


#include <stdio.h>
#include <math.h>

#include "soc.h"

#include "enc.h"
#include "param.h"
#include "pin.h"
#include "state.h"

volatile float g_encoder_offset = 0;
volatile int g_encoder_dir = 0;

void enc_init()
{
  param_float(
      "encoder_offset",
      &g_encoder_offset,
      100,
      PARAM_PERSISTENT);

  param_int(
      "encoder_dir",
      &g_encoder_dir,
      1,
      PARAM_PERSISTENT);

  pin_set_output(ENC_CS_GPIO, ENC_CS_PIN, 1);
  pin_set_alternate_function(ENC_SCLK_GPIO, ENC_SCLK_PIN, ENC_AF);
  pin_set_alternate_function(ENC_MISO_GPIO, ENC_MISO_PIN, ENC_AF);
  pin_set_alternate_function(ENC_MOSI_GPIO, ENC_MOSI_PIN, ENC_AF);

  pin_set_output_speed(ENC_CS_GPIO, ENC_CS_PIN, 1);
  pin_set_output_speed(ENC_SCLK_GPIO, ENC_SCLK_PIN, 1);
  pin_set_output_speed(ENC_MISO_GPIO, ENC_MISO_PIN, 1);
  pin_set_output_speed(ENC_MOSI_GPIO, ENC_MOSI_PIN, 1);

#if defined(BOARD_blue)
  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
#elif defined(BOARD_mini)
  RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
#endif

  ENC_SPI->CR1 =
      SPI_CR1_CPHA |  // clock phase: cpha=1, cpol=0
      SPI_CR1_MSTR |  // master mode
      SPI_CR1_SSM  |  // set software slave select pin management
      SPI_CR1_SSI  ;  // assert software SSI pin

#if defined(BOARD_blue)

  // APB1 clock is 168/4 = 42 MHz
  // AS5047U max SPI rate is 10 MHz, so we have to use 42/8 = 5.25 MHz

  ENC_SPI->CR1 |=
      SPI_CR1_DFF  |  // 16 bit data frames
      SPI_CR1_BR_1 ;  // BR[2:0] = 010b  to divide PCLK by 8 = 5.25 MHz

#elif defined(BOARD_mini)

  // APB1 clock is 168 MHz
  // AS5047U max SPI rate is 10 MHz, so we need 168/32 = 5.25 MHz

  ENC_SPI->CR1 |=
      SPI_CR1_BR_2;  // BR[2:0] = 100b  to divide PCLK by 32 = 5.25 MHz

  ENC_SPI->CR2 |=
      (0xf << SPI_CR2_DS_Pos);  // DS[3:0] = 0xf  for 16-bit data transfers

#endif

  ENC_SPI->CR2 |= SPI_CR2_RXNEIE;  // enable receive interrupt
  ENC_SPI->CR1 |= SPI_CR1_SPE;  // enable SPI

  NVIC_SetPriority(ENC_IRQ, 3);  // lower than PWM and comms priority
  NVIC_EnableIRQ(ENC_IRQ);
}

uint16_t enc_read_pos_blocking()
{
  NVIC_DisableIRQ(ENC_IRQ);
  pin_set_output_low(ENC_CS_GPIO, ENC_CS_PIN);

  ENC_SPI->DR;  // flush RX with a dummy read
  ENC_SPI->DR = 0xffff;
  while (!(ENC_SPI->SR & SPI_SR_RXNE)) { }  // wait for it...

  pin_set_output_high(ENC_CS_GPIO, ENC_CS_PIN);

  uint16_t reading = ENC_SPI->DR & 0x3fff;  // returns the _previous_ read
  if (g_encoder_dir < 0)
    reading = 16383 - reading;

  NVIC_EnableIRQ(ENC_IRQ);

  return reading;
}

void enc_blocking_read_to_state()
{
  g_state.enc = enc_read_pos_blocking();
}

void enc_start_nonblocking_read_to_state()
{
  pin_set_output_low(ENC_CS_GPIO, ENC_CS_PIN);
  ENC_SPI->DR = 0xffff;
}

void ENC_VECTOR()
{
  pin_set_output_high(ENC_CS_GPIO, ENC_CS_PIN);
  const uint16_t pos_raw = ENC_SPI->DR & 0x3fff;  // returns the previous read
  float pos = pos_raw * (float)(2.0 * M_PI / 16384.0);

  if (g_encoder_dir < 0)
    pos *= -1.0f;

  pos += g_encoder_offset;

  if (pos < 0)
    pos += (float)(2 * M_PI);
  else if (pos > (float)(2 * M_PI))
    pos -= (float)(2 * M_PI);

  g_state.enc = pos;
}
