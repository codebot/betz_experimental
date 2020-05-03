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

#include "enc.h"
#include "pin.h"
#include "stm32f405xx.h"

void enc_init()
{
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

  ENC_SPI->CR1 |= SPI_CR1_SPE;  // enable SPI
}

uint16_t enc_read_pos_blocking()
{
  pin_set_output_low(ENC_CS_GPIO, ENC_CS_PIN);

  ENC_SPI->DR;  // flush RX with a dummy read
  ENC_SPI->DR = 0xffff;
  while (!(ENC_SPI->SR & SPI_SR_RXNE)) { }  // wait for it...

  pin_set_output_high(ENC_CS_GPIO, ENC_CS_PIN);

  return ENC_SPI->DR & 0x3fff;  // returns the _previous_ read, not current!
}
