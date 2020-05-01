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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "comms.h"
#include "flash.h"
#include "param.h"
#include "rng.h"
#include "rs485.h"
#include "systime.h"
#include "uuid.h"

// wire formats:
//  * initial message byte is always 0xbe, to allow easier re-sync
//  * next byte is a set of flags:
//    * bit 0 = direction: 0 = host->peripheral, 1 = peripheral->host
//    * bit 1 = broadcast: 0 = intended for single recipient, 1 = broadcast
//    * bit 2 = address format, if not broadcast:
//       * 0 = single-byte 'id' parameter, must be set previously
//       * 1 = 12-byte unique hardware serial number
//    * bit 3 = currently unused. Set to zero.
//    * bit 4 = 1
//    * bit 5 = 0
//    * bit 6 = 1
//    * bit 7 = 0
//  * if not broadcast, peripheral address follows (1 byte or 12 bytes)
//  * next byte is the payload length
//  * then the payload
//  * finally a 16-bit checksum (eventually a CRC-16)

#define COMMS_LONG_ADDR_LEN 12
#define COMMS_PKT_MAX_LEN 256

typedef enum
{
  PS_PREAMBLE = 0,
  PS_FLAGS,
  PS_ADDRESS,
  PS_LENGTH,
  PS_PAYLOAD,
  PS_CSUM_0,
  PS_CSUM_1
} comms_parser_state_t;

// local variables
static comms_parser_state_t g_comms_parser_state = PS_PREAMBLE;
static uint32_t g_comms_parser_expected_length = 0;
static uint32_t g_comms_parser_wpos = 0;
static uint8_t  g_comms_parser_pkt[COMMS_PKT_MAX_LEN] = {0};

static uint16_t g_comms_parser_crc = 0;
static uint16_t g_comms_parser_rx_crc = 0;

static void (*g_comms_raw_tx_fptr)(const uint8_t *, const uint32_t) = 0;
static bool g_comms_parser_broadcast = false;
static bool g_comms_parser_long_address = false;
static bool g_comms_parser_ignore_pkt = false;
static uint8_t g_comms_parser_addr[COMMS_LONG_ADDR_LEN] = {0};
static uint8_t g_comms_id = 0;
static uint8_t g_comms_is_bootloader = 0;

static uint32_t g_comms_discovery_send_time = 0;

// local functions
static void comms_rx_pkt(const uint8_t *p, const uint32_t len);
static void comms_tx(const uint8_t *p, const uint32_t len);
static void comms_tx_long_addr(const uint8_t *p, const uint32_t len);

/////////////////////////////////////////////////////////////////////

void comms_init(const uint8_t is_bootloader)
{
  g_comms_is_bootloader = is_bootloader;
  g_comms_parser_state = PS_PREAMBLE;
}

void comms_tick()
{
  if (g_comms_discovery_send_time)
  {
    const uint32_t t = systime_read();
    if (t >= g_comms_discovery_send_time)
    {
      g_comms_discovery_send_time = 0;
      uint8_t pkt[2] = {0};
      pkt[0] = 0xf0;
      pkt[1] = g_comms_is_bootloader;
      comms_tx_long_addr(pkt, 2);
    }
  }
}

void comms_set_raw_tx_fptr(void (*fptr)(const uint8_t *, const uint32_t))
{
  g_comms_raw_tx_fptr = fptr;
}

static inline uint16_t comms_crc_byte(const uint16_t crc, const uint8_t b)
{
  // placeholder for now. replace with real CRC-16 (usb csum) sometime
  // for now, just barrel-left-shift and XOR with the input byte
  return (((crc & 0x8000) >> 15) | (crc << 1)) ^ b;
}

static uint16_t comms_crc(const uint8_t *p, const uint32_t len)
{
  uint16_t crc = 0;
  for (uint32_t i = 0; i < len; i++)
    crc = comms_crc_byte(crc, p[i]);

  return crc;
}

void comms_rx_byte(const uint8_t b)
{
  // printf("rx 0x%02x state %d\n", (unsigned)b, (int)g_comms_parser_state);
  switch (g_comms_parser_state)
  {
    case PS_PREAMBLE:
      if (b == 0xbe)
      {
        g_comms_parser_crc = comms_crc_byte(0, b);  // reset crc
        g_comms_parser_state = PS_FLAGS;
      }
      break;

    case PS_FLAGS:
      g_comms_parser_crc = comms_crc_byte(g_comms_parser_crc, b);
      // verify the top 4 bits are as expected
      if ((b & 0xf0) != 0x50)
      {
        g_comms_parser_state = PS_PREAMBLE;
        break;
      }
      // if the peripheral->host bit is set, the packet is not intended for us
      g_comms_parser_ignore_pkt = (b & 0x1);
      g_comms_parser_broadcast = (b & 0x2);
      g_comms_parser_long_address = (b & 0x4);
      g_comms_parser_wpos = 0;

      if (g_comms_parser_broadcast)
        g_comms_parser_state = PS_LENGTH;
      else
        g_comms_parser_state = PS_ADDRESS;
      break;

    case PS_ADDRESS:
      g_comms_parser_crc = comms_crc_byte(g_comms_parser_crc, b);
      if (g_comms_parser_long_address)
      {
        g_comms_parser_addr[g_comms_parser_wpos] = b;
        g_comms_parser_wpos++;
        if (g_comms_parser_wpos == COMMS_LONG_ADDR_LEN)
        {
          // verify UUID
          for (int i = 0; i < UUID_LEN; i++)
            if (g_uuid[i] != g_comms_parser_addr[i])
            {
              g_comms_parser_ignore_pkt = true;
              break;
            }
          g_comms_parser_state = PS_LENGTH;
        }
      }
      else
      {
        if (b != g_comms_id)
          g_comms_parser_ignore_pkt = true;
        g_comms_parser_state = PS_LENGTH;
      }
      break;

    case PS_LENGTH:
      g_comms_parser_crc = comms_crc_byte(g_comms_parser_crc, b);
      if (b == 0)
      {
        g_comms_parser_state = PS_PREAMBLE;  // this shouldn't happen...
        break;
      }

      g_comms_parser_expected_length = b;
      g_comms_parser_wpos = 0;
      g_comms_parser_state = PS_PAYLOAD;
      break;

    case PS_PAYLOAD:
      g_comms_parser_crc = comms_crc_byte(g_comms_parser_crc, b);
      g_comms_parser_pkt[g_comms_parser_wpos] = b;
      // printf("  payload %d/%d\n",
      //     g_parser_wpos, g_parser_expected_length);
      if (g_comms_parser_wpos == g_comms_parser_expected_length - 1)
        g_comms_parser_state = PS_CSUM_0;
      if (g_comms_parser_wpos < COMMS_PKT_MAX_LEN - 1)
        g_comms_parser_wpos++;
      break;

    case PS_CSUM_0:
      g_comms_parser_state = PS_CSUM_1;
      g_comms_parser_rx_crc = b;
      break;

    case PS_CSUM_1:
      g_comms_parser_state = PS_PREAMBLE;
      g_comms_parser_rx_crc |= (b << 8);
      if (g_comms_parser_rx_crc == g_comms_parser_crc)
      {
        // printf("  pkt csum ok\n");
        if (g_comms_parser_ignore_pkt)
        {
          // printf("ignoring packet\r\n");
          break;
        }
        comms_rx_pkt(g_comms_parser_pkt, g_comms_parser_expected_length);
      }
      else
      {
        printf("csum fail: 0x%0x != 0x%x\n",
            g_comms_parser_crc,
            g_comms_parser_rx_crc);
      }
      break;

    default:
      g_comms_parser_state = PS_PREAMBLE;
      break;
  }
}

void comms_read_flash(const uint8_t *data, const uint32_t len)
{
  if (len < 9)
    return;  // must have >= 9 bytes in request message
  uint32_t read_addr = 0, read_len = 0;
  memcpy(&read_addr, &data[1], sizeof(read_addr));
  memcpy(&read_len, &data[5], sizeof(read_len));
  printf("comms_read_flash()\r\n");
  if (read_len > 128)
  {
    printf("invalid flash read: len = %d\r\n", (int)read_len);
    return;  // cannot. too long.
  }
  // sanity check to make sure the address range lies in flash
  if (read_addr < 0x08000000 || read_addr > 0x080fffff)
  {
    printf("invalid flash read: addr = 0x%08x\r\n", (unsigned)read_addr);
    return;  // cannot. outside flash.
  }
  uint8_t pkt[128 + 9] = {0};  // max length of return request
  pkt[0] = 0xf1;
  memcpy(&pkt[1], &read_addr, sizeof(read_addr));
  memcpy(&pkt[5], &read_len, sizeof(read_len));
  flash_read(read_addr, read_len, (uint8_t *)&pkt[9]);
  comms_tx_long_addr(pkt, 9 + read_len);
}

void comms_discovery(const uint8_t *p, const uint32_t len)
{
  printf("comms_discovery()\n");
  if (len >= 3)
  {
    const uint16_t req_max_delay = (uint16_t)(p[1]) | (p[2] << 8);
    uint32_t sampled_delay = rng_read() % (uint32_t)req_max_delay;
    g_comms_discovery_send_time = systime_read() + sampled_delay;
    printf("discovery response in %u usec\n", (unsigned)sampled_delay);
    // for now, ignore the corner cases where this equals zero or wraparound
    // we'll query multiple times on the host to enumerate anyway.
  }
}

void comms_num_params()
{
  printf("comms_num_params()\n");
  uint8_t pkt[5] = {0};
  pkt[0] = 0x01;
  const uint32_t num_params = param_count();
  memcpy(&pkt[1], &num_params, 4);
  comms_tx_long_addr(pkt, 5);
}

void comms_rx_pkt(const uint8_t* p, const uint32_t len)
{
  // printf("comms_rx_pkt() received %d bytes\r\n", (int)len);
  if (len == 0)
    return;  // adios amigo
  //for (uint32_t i = 0; i < len; i++)
  //  printf("%02d: %02x\r\n", (int)i, (unsigned)data[i]);
  const uint8_t pkt_id = p[0];
  switch (pkt_id)
  {
    case 0x01: comms_num_params(); break;
    case 0xf0: comms_discovery(p, len); break;
    case 0xf1: comms_read_flash(p, len); break;
    default: 
      printf("unhandled packet id: [%02x]\n", pkt_id);
      break;
  }
}

void comms_tx(const uint8_t *data, const uint32_t len)
{
  printf("comms_tx(%d)\r\n", (int)len);
  if (len > 250)
  {
    printf("woah! unable to handle packets > 250 bytes.\n");
    return;
  }
  const int framed_pkt_len = len + 6;
  uint8_t framed_pkt[framed_pkt_len];
  framed_pkt[0] = 0xbe;
  framed_pkt[1] = 0x51;
  framed_pkt[2] = g_comms_id;
  framed_pkt[3] = (uint8_t)len;
  for (uint32_t i = 0; i < len; i++)
    framed_pkt[i+4] = data[i];

  uint16_t crc = 0;
  for (uint32_t i = 0; i < framed_pkt_len - 2; i++)
  {
    // this is just a placeholder hack. Do a real CRC-16 someday.
    crc <<= 1;
    crc ^= framed_pkt[i];
  }
  framed_pkt[len+4] = (uint8_t)(crc & 0xff);
  framed_pkt[len+5] = (uint8_t)(crc >> 8);

  if (g_comms_raw_tx_fptr)
    g_comms_raw_tx_fptr(framed_pkt, len + 5);
  else
    printf("woah! no raw tx fptr set\n");
}

static void comms_tx_long_addr(const uint8_t *data, const uint32_t len)
{
  // printf("comms_tx_long_addr(%d)\r\n", (int)len);
  if (len > 240)
  {
    printf("woah! unable to handle packets > 240 bytes.\n");
    return;
  }
  const int framed_pkt_len = len + 17;
  uint8_t framed_pkt[framed_pkt_len];
  framed_pkt[0] = 0xbe;
  framed_pkt[1] = 0x55;
  memcpy(&framed_pkt[2], g_uuid, 12);
  framed_pkt[14] = (uint8_t)len;
  for (uint32_t i = 0; i < len; i++)
    framed_pkt[i+15] = data[i];

  const uint16_t crc = comms_crc(framed_pkt, framed_pkt_len - 2);
  framed_pkt[len+15] = (uint8_t)(crc & 0xff);
  framed_pkt[len+16] = (uint8_t)(crc >> 8);

  if (g_comms_raw_tx_fptr)
    g_comms_raw_tx_fptr(framed_pkt, len + 17);
  else
    printf("woah! no raw tx fptr set\n");
}
