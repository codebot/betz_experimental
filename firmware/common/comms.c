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
#include "state.h"
#include "sys.h"
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
static int g_comms_id = 0;
static bool g_comms_is_bootloader = false;

static uint32_t g_comms_discovery_send_time = 0;
static bool g_comms_init_complete = false;

// local functions
static void comms_rx_pkt(
    const uint8_t *p,
    const uint32_t len,
    const bool long_address);

// static void comms_tx(const uint8_t *p, const uint32_t len);
static void comms_tx_long_addr(const uint8_t *p, const uint32_t len);

/////////////////////////////////////////////////////////////////////

void comms_init(void (*tx_fptr)(const uint8_t *, const uint32_t))
{
  g_comms_parser_state = PS_PREAMBLE;
  g_comms_raw_tx_fptr = tx_fptr;
  param_int("id", &g_comms_id, 0, PARAM_PERSISTENT);
}

void comms_tick()
{
  if (!g_comms_init_complete)
  {
    g_comms_init_complete = true;

    if (!g_comms_is_bootloader)
    {
      uint8_t pkt[1] = {1};
      pkt[0] = 0xf3;
      comms_tx_long_addr(pkt, 1);
    }
  }

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
        comms_rx_pkt(
            g_comms_parser_pkt,
            g_comms_parser_expected_length,
            g_comms_parser_long_address);
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
  // printf("comms_read_flash()\r\n");
  if (read_len > 128)
  {
    printf("invalid flash read: len = %d\r\n", (int)read_len);
    return;  // cannot. too long.
  }
  // sanity check to make sure the address range lies in flash
  if (read_addr < 0x08000000 || read_addr > 0x0807ffff)
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

void comms_write_flash(const uint8_t *data, const uint32_t len)
{
  if (!g_comms_is_bootloader)
    return;

  if (len < 9)
    return;  // must have >= 9 bytes in request message
  const uint32_t write_len = len - 5;
  uint32_t write_addr = 0;
  memcpy(&write_addr, &data[1], sizeof(write_addr));
  const uint8_t *write_data = &data[5];

  /*
  printf(
      "comms_write_flash(0x%08x, %u)\r\n",
      (unsigned)write_addr,
      (unsigned)write_len);
  */

  if (write_len > 128)
  {
    printf("invalid flash write: len = %d\r\n", (int)write_len);
    return;  // cannot. too long.
  }

  // sanity check to make sure the address range lies in flash
  if (write_addr < 0x08020000 || write_addr > 0x0807ffff)
  {
    printf("invalid flash write: addr = 0x%08x\r\n", (unsigned)write_addr);
    return;  // cannot. outside flash.
  }

  if (flash_write(write_addr, write_len, write_data))
  {
    uint8_t pkt[9] = {0};  // length of return request
    pkt[0] = 0xf2;
    memcpy(&pkt[1], &write_addr, sizeof(write_addr));
    memcpy(&pkt[5], &write_len, sizeof(write_len));
    comms_tx_long_addr(pkt, 9);
  }
}

void comms_discovery(const uint8_t *p, const uint32_t len)
{
  if (len >= 3)
  {
    const uint16_t req_max_delay = (uint16_t)(p[1]) | (p[2] << 8);
    uint32_t sampled_delay = rng_read() % (uint32_t)req_max_delay;
    g_comms_discovery_send_time = systime_read() + sampled_delay;
    // printf("discovery response in %u usec\n", (unsigned)sampled_delay);
    // for now, ignore the corner cases where this equals zero or wraparound
    // we'll query multiple times on the host to enumerate anyway.
  }
}

void comms_boot()
{
  if (!g_comms_is_bootloader)
    return;

  sys_run_application();  // this isn't really a function, more a JUMP
}

void comms_reset()
{
  sys_reset();  // buh bye
}

void comms_num_params()
{
  // printf("comms_num_params()\r\n");
  uint8_t pkt[5] = {0};
  pkt[0] = 0x01;
  const uint32_t num_params = param_count();
  memcpy(&pkt[1], &num_params, 4);
  comms_tx_long_addr(pkt, 5);
}

void comms_param_name_value(const uint8_t *rx_pkt, const uint32_t rx_len)
{
  if (rx_len < 5)
    return; // too short

  uint8_t tx_pkt[100] = {0};
  tx_pkt[0] = 0x02;
  uint32_t param_idx = 0;
  memcpy(&param_idx, &rx_pkt[1], sizeof(param_idx));
  if (param_idx >= param_count())
    return;  // invalid

  memcpy(&tx_pkt[1], &param_idx, sizeof(param_idx));

  tx_pkt[5] = (uint8_t)param_get_type(param_idx);
  tx_pkt[6] = (uint8_t)param_get_storage(param_idx);

  const int tx_name_len =
      strnlen(param_get_name(param_idx), sizeof(tx_pkt) - 12);
  tx_pkt[7] = tx_name_len;
  strncpy((char *)&tx_pkt[8], param_get_name(param_idx), tx_name_len);

  const int val_pos = 8 + tx_name_len;
  memcpy(&tx_pkt[val_pos], (void *)param_get_ptr(param_idx), 4);
  const int tx_len = val_pos + 4;

  comms_tx_long_addr(tx_pkt, tx_len);
}

void comms_param_set_value(const uint8_t *rx_pkt, const uint32_t len)
{
  if (len < 9)
    return; // too short, something must be wrong

  uint32_t param_idx = 0;
  memcpy(&param_idx, &rx_pkt[1], sizeof(param_idx));
  if (param_idx >= param_count())
    return;  // invalid
  
  volatile void *ptr = param_get_ptr(param_idx);
  if (!ptr)
    return;  // shouldn't be possible, but... let's always check anyway

  const param_type_t type = param_get_type(param_idx);
  if (type == PARAM_TYPE_INT)
  {
    int i = 0;
    memcpy(&i, &rx_pkt[5], 4);
    *(int *)ptr = i;
  }
  else if (type == PARAM_TYPE_FLOAT)
  {
    float f = 0;
    memcpy(&f, &rx_pkt[5], 4);
    *(float *)ptr = f;
  }
  
  // todo: someday send confirmation back, if requested?
}

void comms_state_poll(
    const uint8_t *p,
    const uint32_t len,
    const bool long_address)
{
  if (len < 2)
    return;  // too short

  // todo: remove this; just for testing... state will be filled in
  // by normal controller ticks via TIM1

  const uint8_t verbosity = p[1];

  if (verbosity == 1)
  {
    uint8_t tx_pkt[40] __attribute__((aligned(4))) = {0};
    tx_pkt[0] = 0x10;
    tx_pkt[1] = 0x00;  // padding, for future use by flags and stuff
    tx_pkt[2] = 0x00;  // padding, for future use by flags and stuff
    tx_pkt[3] = 0x00;  // padding, for future use by flags and stuff
  
    // Now that we're sure to be 32-bit aligned, we can just copy stuff in.
    // To avoid some compiler warnings about breaking aliasing rules, we'll
    // do it in 2 steps. We know this is safe, and this is a performance
    // critical block, so avoiding the memcpy() calls is important.

    uint32_t *t_dest = (uint32_t *)&tx_pkt[4];
    *t_dest = g_state.t;

    float *enc_dest = (float *)&tx_pkt[8];
    *enc_dest = g_state.enc;

    // todo: match the poll for short/long address
    comms_tx_long_addr(tx_pkt, 12);
  }
  else
  {
    printf("unhandled state verbosity level: %d!\r\n", (int)verbosity);
  }
}

void comms_rx_pkt(
    const uint8_t *p,
    const uint32_t len,
    const bool long_address)
{
  //printf("comms_rx_pkt() received %d bytes\r\n", (int)len);
  if (len == 0)
    return;  // adios amigo
  //for (uint32_t i = 0; i < len; i++)
  //  printf("%02d: %02x\r\n", (int)i, (unsigned)data[i]);
  const uint8_t pkt_id = p[0];
  switch (pkt_id)
  {
    case 0x01: comms_num_params(); break;
    case 0x02: comms_param_name_value(p, len); break;
    case 0x03: comms_param_set_value(p, len); break;
    case 0x10: comms_state_poll(p, len, long_address); break;
    case 0xf0: comms_discovery(p, len); break;
    case 0xf1: comms_read_flash(p, len); break;
    case 0xf2: comms_write_flash(p, len); break;
    case 0xf3: comms_boot(); break;
    case 0xf4: comms_reset(); break;
    default: 
      printf("unhandled packet id: 0x%02x\n", pkt_id);
      break;
  }
}

#if 0
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
#endif

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
    printf("woah! no raw tx fptr set\r\n");
}

void comms_set_bootloader_mode(const bool is_bootloader)
{
  g_comms_is_bootloader = is_bootloader;
}
