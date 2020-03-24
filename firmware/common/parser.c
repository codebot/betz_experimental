#include <stdio.h>
#include <stdbool.h>
#include "parser.h"
#include "rs485.h"
#include "uuid.h"

#define PARSER_PKT_MAX_LEN 256
static parser_state_t g_parser_state = PS_PREAMBLE;
static uint32_t g_parser_expected_length = 0;
static uint32_t g_parser_wpos = 0;
static uint8_t  g_parser_pkt[PARSER_PKT_MAX_LEN] = {0};
static uint16_t g_parser_csum = 0, g_parser_rx_csum = 0;
static void (*g_parser_rx_pkt_fptr)(const uint8_t *, const uint32_t) = 0;
static bool g_parser_broadcast = false;
static bool g_parser_long_address = false;
static bool g_parser_ignore_pkt = false;
static uint8_t g_parser_addr[RS485_LONG_ADDR_LEN] = {0};

void parser_init()
{
  g_parser_state = PS_PREAMBLE;
}

void parser_set_rx_pkt_fptr(void (*fptr)(const uint8_t *, const uint32_t))
{
  g_parser_rx_pkt_fptr = fptr;
}

static void parser_csum_byte(const uint8_t b)
{
  // placeholder for now. replace with real CRC-16 (usb csum) sometime
  g_parser_csum <<= 1;
  g_parser_csum ^= b;
}

void parser_process_byte(const uint8_t b)
{
  // printf("rx 0x%02x state %d\n", (unsigned)b, (int)g_parser_state);
  switch (g_parser_state)
  {
    case PS_PREAMBLE:
      if (b == 0xbe)
      {
        g_parser_csum = 0;
        parser_csum_byte(b);
        g_parser_state = PS_FLAGS;
      }
      break;

    case PS_FLAGS:
      parser_csum_byte(b);
      // verify the top 4 bits are as expected
      if ((b & 0xf0) != 0x50)
      {
        g_parser_state = PS_PREAMBLE;
        break;
      }
      // if the peripheral->host bit is set, the packet is not intended for us
      g_parser_ignore_pkt = (b & 0x1);
      g_parser_broadcast = (b & 0x2);
      g_parser_long_address = (b & 0x4);
      g_parser_wpos = 0;
      g_parser_state = PS_ADDRESS;
      break;

    case PS_ADDRESS:
      parser_csum_byte(b);
      if (g_parser_long_address)
      {
        g_parser_addr[g_parser_wpos] = b;
        g_parser_wpos++;
        if (g_parser_wpos == RS485_LONG_ADDR_LEN)
        {
          if ((g_uuid[0] != *((uint32_t *)&g_parser_addr[0])) ||
              (g_uuid[1] != *((uint32_t *)&g_parser_addr[4])) ||
              (g_uuid[2] != *((uint32_t *)&g_parser_addr[8])))
            g_parser_ignore_pkt = true;
          g_parser_state = PS_LENGTH;
        }
      }
      else
      {
        if (b != g_rs485_id)
          g_parser_ignore_pkt = true;
        g_parser_state = PS_LENGTH;
      }
      break;

    case PS_LENGTH:
      parser_csum_byte(b);
      if (b == 0)
      {
        g_parser_state = PS_PREAMBLE;  // this shouldn't happen...
        break;
      }

      g_parser_expected_length = b;
      g_parser_wpos = 0;
      g_parser_state = PS_PAYLOAD;
      break;

    case PS_PAYLOAD:
      parser_csum_byte(b);
      g_parser_pkt[g_parser_wpos] = b;
      // printf("  payload %d/%d\n",
      //     g_parser_wpos, g_parser_expected_length);
      if (g_parser_wpos == g_parser_expected_length - 1)
        g_parser_state = PS_CSUM_0;
      if (g_parser_wpos < PARSER_PKT_MAX_LEN - 1)
        g_parser_wpos++;
      break;

    case PS_CSUM_0:
      g_parser_state = PS_CSUM_1;
      g_parser_rx_csum = b;
      break;

    case PS_CSUM_1:
      g_parser_state = PS_PREAMBLE;
      g_parser_rx_csum |= (b << 8);
      if (g_parser_rx_csum == g_parser_csum)
      {
        // printf("  pkt csum ok\n");
        if (g_parser_ignore_pkt)
        {
          printf("ignoring packet\r\n");
          break;
        }
        if (g_parser_rx_pkt_fptr)
          g_parser_rx_pkt_fptr(g_parser_pkt, g_parser_expected_length);
      }
      else
      {
        printf("csum fail: 0x%0x != 0x%x\n",
            g_parser_csum,
            g_parser_rx_csum);
      }
      break;
    default:
      g_parser_state = PS_PREAMBLE;
  }
}
