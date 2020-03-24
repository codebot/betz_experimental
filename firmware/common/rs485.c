#include <stdio.h>
#include <string.h>
#include "param.h"
#include "parser.h"
#include "status_led.h"
#include "pin.h"
#include "rs485.h"
#include "stm32f405xx.h"

// pin connections
// PB10 = usart3 TX
// PB11 = usart3 RX

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

#define RS485_USART_GPIO GPIOB
#define RS485_USART USART1
#define TX_PIN 6
#define RX_PIN 7

#define RS485_DIR_GPIO GPIOD
#define RS485_DIR_PIN 2

#define RS485_RX_RING_LEN 512
static volatile uint8_t g_rs485_rx_ring[RS485_RX_RING_LEN];
static volatile uint32_t g_rs485_rx_ring_rpos = 0, g_rs485_rx_ring_wpos = 0;
uint8_t g_rs485_id = 42;

static void rs485_rx(const uint8_t *data, const uint32_t len);

void rs485_init()
{
  parser_init();
  parser_set_rx_pkt_fptr(rs485_rx);
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  pin_set_output(RS485_DIR_GPIO, RS485_DIR_PIN, 0);
  pin_set_alternate_function(RS485_USART_GPIO, TX_PIN, 7);  // AF7 = USART3 TX
  pin_set_alternate_function(RS485_USART_GPIO, RX_PIN, 7);  // AF7 = USART3 RX
  RS485_USART->CR1 &= ~USART_CR1_UE;
  // we want 1 megabit (for now):
  //   peripheral clock = 84 MHz. We want divisor = 84/16 = 5.25
  //   mantissa = 5, fraction (sixteenths) = 4
  RS485_USART->BRR = (5 << 4) | 4;
  RS485_USART->CR1 |=  USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  // for 1 Mbaud we want USARTDIV = 36M / 1M = 36
  RS485_USART->CR1 |= USART_CR1_UE;
  NVIC_SetPriority(USART1_IRQn, 1);
  NVIC_EnableIRQ(USART1_IRQn);
}

void rs485_tx(
    const uint8_t *data,
    const uint32_t len)
{
  //printf("rs485_tx(%d)\r\n", (int)len);
  if (len > 250)
  {
    printf("woah! unable to handle packets > 250 bytes.\n");
    return;
  }
  const int framed_pkt_len = len + 5;
  uint8_t framed_pkt[framed_pkt_len];
  framed_pkt[0] = 0xbe;
  framed_pkt[1] = 0x50;
  framed_pkt[2] = g_rs485_id;
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

  pin_set_output_high(RS485_DIR_GPIO, RS485_DIR_PIN);  // enable transmitter
  for (volatile int dumb = 0; dumb < 100; dumb++) { }
  for (uint32_t i = 0; i < len+5; i++)
  {
    while (!(RS485_USART->SR & USART_SR_TXE)) { } // wait for tx to clear
    RS485_USART->DR = framed_pkt[i];
  }
  while (!(RS485_USART->SR & USART_SR_TC)) { } // wait for TX to finish
  for (volatile int dumb = 0; dumb < 100; dumb++) { }
  pin_set_output_low(RS485_DIR_GPIO, RS485_DIR_PIN);  // disable transmitter
}

void usart1_vector()
{
  volatile uint8_t __attribute__((unused)) sr = USART1->SR;  // clear errors
  volatile uint8_t b = USART1->DR;  // drain
  g_rs485_rx_ring[g_rs485_rx_ring_wpos] = b;
  if (++g_rs485_rx_ring_wpos >= RS485_RX_RING_LEN)
    g_rs485_rx_ring_wpos = 0;
}

void rs485_tick()
{
  // called during the CPU's idle time
  while (g_rs485_rx_ring_rpos != g_rs485_rx_ring_wpos)
  {
    parser_process_byte(g_rs485_rx_ring[g_rs485_rx_ring_rpos]);
    if (++g_rs485_rx_ring_rpos >= RS485_RX_RING_LEN)
      g_rs485_rx_ring_rpos = 0;
  }
}

static void rs485_rx_req_num_params()
{
  //printf("rs485_rx_req_num_params()\n");
  const uint32_t num_params = param_count();
  uint8_t pkt[5] = {0};
  pkt[0] = 0x01;
  pkt[1] = num_params & 0xff;
  pkt[2] = (num_params >>  8) & 0xff;
  pkt[3] = (num_params >> 16) & 0xff;
  pkt[4] = (num_params >> 24) & 0xff;
  rs485_tx(pkt, sizeof(pkt));
}

static void rs485_read_flash(const uint8_t *data, const uint32_t len)
{
  if (len < 9)
    return;  // must have >= 9 bytes in request message
  uint32_t read_addr = 0, read_len = 0;
  memcpy(&read_addr, &data[1], sizeof(read_addr));
  memcpy(&read_len, &data[5], sizeof(read_len));
  printf("rs485_read_flash()\r\n");
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
  pkt[0] = 0xf0;
  memcpy(&pkt[1], &read_addr, sizeof(read_addr));
  memcpy(&pkt[5], &read_len, sizeof(read_len));
  memcpy(&pkt[9], (const uint8_t *)read_addr, read_len);
  rs485_tx(pkt, 9 + read_len);
}

void rs485_rx(const uint8_t *data, const uint32_t len)
{
  //printf("rs485_rx received %d bytes\r\n", (int)len);
  if (len == 0)
    return;  // adios amigo
  //for (uint32_t i = 0; i < len; i++)
  //  printf("%02d: %02x\r\n", (int)i, (unsigned)data[i]);
  const uint8_t pkt_id = data[0];
  switch (pkt_id)
  {
    case 0x01: rs485_rx_req_num_params(); break;
    case 0xf0: rs485_read_flash(data, len); break;
    default: break;
  }
}
