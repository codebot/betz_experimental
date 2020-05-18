#include "status_led.h"
#include "pin.h"

#if defined(BOARD_blue)
#include "stm32f405xx.h"
#define LED_GPIO GPIOB
#define LED_PIN  8
#elif defined(BOARD_mini)
#include "stm32g474xx.h"
// TODO: NOT THIS
#define LED_GPIO GPIOA
#define LED_PIN 0
#else
#error AHHHH no board in status_led.c
#endif

void status_led_init()
{
  pin_set_output(LED_GPIO, LED_PIN, 1);
}

void status_led_on()
{
  pin_set_output_low(LED_GPIO, LED_PIN);
}

void status_led_off()
{
  pin_set_output_high(LED_GPIO, LED_PIN);
}

void status_led_toggle()
{
  pin_toggle_state(LED_GPIO, LED_PIN);
}
