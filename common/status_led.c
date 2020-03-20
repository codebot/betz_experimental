#include "status_led.h"
#include "pin.h"
#include "stm32f405xx.h"

#define LED_GPIO GPIOB
#define LED_PIN  8

void status_led_init()
{
  pin_set_output(LED_GPIO, LED_PIN, 0);
}

void status_led_on()
{
  pin_set_output_high(LED_GPIO, LED_PIN);
}

void status_led_off()
{
  pin_set_output_low(LED_GPIO, LED_PIN);
}

void status_led_toggle()
{
  pin_toggle_state(LED_GPIO, LED_PIN);
}
