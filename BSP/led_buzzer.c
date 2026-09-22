#include "led_buzzer.h"
#include "ti_msp_dl_config.h"

void led_buzzer_init(void)
{
    led_off();
    buzzer_off();
}

void led_on(void)
{
    DL_GPIO_clearPins(GPIO_LED_BUZZER_PORT, GPIO_LED_BUZZER_LED_PIN);
}

void led_off(void)
{
    DL_GPIO_setPins(GPIO_LED_BUZZER_PORT, GPIO_LED_BUZZER_LED_PIN);
}

void buzzer_on(void)
{
    DL_GPIO_setPins(GPIO_LED_BUZZER_PORT, GPIO_LED_BUZZER_BUZZER_PIN);
}

void buzzer_off(void)
{
    DL_GPIO_clearPins(GPIO_LED_BUZZER_PORT, GPIO_LED_BUZZER_BUZZER_PIN);
}
