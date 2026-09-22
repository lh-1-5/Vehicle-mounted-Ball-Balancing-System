/*
 * bsp.c - BSP 层初始化聚合
 * 把所有 BSP 驱动的 init 集中调, system_init 只调 bsp_init 一个.
 */
#include "bsp.h"
#include "uart.h"
#include "led_buzzer.h"
#include "oled.h"
#include "key.h"
#include "Encoder.h"
#include "Motor.h"
#include "step_motor.h"

void bsp_init(void)
{
    uart_camera_init();
    uart_communicate_init();
    led_buzzer_init();
    OLED_Init();
    key_init();
    encoder_init();
    motor_init();
    Step_Init();
    step_off();        /* 步进上电停转 */
    
}
