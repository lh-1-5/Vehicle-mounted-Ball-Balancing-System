#include "delay.h"

/* ms 延时: CPUCLK_FREQ/1000 = 每 ms 的周期数 (80MHz 下 = 80000) */
void delay_ms(uint32_t ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000u));
}

/* us 延时: CPUCLK_FREQ/1000000 = 每 us 的周期数 (80MHz 下 = 80) */
void delay_us(uint32_t us)
{
    delay_cycles(us * (CPUCLK_FREQ / 1000000u));
}
