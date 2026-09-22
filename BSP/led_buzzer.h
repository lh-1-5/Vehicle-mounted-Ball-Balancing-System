#ifndef LED_BUZZER_H
#define LED_BUZZER_H

/** Set both outputs to their inactive states: LED off and buzzer silent. */
void led_buzzer_init(void);

/** Turn on the LED (PB25 is driven low). */
void led_on(void);

/** Turn off the LED (PB25 is driven high). */
void led_off(void);

/** Turn on the buzzer (PB27 is driven high). */
void buzzer_on(void);

/** Turn off the buzzer (PB27 is driven low). */
void buzzer_off(void);

#endif /* LED_BUZZER_H */
