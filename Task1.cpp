#include "Microbit.h"

/* Flags for conditional compilation */
#define _DEBUG

/* Pin mapping from GPIO to edge connector pins */
#define P0    2
#define P1    3
#define P2    4
#define P8    10
#define P9    9
#define P11   23
#define P13   17
#define P14   1
#define P15   13

#define NUM_LEDS    8

/* GPIO addresses : base (0x50000000) + offset */
#define GPIO_OUT        0x50000504
#define GPIO_IN         0x50000510
#define GPIO_DIR        0x50000514

/* GPIOTE addresses: base (0x40000600) + offset */
#define GPIOTE_IN       0x40006100
#define GPIOTE_CONFIG   0x40006510

/* GPIOTE configuration offsets */
#define EVENT_MODE      1           // bit 0
#define PSEL_13         (P15 << 8)  // bits 8-11
#define FALLING_EDGE    (2 << 16)   // bits 16-17

#define GPIO_EVENT_DETECT   1
#define GPIO_EVENT_CLEAR    0

/* Timer addresses: base (0x40008000) + offset*/
#define TIMER_START             0x40008000
#define TIMER_CAPTURE           0x40008040
#define TIMER_EVENT_COMPARE     0x40008140
#define TIMER_BIT_MODE          0x40008508
#define TIMER_PRESCALAR         0x40008510
#define TIMER_COMPARE_REGISTER  0x40008540

/* Timer configuration offsets */
#define BIT_MODE    3   // 32-bits
#define PRESCALAR   8

/* Values used for milliseconds to ticks conversion */
#define BASE_FREQ_16M   16000000.0

/* Bit patterns that turns out the required pins for LEDs from 0 to 14 */
#define LED_BITS        0x82061E    // Or 8521246 in decimal

/* Bit shift function */
#define BIT_SHIFT(pin)  (uint32_t) (1 << pin)

/* Constant array storing LED masks */
const uint32_t LED_MASKS[] = {
    BIT_SHIFT(P14),
    BIT_SHIFT(P13),
    BIT_SHIFT(P11),
    BIT_SHIFT(P9),
    BIT_SHIFT(P8),
    BIT_SHIFT(P2),
    BIT_SHIFT(P1),
    BIT_SHIFT(P0)
};

/********************* Helper functions *********************/

/* Switches bits based on mask */
void switchBitsWithMask(volatile uint32_t mask)
{
    volatile uint32_t *out = (volatile uint32_t *) GPIO_OUT;
    volatile uint32_t *dir = (volatile uint32_t *) GPIO_DIR;

    /* Rewrite out and dir with mask value without OR */
    *out = mask;
    *dir = mask;
}

/* Conversion from milliseconds to number of ticks */
uint32_t msToTicks(float delay_ms) {
    float fTIMER = BASE_FREQ_16M / powf(2.0, (float)PRESCALAR);
    float ticks_per_ms = fTIMER / 1000.0;

    return (uint32_t)(ticks_per_ms * delay_ms);
}

/* Starts a 32-bit timer */
void startTimer()
{
    volatile uint32_t *start_timer = (volatile uint32_t *) TIMER_START;
    volatile uint32_t *bit_mode = (volatile uint32_t *) TIMER_BIT_MODE;
    volatile uint32_t *prescalar = (volatile uint32_t *) TIMER_PRESCALAR;

    *bit_mode = BIT_MODE;
    *prescalar = PRESCALAR;
    *start_timer = 1;
}

/* Gets the initial time by capturing timer counter */
uint32_t getTime()
{
    volatile uint32_t *current_time = (volatile uint32_t *) TIMER_COMPARE_REGISTER;
    volatile uint32_t *capture = (volatile uint32_t *) TIMER_CAPTURE;

    *capture = 1;
    *capture = 0;

    return *current_time;
}

/**
 * Delays execution until timer counter reaches specific absolute time
 *
 * Uses the timer module to delay execution until timer counter == next_time
 * This control mechanism is status driven, and uses the microcontroller to
 * poll for information. This prevents cumulative drift and is more precise.
 *
 * General usage of this function is as follows:
 *
 * begin:
 *   next_time := getTime() + interval
 *   loop:
 *      action;
 *      delay_until(next_time);
 *      next_time := next_time + interval;
 *   end loop;
 * end;
 */
void delayUntil(uint32_t next_time)
{
    volatile uint32_t *compare_register = (volatile uint32_t *) TIMER_COMPARE_REGISTER;
    volatile uint32_t *events_compare = (volatile uint32_t *) TIMER_EVENT_COMPARE;

    *compare_register = next_time;
    while (!(*events_compare));
    *events_compare = 0;
}

/* Delays execution based on arbitrary interval */
void delay(uint32_t interval)
{
    volatile int count = interval * 9000; // Rough number

    while (count--);
}

/********************* Task functions *********************/

void turnOn()
{
    switchBitsWithMask((uint32_t) LED_BITS);
}

void setLEDs(uint8_t value)
{
    uint8_t i;
    volatile uint32_t mask = 0;

    /* Switch on LED pin mask based on given value */
    for (i = 0; i < NUM_LEDS; i++) {
        if (value & BIT_SHIFT(i))
            mask |= LED_MASKS[i];
    }
    switchBitsWithMask(mask);
}

void rollingCounter()
{
    uint8_t i = 0;
    uint32_t next_time;
    uint32_t interval = msToTicks(117); // (30000 ms) / (256 ops)

    startTimer();
    next_time = getTime() + interval;
    for(;;) {
        setLEDs(i++);
        delayUntil(next_time);
        next_time += interval;
    }
}

void knightRider()
{
    int8_t direction = 1;
    uint8_t position = 0;
    uint32_t next_time;
    uint32_t interval = msToTicks(125);

    startTimer();
    next_time = getTime() + interval;
    for (;;) {
        setLEDs(1 << position);
        delayUntil(next_time);
        next_time += interval;

        /* Check boundary conditions and reverse if needed */
        if ((position == (NUM_LEDS - 1) && direction == 1) ||
                (position == 0 && direction == -1))
            direction = -direction;

        position += direction;
    }
}

void countClicks()
{
    uint8_t i = 0;
    uint8_t current_state = 0, last_state = 0;
    volatile uint32_t *events_in = (volatile uint32_t *) GPIOTE_IN;
    volatile uint32_t *task_config = (volatile uint32_t *) GPIOTE_CONFIG;

    /* Configure the task for falling edge detection on P13 */
    *task_config |= EVENT_MODE | PSEL_13 | FALLING_EDGE;

    for (;;) {
        current_state = *events_in;

        if (current_state == GPIO_EVENT_DETECT &&
                last_state == GPIO_EVENT_CLEAR) {
            setLEDs(++i);
            *events_in = GPIO_EVENT_CLEAR; // Clear input after detection
        }
        last_state = current_state;

        delay(10);  // Debouncing delay prevents spurious transitions
    }
}

#ifdef _DEBUG
#define TEST_UINT 81

int main()
{
    // turnOn();
    // setLEDs((uint8_t)TEST_UINT);
    // rollingCounter();
    // knightRider();
    countClicks();
}
#endif