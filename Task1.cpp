#include "Microbit.h"

#define _DEBUG

/* GPIO addresses : base (0x50000000) + offset */
#define GPIO_OUT        0x50000504
#define GPIO_IN         0x50000510
#define GPIO_DIR        0x50000514

#define GPIO_EVENT_DETECT   1
#define GPIO_EVENT_CLEAR    0

/* GPIOTE addresses: base (0x40000600) + offset */
#define GPIOTE_IN       0x40006100
#define GPIOTE_CONFIG   0x40006510

/* GPIOTE configuration offsets */
#define EVENT_MODE      1
#define PSEL_13         (P15 << 8)
#define FALLING_EDGE    (2 << 16)

/* Timer addresses: base (0x40008000) + offset*/
#define TIMER_START             0x40008000
#define TIMER_CAPTURE           0x40008040
#define TIMER_EVENT_COMPARE     0x40008140
#define TIMER_BIT_MODE          0x40008508
#define TIMER_PRESCALAR         0x40008510
#define TIMER_COMPARE_REGISTER  0x40008540

#define BASE_FREQ_16M   16000000.0
#define MS_PER_SECOND   1000.0

/* Timer configuration offsets */
#define MODE_32_BITS    3
#define PRESCALAR       8

/**
 * Rough approximation of number of loop iterations to approximate 1 ms
 *
 * 10666 ~= 64 MHz / (1000 ms/s * (2(ldr) + 1(sub) + 2(str) + 1(cbnz)))
 */
#define DELAY_1MS_ITERATIONS    10667

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

/* Bit patterns that turns on the required pins for LEDs from 0 to 14 */
#define LED_BITS        0x82061E

#define NUM_LEDS    8
#define BIT_SHIFT(pin)  (uint32_t) (1 << pin)

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

void switchBitsWithMask(volatile uint32_t mask)
{
    volatile uint32_t *out = (volatile uint32_t *) GPIO_OUT;
    volatile uint32_t *dir = (volatile uint32_t *) GPIO_DIR;

    *out = mask;
    *dir = mask;
}

uint32_t convertMsToTicks(float delay_ms)
{
    float fTIMER = BASE_FREQ_16M / ((float) BIT_SHIFT(PRESCALAR));
    float ticks_per_ms = fTIMER / MS_PER_SECOND;

    return (uint32_t)(ticks_per_ms * delay_ms);
}

void startTimer()
{
    volatile uint32_t *start_timer = (volatile uint32_t *) TIMER_START;
    volatile uint32_t *bit_mode = (volatile uint32_t *) TIMER_BIT_MODE;
    volatile uint32_t *prescalar = (volatile uint32_t *) TIMER_PRESCALAR;

    *bit_mode = MODE_32_BITS;
    *prescalar = PRESCALAR;
    *start_timer = 1;
}

uint32_t captureTime()
{
    volatile uint32_t *current_time = (volatile uint32_t *) TIMER_COMPARE_REGISTER;
    volatile uint32_t *capture = (volatile uint32_t *) TIMER_CAPTURE;

    *capture = 1;
    *capture = 0;   // Stops repeated captures

    return *current_time;
}

/**
 * Delays execution until timer reaches specified absolute time
 *
 * This status driven control mechanism prevents cumulative drift for higher
 * accuracy, with the loop delaying on average for the specified interval.
 */
void delayUntil(uint32_t next_time)
{
    volatile uint32_t *compare_register = (volatile uint32_t *) TIMER_COMPARE_REGISTER;
    volatile uint32_t *events_compare = (volatile uint32_t *) TIMER_EVENT_COMPARE;

    *compare_register = next_time;
    while (!(*events_compare));
    *events_compare = 0;
}

/* Software delay by looping for specified interval in milliseconds */
void delay(uint32_t delay_ms)
{
    volatile int count = delay_ms * DELAY_1MS_ITERATIONS;

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
    uint32_t mask = 0;

    /* Switch on LED pin mask based on given value */
    for (i = 0; i < NUM_LEDS; i++) {
        if (value & BIT_SHIFT(i))
            mask |= LED_MASKS[i];
    }
    switchBitsWithMask(mask);
}

void rollingCounter()
{
    uint8_t counter = 0;
    uint32_t next_time;
    const uint32_t interval = convertMsToTicks(117); // 117 = 30000 ms / 256 operations

    startTimer();
    next_time = captureTime() + interval;
    for (;;) {
        setLEDs(counter++);
        delayUntil(next_time);
        next_time += interval;
    }
}

void knightRider()
{
    uint32_t next_time;
    int8_t direction = 1, position = 0;
    const uint32_t interval = convertMsToTicks(125); // Takes approx. 1s to go through 8 LEDs

    startTimer();
    next_time = captureTime() + interval;
    for (;;) {
        setLEDs(1 << position);

        /* Check boundary conditions and reverse if needed */
        if ((position == (NUM_LEDS - 1) && direction == 1) ||
                (position == 0 && direction == -1))
            direction = -direction;

        position += direction;

        delayUntil(next_time);
        next_time += interval;
    }
}

void countClicks()
{
    uint8_t num_clicks = 0, current_state = 0, last_state = 0;
    volatile uint32_t *events_in = (volatile uint32_t *) GPIOTE_IN;
    volatile uint32_t *task_config = (volatile uint32_t *) GPIOTE_CONFIG;

    /* Configure the task for falling edge detection on P13 */
    *task_config |= EVENT_MODE | PSEL_13 | FALLING_EDGE;

    for (;;) {
        current_state = *events_in;

        if (current_state == GPIO_EVENT_DETECT &&
                last_state == GPIO_EVENT_CLEAR) {
            setLEDs(++num_clicks);
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
    rollingCounter();
    // knightRider();
    // countClicks();
}
#endif