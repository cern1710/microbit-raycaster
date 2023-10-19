#include "Microbit.h"

// #define _DEBUG

#define MMIO32(addr)    (*(volatile uint32_t *)(addr))

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
#define MS_PER_SECOND   1000

/* Timer configuration offsets */
#define MODE_32_BITS    3
#define PRESCALAR       8

/**
 * Rough approximation of number of loop iterations to approximate 1 ms.
 *
 * 10667 ~= 64 MHz / (1000 ms/s * (2(ldr) + 1(sub) + 2(str) + 1(cbnz)))
 * Source: https://www.cse.scu.edu/~dlewis/book3/docs/ARM_Cortex-M4F_Instruction_Summary.pdf
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

void switchBitsWithMask(uint32_t mask)
{
    MMIO32(GPIO_OUT) = mask;
    MMIO32(GPIO_DIR) = mask;
}

uint32_t convertMsToTicks(float delay_ms)
{
    float fTIMER = BASE_FREQ_16M / BIT_SHIFT(PRESCALAR);
    float ticks_per_ms = fTIMER / MS_PER_SECOND;

    return (uint32_t)(ticks_per_ms * delay_ms);
}

void startTimer()
{
    MMIO32(TIMER_BIT_MODE) = MODE_32_BITS;
    MMIO32(TIMER_PRESCALAR) = PRESCALAR;
    MMIO32(TIMER_START) = 1;
}

uint32_t captureTime()
{
    MMIO32(TIMER_CAPTURE) = 1;
    MMIO32(TIMER_CAPTURE) = 0;   // Stops repeated captures

    return MMIO32(TIMER_COMPARE_REGISTER);
}

/**
 * Hardware-based delay waits until timer reaches specified absolute time.
 *
 * This status-driven control mechanism ensures higher accuracy by preventing
 * cumulative drift. The system pauses, on average, for the provided interval.
 */
void delayUntil(uint32_t next_time)
{
    MMIO32(TIMER_COMPARE_REGISTER) = next_time;

    while (!(MMIO32(TIMER_EVENT_COMPARE)));
    MMIO32(TIMER_EVENT_COMPARE) = 0;
}

/**
 * Software delay that pauses execution for a given interval in milliseconds.
 *
 * Utilizes a simple busy-wait loop. Due to local and cumulative drifts, the
 * actual (software) delay might be slightly longer than the specified interval.
 */
void delay(uint32_t delay_ms)
{
    volatile int count = delay_ms * DELAY_1MS_ITERATIONS;

    while (count--);
}

/********************* Task functions *********************/

void turnOn()
{
    switchBitsWithMask(LED_BITS);
}

void setLEDs(uint8_t value)
{
    uint8_t i;
    uint32_t mask = 0;

    /* Switch on LED pin mask based on given value */
    for (i = 0; i < NUM_LEDS; i++)
        mask |= (value & BIT_SHIFT(i)) ? LED_MASKS[i] : 0;

    switchBitsWithMask(mask);
}

void rollingCounter()
{
    uint8_t counter = 0;
    uint32_t next_time;
    const uint32_t interval = convertMsToTicks(117); // 30000 ms / 256 ops

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
    int8_t direction = -1, position = 0;
    const uint32_t interval = convertMsToTicks(125); // ~1s for 1 full scroll

    startTimer();
    next_time = captureTime() + interval;
    for (;;) {
        setLEDs(BIT_SHIFT(position));

        /* Check boundary conditions and reverse if mod(pos, 7) == 0 */
        if (!(position % (NUM_LEDS - 1)))
            direction = -direction;
        position += direction;

        delayUntil(next_time);
        next_time += interval;
    }
}

void countClicks()
{
    uint8_t num_clicks = 0;
    uint8_t current_state = 0, last_state = 0;

    MMIO32(GPIOTE_CONFIG) = EVENT_MODE | PSEL_13 | FALLING_EDGE;

    /* One empty loop after click event to prevent multiple detections. */
    for (;;) {
        current_state = MMIO32(GPIOTE_IN);

        if (current_state == GPIO_EVENT_DETECT &&
                last_state == GPIO_EVENT_CLEAR) {
            setLEDs(++num_clicks);
            MMIO32(GPIOTE_IN) = GPIO_EVENT_CLEAR; // Clear input after detection
        }
        last_state = current_state;

        /**
         * Debouncing delay prevents spurious transitions.
         * Source: https://my.eng.utah.edu/~cs5780/debouncing.pdf
         */
        delay(30);
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