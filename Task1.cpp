#include "Microbit.h"

#define _DEBUG
#define USE_INLINE_ASM

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

#define NUM_LEDS    8   // Number of LEDs

/* GPIO addresses : base (0x50000000) + offset */
#define GPIO_OUT        0x50000504
#define GPIO_IN         0x50000510
#define GPIO_DIR        0x50000514

/* GPIOTE addresses: base (0x40000600) + offset */
#define GPIOTE_IN       0x40006100
#define GPIOTE_CONFIG   0x40006510

/* GPIOTE config offsets */
#define EVENT_MODE      1           // bit 0
#define PSEL_13         (P15 << 8)  // bits 8-11
#define FALLING_EDGE    (2 << 16)   // bits 16-17

#define GPIO_EVENT_DETECT   1
#define GPIO_EVENT_CLEAR    0

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

#ifndef USE_INLINE_ASM
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

/* Delays execution based on milliseconds*/
void delay(int ms)
{
    volatile int count = ms * 9000; // Rough number based on experiments

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

    for(;;) {
        setLEDs(i++);
        delay(120); // Approx. delay of 118ms for 256 ops.
    }
}

void knightRider()
{
    int8_t direction = 1;
    uint8_t position = 0;

    for (;;) {
        setLEDs(1 << position);
        delay(100);

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
    uint32_t current_state = 0, last_state = 0;
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

#else
/********************* Helper functions *********************/

/* Switches bits based on mask */
void switchBitsWithMask(volatile uint32_t mask)
{
    __asm__ volatile(
        "str %[mask_val], [%[out]] \n\t"    // Store mask value to GPIO_OUT
        "str %[mask_val], [%[dir]]"         // Store mask value to GPIO_DIR
        : /* No outputs */
        : [mask_val]"r" (mask), [out]"r" (GPIO_OUT), [dir]"r" (GPIO_DIR)
    );
}

/* Delays execution based on milliseconds*/
void delay(int ms)
{
    __asm__ volatile(
        "mov r2, #0 \n\t"           // Initialize r2 to 0
        "ldr r3, = 9000 \n\t"       // Load 9000 to r3
        "mul r2, %[_ms], r3 \n\t"   // Multiply ms by 9000 to get count
        "1: subs r2, r2, #1 \n\t"   // Subtract 1 from count
        "bne 1b"                    // If count is not 0, branch to 1
        :
        : [_ms]"r" (ms)
        : "cc", "r2", "r3"
    );
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

    /* switch on LED pin mask based on given value */
    for (i = 0; i < NUM_LEDS; i++) {
        if (value & BIT_SHIFT(i))
            mask |= LED_MASKS[i];
    }
    switchBitsWithMask(mask);
}

void rollingCounter()
{
    uint8_t i = 0;

    for(;;) {
        setLEDs(i++);
        delay(120); // approx. delay of 118ms for 256 ops.
    }
}

void knightRider()
{
    int8_t direction = 1;
    uint8_t position = 0;

    for (;;) {
        setLEDs(1 << position);
        delay(100);

        /* check boundary conditions and reverse if needed */
        if ((position == (NUM_LEDS - 1) && direction == 1) ||
                (position == 0 && direction == -1))
            direction = -direction;

        position += direction;
    }
}

void countClicks()
{
    uint8_t i = 0;
    uint32_t current_state = 0, last_state = 0;
    volatile uint32_t *events_in = (volatile uint32_t *) GPIOTE_IN;
    volatile uint32_t *task_config = (volatile uint32_t *) GPIOTE_CONFIG;

    /* Configure the task for falling edge detection on P13 */
    *task_config |= EVENT_MODE | PSEL_13 | FALLING_EDGE;

    for (;;) {
        current_state = *events_in;

        if (current_state == GPIO_EVENT_DETECT &&
                last_state == GPIO_EVENT_CLEAR) {
            setLEDs(++i);
            *events_in = GPIO_EVENT_CLEAR; // clear input after detection
        }
        last_state = current_state;

        delay(10);  // debouncing delay prevents spurious transitions
    }
}
#endif

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