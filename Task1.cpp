#include "Microbit.h"

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

/* GPIO addresses : base (0x50000000) + offset */
#define GPIO_OUT        0x50000504
#define GPIO_IN         0x50000510
#define GPIO_DIR        0x50000514

/* GPIOTE addresses: base (0x40000600) + offset */
#define GPIOTE_IN       0x40006100
#define GPIOTE_CONFIG   0x40006510

/* GPIOTE config offsets */
#define EVENT_MODE      1
#define PSEL_13         (P15 << 8)
#define FALLING_EDGE    (2 << 16)

#define GPIO_EVENT_DETECT   1
#define GPIO_EVENT_CLEAR    0

/* Bit patterns that turns out the required pins for LEDs from 0 to 14 */
#define LED_BITS        0x82061E    // or 8521246 in dec

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

    /* rewrite out and dir with mask value without OR */
    *out = mask;
    *dir = mask;
}

/* Delays execution based on milliseconds*/
void delay(int ms)
{
    volatile int count = ms * 9000; // rough number based on experiments

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

    /* switch on LED pin mask based on given value */
    for (i = 0; i < 8; i++) {
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
        if ((position == 7 && direction == 1) ||
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