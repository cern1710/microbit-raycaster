#include "Microbit.h"

/**
 * Flag used for debugging
 * Uncomment this to run int main() in this file
 */
#define _DEBUG

/* GPIO addresses : base (0x5000000) + offset */
#define GPIO_OUT    0x50000504
#define GPIO_IN     0x50000510
#define GPIO_DIR    0x50000514

/* Bit patterns that turns out the required pins for LEDs from 0 to 14 */
#define LED_BITS    0x82061E    // or 8521246 in dec

/* Bit shift function */
#define BIT_SHIFT(pin)          (uint32_t) (1 << pin)

/* Gets the k-th bit of n*/
#define GET_BIT_IN_POS(n, k)    ((n & (1 << k)) >> k)

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
    volatile uint32_t *dir= (volatile uint32_t *) GPIO_DIR;

    *out = mask;
    *dir = mask;
}

/* Delays execution based on milliseconds*/
void delay(int ms) {
    volatile int count = ms * 9000; // rough number based on experiments

    while(count--);
}

/********************* Task functions *********************/

void turnOn()
{
    switchBitsWithMask((uint32_t) LED_BITS);
}

void setLEDs(uint8_t value) {
    uint8_t i;
    volatile uint32_t mask = 0;

    // switch on LED pin mask based on given value
    for (i = 0; i < 8; i++) {
        if (value & (1 << i))
            mask |= LED_MASKS[i];
    }

    switchBitsWithMask(mask);
}

void rollingCounter()
{
    uint8_t i = 0;

    for(;;) {
        setLEDs(i++);
        delay(118); // approx. delay of 118ms for 256 ops.
    }
}

void knightRider()
{
    int8_t direction = 1;
    uint8_t position = 0;

    for (;;) {
        setLEDs(1 << position);
        delay(100);

        // check boundary conditions and reverse if needed
        if ((position == 7 && direction == 1) ||
                (position == 0 && direction == -1))
            direction = -direction;

        position += direction;
    }
}

void countClicks()
{
    volatile uint32_t *addr = (volatile uint32_t *) 0x5000734;
    *addr |= 3 << 2 | 3 << 16;
    int8_t i = 0;
    uint8_t current_state = 0, last_state = 0;
    volatile uint32_t *gpio_in = (volatile uint32_t *) GPIO_IN;

    for (;;) {
        current_state = GET_BIT_IN_POS(*gpio_in, 13);

        if (current_state == 0 && last_state == 1)
            setLEDs(++i);

        // debouncing
        delay(10);
        last_state = current_state;
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
