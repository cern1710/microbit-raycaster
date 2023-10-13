#include "Microbit.h"

/* Flags for conditional compilation */
#define _DEBUG
// #define USE_INLINE_ASM

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

#ifndef USE_INLINE_ASM
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

void startTimer()
{
    volatile uint32_t *timer = (volatile uint32_t *) 0x40008000;
    volatile uint32_t *bit_mode = (volatile uint32_t *) 0x40008508;

    *bit_mode = 2;
    *timer |= 1;
}

uint32_t getTime()
{
    return *((volatile uint32_t *)0x40008540);
}

void delay_until(int ms)
{
    // 1. set the capture value
    // 2. start the loop
    // 3. check the compare value in a loop
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
extern const uint32_t LED_MASKS[] = {
    BIT_SHIFT(P14),
    BIT_SHIFT(P13),
    BIT_SHIFT(P11),
    BIT_SHIFT(P9),
    BIT_SHIFT(P8),
    BIT_SHIFT(P2),
    BIT_SHIFT(P1),
    BIT_SHIFT(P0)
};

extern "C" {
    void setLEDs(uint8_t value);
    void delay(int ms);
}

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
        /* While loop checking if r3 > 0; bne to 1*/
        "mov r2, #0 \n\t"
        "ldr r3, = 9000 \n\t"
        "mul r2, %[_ms], r3 \n\t"
        "1: subs r2, r2, #1 \n\t"
        "bne 1b"
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
    volatile uint32_t mask = 0;

    __asm__ volatile(
        "movs r2, #0 \n\t"
        "ldr r3, [%[value_ptr]] \n\t"
        "ldr r4, =LED_MASKS \n\t"

        /* Bit shift r5 and update mask */
        "1: \n\t"
        "mov r5, #1 \n\t"
        "lsl r5, r5, r2 \n\t"           // r5 = 2^(i)
        "tst r3, r5 \n\t"               // Test if bit is set in 'value'
        "beq 2f \n\t"                   // If not, skip to "2"
        "ldr r6, [r4, r2, lsl #2] \n\t" // Load ith mask from array
        "orr %[mask], %[mask], r6 \n\t" // Update the mask with bitwise or

        /* Compare ++i to NUM_LEDs; loop to 1 if (i < NUM_LEDS)*/
        "2: \n\t"
        "adds r2, r2, #1 \n\t"
        "cmp r2, %[num_leds] \n\t"
        "blt 1b \n\t"

        : [mask]"=r"(mask)
        : [value_ptr]"r"(&value), [num_leds]"I"(NUM_LEDS)
        : "cc", "r2", "r3", "r4", "r5", "r6"
    );

    switchBitsWithMask(mask);
}

void rollingCounter() {
    __asm__ volatile(
        "movs r0, #0 \n\t"    // Initialize i = 0

        "1: \n\t"
        "bl setLEDs \n\t"     // Call setLEDs(i)
        "adds r0, r0, #1 \n\t"// i++
        "push {r0} \n\t"      // Save r0 (i)
        "ldr r1, =120 \n\t"   // Load delay value
        "bl delay \n\t"       // Call delay(120)
        "pop {r0} \n\t"       // Restore r0 (i)
        "b 1b"                // Infinite loop
    );
}

void knightRider() {
    __asm__ volatile(
        "movs r0, #1 \n\t"     // direction = 1
        "movs r1, #0 \n\t"     // position = 0

        "1: \n\t"
        "lsls r2, r1, #1 \n\t" // r2 = 1 << position
        "bl setLEDs \n\t"      // Call setLEDs(r2)
        "ldr r3, =100 \n\t"    // Load delay value
        "bl delay \n\t"        // Call delay(100)
        "cmp r1, #7 \n\t"      // Check for position == NUM_LEDS - 1
        "beq 2f \n\t"
        "cmp r1, #0 \n\t"      // Check for position == 0
        "bne 3f \n\t"
        "2: \n\t"
        "negs r0, r0 \n\t"     // direction = -direction
        "3: \n\t"
        "adds r1, r1, r0 \n\t" // position += direction
        "b 1b"                 // Infinite loop
    );
}

void countClicks() {
    __asm__ volatile(
        /* Load constants into registers */
        "ldr r6, =GPIOTE_IN \n\t"
        "ldr r7, =GPIOTE_CONFIG \n\t"
        "ldr r8, =EVENT_MODE \n\t"
        "ldr r9, =PSEL_13 \n\t"
        "ldr r10, =FALLING_EDGE \n\t"
        "ldr r11, =GPIO_EVENT_DETECT \n\t"
        "ldr r12, =GPIO_EVENT_CLEAR \n\t"

        "movs r0, #0 \n\t"  // i = 0
        "movs r1, #0 \n\t"  // current_state = 0
        "movs r2, #0 \n\t"  // last_state = 0

        "ldr r5, [r7] \n\t" // Load from GPIOTE_CONFIG to r5
        "orr r5, r8 \n\t"   // OR with EVENT_MODE
        "orr r5, r9 \n\t"   // OR with PSEL_13
        "orr r5, r10 \n\t"  // OR with FALLING_EDGE
        "str r5, [r7] \n\t" // Store the result to GPIOTE_CONFIG

        "1: \n\t"
        "ldr r1, [r6] \n\t" // current_state = *events_in
        "cmp r1, r11 \n\t"  // Compare with GPIO_EVENT_DETECT
        "bne 2f \n\t"
        "cmp r2, r12 \n\t"  // Compare with GPIO_EVENT_CLEAR
        "bne 2f \n\t"
        "adds r0, r0, #1 \n\t"  // i++
        "bl setLEDs \n\t"   // Call setLEDs(i)
        "str r12, [r6] \n\t"    // *events_in = GPIO_EVENT_CLEAR

        "2: \n\t"
        "mov r2, r1 \n\t"   // last_state = current_state
        "movs r5, #10 \n\t"
        "bl delay \n\t"     // Call delay(10)
        "b 1b"              // Infinite loop
    );
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