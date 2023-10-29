#include "Microbit.h"
#include "nrf52833.h"

#define _DEBUG

#define HIGH(pin)  (uint32_t)(1 << pin)
#define LOW(pin)   (uint32_t)(1 << pin)

/**
 * Rough approximation of number of loop iterations to approximate 1 ms.
 *
 * 10667 ~= 64 MHz / (1000 ms/s * (2(ldr) + 1(sub) + 2(str) + 1(cbnz)))
 * Source: https://www.cse.scu.edu/~dlewis/book3/docs/ARM_Cortex-M4F_Instruction_Summary.pdf
 */
#define DELAY_1MS_ITERATIONS    10667

enum Rows {
    ROW1 = 21,
    ROW2 = 22,
    ROW3 = 15,
    ROW4 = 24,
    ROW5 = 19
};

enum Columns {
    COL1 = 28,
    COL2 = 11,
    COL3 = 31,
    COL4 = 5,   // NOTE: This is in P1 (srsly, why?)
    COL5 = 30
};

/********************* Helper functions *********************/

void setLED(Rows row, Columns col)
{
    if (col == COL4)
        NRF_P1->DIR |= LOW(col);
    else
        NRF_P0->DIR |= LOW(col);
    NRF_P0->OUT |= HIGH(row);
    NRF_P0->DIR |= HIGH(row);
}

void clearLEDs()
{
    NRF_P1->DIR = 0;
    NRF_P0->DIR = 0;
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

void beHappy()
{
    while(1) {
        setLED(ROW2, COL2);
        setLED(ROW2, COL4);
        delay(5);
        clearLEDs();

        setLED(ROW4, COL1);
        setLED(ROW4, COL5);
        delay(5);
        clearLEDs();

        setLED(ROW5, COL2);
        setLED(ROW5, COL3);
        setLED(ROW5, COL4);
        delay(5);
        clearLEDs();
    }
}

void beVeryHappy()
{

}

void beHappyAndFree()
{

}

void showNumber(int n)
{

}

#ifdef _DEBUG
#define TEST_UINT 81

int main()
{
    beHappy();
    // beVeryHappy();
    // beHappyAndFree();
    // showNumber(TEST_UINT);
}
#endif