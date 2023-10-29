#include "Microbit.h"
#include "nrf52833.h"

#define _DEBUG

#define BIT_SHIFT(pin)  (uint32_t)(1 << pin)

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

void setLED(Rows row, Columns col) {
    if (col == COL4)
        NRF_P1->OUT = BIT_SHIFT(col);
    else
        NRF_P0->OUT = BIT_SHIFT(col);
    NRF_P0->OUT = BIT_SHIFT(row);
}

void clearLED(Rows row, Columns col) {
    if (col == COL4)
        NRF_P1->OUTCLR = BIT_SHIFT(col);
    else
        NRF_P0->OUTCLR = BIT_SHIFT(col);
    NRF_P0->OUTCLR = BIT_SHIFT(row);
}

/********************* Task functions *********************/

void beHappy()
{

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
    // beHappy();
    // beVeryHappy();
    // beHappyAndFree();
    // showNumber(TEST_UINT);
}
#endif