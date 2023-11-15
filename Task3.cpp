#include "Microbit.h"
#include "nrf52833.h"

// #define _DEBUG

#define SERIAL_TX_PIN   6
#define CLOCK_SPEED     64000000
#define TARGET_BAUD     115200

void initGPIO()
{
    NRF_P0->DIRSET = (1UL << SERIAL_TX_PIN);
    NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
}

void delay(int i)
{
    volatile int delayCount = i;

    while (delayCount--);
}

void setTxPin(bool level)
{
    if (level)
        NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
    else
        NRF_P0->OUTCLR = (1UL << SERIAL_TX_PIN);
}

void sendByte(char byte, int k)
{
    setTxPin(false);
    delay(k);

    for (int i = 0; i < 8; i++) {
        setTxPin((byte >> i) & 0x01);
        delay(k);
    }

    setTxPin(true);
    delay(k);
}

void bitBangSerial(char *str, int i)
{
    while (*str)
        sendByte(*str++, i);
}

#ifdef _DEBUG

#define STRING_LITERAL "Testing"
#define STRING_SIZE sizeof(STRING_LITERAL)

int main() {
    char string[STRING_SIZE];
    strncpy(string, STRING_LITERAL, STRING_SIZE);

    initGPIO();

    // approx 75-81?
    bitBangSerial(string, 81);
}

#endif