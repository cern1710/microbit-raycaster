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

void delay()
{
    volatile int delayCount = 75; // approx 75 to 81
    while (delayCount--);
}

void transmitAndDelay(bool level)
{
    if (level)
        NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
    else
        NRF_P0->OUTCLR = (1UL << SERIAL_TX_PIN);
    delay();
}

void sendByte(char byte)
{
    transmitAndDelay(false);

    for (int i = 0; i < 8; i++)
        transmitAndDelay((byte >> i) & 0x01);

    transmitAndDelay(true);
}

void bitBangSerial(char *str)
{
    while (*str)
        sendByte(*str++);
}

#ifdef _DEBUG

#define STRING_LITERAL "Testing"
#define STRING_SIZE sizeof(STRING_LITERAL)

int main() {
    char string[STRING_SIZE];
    strncpy(string, STRING_LITERAL, STRING_SIZE);

    initGPIO();
    bitBangSerial(string);
}

#endif