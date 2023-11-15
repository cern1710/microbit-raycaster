#include "Microbit.h"
#include "nrf52833.h"
#include <cstdio>

#define _DEBUG

#define SERIAL_TX_PIN   6
#define CLOCK_SPEED     64000000
#define TARGET_BAUD     115200

#define DELAY_11520_BAUD    78

#define ACCEL_I2C_ADDRESS   0x32

#define TWI_FREQ_250_KBPS  (0x04000000UL)

#define X_LOW_AXIS_REGISTER 0x28
#define Y_LOW_AXIS_REGISTER 0x2A
#define Z_LOW_AXIS_REGISTER 0x2C
#define X_HIGH_AXIS_REGISTER 0x29
#define Y_HIGH_AXIS_REGISTER 0x2B
#define Z_HIGH_AXIS_REGISTER 0x2D

#define SCL_PIN 26
#define SDA_PIN 27

void bitBangSerial(char *str, int delayCount);

/**
 * Rough approximation of number of loop iterations to approximate 1 ms.
 *
 * 10667 ~= 64 MHz / (1000 ms/s * (2(ldr) + 1(sub) + 2(str) + 1(cbnz)))
 * Source: https://www.cse.scu.edu/~dlewis/book3/docs/ARM_Cortex-M4F_Instruction_Summary.pdf
 */
#define DELAY_1MS_ITERATIONS    10667

void initGPIO()
{
    NRF_P0->DIRSET = (1UL << SERIAL_TX_PIN);
    NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
}

void resetI2CFlags()
{
    NRF_TWI0->EVENTS_STOPPED = 0;       // Clear any error flags.
    NRF_TWI0->EVENTS_RXDREADY = 0;
    NRF_TWI0->EVENTS_TXDSENT = 0;
    NRF_TWI0->EVENTS_ERROR = 0;
    NRF_TWI0->EVENTS_BB = 0;
    NRF_TWI0->EVENTS_SUSPENDED = 0;
    NRF_TWI0->ERRORSRC = 0xFFFFFFFF;
}

void delay(int delayCount)
{
    volatile int count = delayCount; // approx 75 to 81
    while (count--);
}

void delayMilliseconds(uint32_t delay_ms)
{
    volatile int count = delay_ms * DELAY_1MS_ITERATIONS;
    while (count--);
}

void transmitAndDelay(bool level, int delayCount)
{
    if (level)
        NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
    else
        NRF_P0->OUTCLR = (1UL << SERIAL_TX_PIN);
    delay(delayCount);
}

void sendByte(char byte, int delayCount)
{
    transmitAndDelay(false, delayCount);

    for (int i = 0; i < 8; i++)
        transmitAndDelay((byte >> i) & 0x01, delayCount);

    transmitAndDelay(true, delayCount);
}

int readAccelerometerAxis(uint8_t axisRegister)
{
    uint32_t result = 0;

    // Disable the TWI peripheral
    NRF_TWI0->ENABLE = 0;

    // Configure SCL and SDA pin modes for I2C
    NRF_P0->PIN_CNF[SCL_PIN] = 0x00000600;
    NRF_P0->PIN_CNF[SDA_PIN] = 0x00000600;

    // Select the pin numbers associated with the internal I2C SCL and SDA lines
    NRF_TWI0->PSEL.SCL = SCL_PIN;
    NRF_TWI0->PSEL.SDA = SDA_PIN;

    // Set the I2C Address of the accelerometer and bus speed
    NRF_TWI0->ADDRESS = ACCEL_I2C_ADDRESS;
    NRF_TWI0->FREQUENCY = TWI_FREQ_250_KBPS;

    // Suspend the peripheral after every byte
    NRF_TWI0->SHORTS = 1;

    // Reset I2C flags
    resetI2CFlags();

    // Enable the TWI peripheral
    NRF_TWI0->ENABLE = 1;

    // Address of the desired axis register inside the accelerometer
    NRF_TWI0->TXD = axisRegister;

    // Start an I2C write transaction
    NRF_TWI0->TASKS_STARTTX = 1;

    // Wait for data to be sent
    while (NRF_TWI0->EVENTS_TXDSENT == 0);  // this part
    NRF_TWI0->EVENTS_TXDSENT = 0;

    // Start receiving
    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STARTRX = 1;

    // Wait for data to be received
    while (NRF_TWI0->EVENTS_RXDREADY == 0);
    NRF_TWI0->EVENTS_RXDREADY = 0;

    // Store the byte that has been received
    result = NRF_TWI0->RXD;

    // Finish the transaction
    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STOP = 1;

    return result;
}

void bitBangSerial(char *str, int delayCount)
{
    while (*str)
        sendByte(*str++, delayCount);
}

void showAccelerometerSample()
{
    char output[50];
    int x = 0, y = 1, z = 10;

    while (1) {
        // x = readAccelerometerAxis(X_HIGH_AXIS_REGISTER) << 8 | readAccelerometerAxis(X_LOW_AXIS_REGISTER);
        // y = readAccelerometerAxis(Y_HIGH_AXIS_REGISTER) << 8 | readAccelerometerAxis(Y_LOW_AXIS_REGISTER);
        // z = readAccelerometerAxis(Z_HIGH_AXIS_REGISTER) << 8 | readAccelerometerAxis(Z_LOW_AXIS_REGISTER);
        snprintf(output, sizeof(output), "[X: %d] [Y: %d] [Z: %d]\r\n", x, y, z);
        bitBangSerial(output, DELAY_11520_BAUD);
        delayMilliseconds(200);  // 200 ms delay
        x++;
        y--;
        z *= 2;
    }
}

#ifdef _DEBUG

#define STRING_LITERAL "Testing"
#define STRING_SIZE sizeof(STRING_LITERAL)

int main() {
    char string[STRING_SIZE];
    strncpy(string, STRING_LITERAL, STRING_SIZE);

    initGPIO();
    // bitBangSerial(string, DELAY_11520_BAUD);
    showAccelerometerSample();
}

#endif