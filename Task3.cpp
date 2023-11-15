#include "Microbit.h"
#include "nrf52833.h"
#include <cstdio>

#define _DEBUG

#define SERIAL_TX_PIN   6
#define CLOCK_SPEED     64000000
#define TARGET_BAUD     115200

#define ACCEL_I2C_ADDRESS   0x19

#define X_AXIS_REGISTER 0x28
#define Y_AXIS_REGISTER 0x2A
#define Z_AXIS_REGISTER 0x2C

#define SCL_PIN 26
#define SDA_PIN 27

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

void delay()
{
    volatile int delayCount = 75; // approx 75 to 81
    while (delayCount--);
}

void delayMicroseconds(uint32_t delay_ms)
{
    volatile int count = delay_ms * DELAY_1MS_ITERATIONS;
    while (count--);
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

int readAccelerometerAxis(uint8_t axisRegister)
{
    uint32_t result = 0;

    // Disable the TWI peripheral
    NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Disabled;

    // Configure SCL and SDA pin modes for I2C
    NRF_P0->PIN_CNF[SCL_PIN] = 0x00000600;  // Replace SCL_PIN with actual pin number
    NRF_P0->PIN_CNF[SDA_PIN] = 0x00000600;  // Replace SDA_PIN with actual pin number

    // Select the pin numbers associated with the internal I2C SCL and SDA lines
    NRF_TWI0->PSEL.SCL = SCL_PIN;  // Replace with actual SCL pin number
    NRF_TWI0->PSEL.SDA = SDA_PIN;  // Replace with actual SDA pin number

    // Set the I2C Address of the accelerometer and bus speed
    NRF_TWI0->ADDRESS = ACCEL_I2C_ADDRESS;   // Replace with the I2C address of the accelerometer
    NRF_TWI0->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K250;

    // Suspend the peripheral after every byte
    NRF_TWI0->SHORTS = 1;

    // Reset I2C flags
    resetI2CFlags();

    // Enable the TWI peripheral
    NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Enabled;

    // Address of the desired axis register inside the accelerometer
    NRF_TWI0->TXD = axisRegister;

    // Start an I2C write transaction
    NRF_TWI0->TASKS_STARTTX = 1;

    // Wait for data to be sent
    while (NRF_TWI0->EVENTS_TXDSENT == 0);
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

void bitBangSerial(char *str)
{
    while (*str)
        sendByte(*str++);
}

void showAccelerometerSample()
{
    char output[50];
    int x, y, z;

    while (1) {
        x = readAccelerometerAxis(X_AXIS_REGISTER);  // Replace with actual register address
        y = readAccelerometerAxis(Y_AXIS_REGISTER);  // Replace with actual register address
        z = readAccelerometerAxis(Z_AXIS_REGISTER);  // Replace with actual register address

        snprintf(output, sizeof(output), "[X: %d] [Y: %d] [Z: %d]\r\n", x, y, z);
        bitBangSerial(output);

        // Delay to update about five times per second
        for (int i = 0; i < 5; i++)
            delayMicroseconds(200000);  // 200 ms delay
    }
}

#ifdef _DEBUG

#define STRING_LITERAL "Testing"
#define STRING_SIZE sizeof(STRING_LITERAL)

int main() {
    char string[STRING_SIZE];
    strncpy(string, STRING_LITERAL, STRING_SIZE);

    initGPIO();
    bitBangSerial(string);
    // showAccelerometerSample();
}

#endif