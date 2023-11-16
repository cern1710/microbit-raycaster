#include "Microbit.h"
#include "nrf52833.h"

#define _DEBUG

#define SCL_PIN         8
#define SDA_PIN         16
#define SERIAL_TX_PIN   6

#define TWI_DISABLE         0
#define TWI_ENABLE          5
#define TWI_FREQ_250_KBPS   0x04000000

#define ACCEL_I2C_ADDRESS   0x19
#define ACCEL_CTRL_REG1     0x20
#define ACCEL_CTRL_REG4     0x23

#define CTRL_REG1_CONFIG    0x97    // HR 1.344 kHz, enable XYZ axis
#define CTRL_REG4_CONFIG    0x08    // Set to high-resolution mode (12-bit output)

#define X_LOW_AXIS_REGISTER  0x28
#define Y_LOW_AXIS_REGISTER  0x2A
#define Z_LOW_AXIS_REGISTER  0x2C
#define X_HIGH_AXIS_REGISTER 0x29
#define Y_HIGH_AXIS_REGISTER 0x2B
#define Z_HIGH_AXIS_REGISTER 0x2D

#define CONCAT_REGISTERS(high, low) ((int16_t)(((high) | (low)) >> 5))

/**
 * Rough approximation of number of loop iterations to approximate 1 ms.
 *
 * 10667 ~= 64 MHz / (1000 ms/s * (2(ldr) + 1(sub) + 2(str) + 1(cbnz)))
 * Source: https://www.cse.scu.edu/~dlewis/book3/docs/ARM_Cortex-M4F_Instruction_Summary.pdf
 */
#define DELAY_1MS_ITERATIONS    10667
#define DELAY_11520_BAUD        76

/* #####################################  Delay functions  ##################################### */

void delayMilliseconds(uint32_t delay_ms)
{
    volatile int count = delay_ms * DELAY_1MS_ITERATIONS;

    while (count--);
}

/* Busy-wait delay function in terms of iterations */
void delay(int delay_count)
{
    volatile int count = delay_count;

    while (count--);
}

/* ##############################  String manipulation functions  ############################## */

void myStrcpy(char *dst, const char *src)
{
    while (*src) {
        *dst = *src;
        dst++;
        src++;
    }
    *dst = '\0';
}

void myStrcat(char *dst, const char *src)
{
    while (*dst)
        dst++;
    while (*src) {
        *dst = *src;
        dst++;
        src++;
    }
    *dst = '\0';
}

/* String reversal function */
void reverse(char *str, int length)
{
    char temp;

    for (int start = 0, end = length - 1; start < end; start++, end--) {
        temp = str[start];
        str[start] = str[end];
        str[end] = temp;
    }
}

void intToStr(int num, char *str)
{
    int i = 0;
    bool isNegative = false;

    if (num == 0) {
        str[i++] = '0';
    } else {
        if (num < 0) {  // Make number positive for processing
            isNegative = true;
            num = -num;
        }
        for (; num > 0; num /= 10)  // Convert number to string
            str[i++] = (num % 10) + '0';

        if (isNegative)     // Append '-' to tail of string
            str[i++] = '-';
        reverse(str, i);
    }
    str[i] = '\0';  // Null-terminate the string
}

void formatOutput(int x, int y, int z, char* output)
{
    char numStr[6];

    intToStr(x, numStr);
    myStrcpy(output, "[X: ");
    myStrcat(output, numStr);
    myStrcat(output, "] ");

    intToStr(y, numStr);
    myStrcat(output, "[Y: ");
    myStrcat(output, numStr);
    myStrcat(output, "] ");

    intToStr(z, numStr);
    myStrcat(output, "[Z: ");
    myStrcat(output, numStr);
    myStrcat(output, "]\r\n");
}

/* #####################################  I2C functions  ####################################### */

void resetI2CFlags()
{
    NRF_TWI0->EVENTS_STOPPED    = 0;       // Clear any error flags.
    NRF_TWI0->EVENTS_RXDREADY   = 0;
    NRF_TWI0->EVENTS_TXDSENT    = 0;
    NRF_TWI0->EVENTS_ERROR      = 0;
    NRF_TWI0->EVENTS_BB         = 0;
    NRF_TWI0->EVENTS_SUSPENDED  = 0;
    NRF_TWI0->ERRORSRC          = 0xFFFFFFFF;
}

/* Sets transmission level and delays for specified baud rate (11520) */
void transmitLevel(bool level, int delay_count)
{
    if (level)
        NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
    else
        NRF_P0->OUTCLR = (1UL << SERIAL_TX_PIN);
    delay(delay_count);
}

/* Sends a byte over communication line with delay between each bit */
void sendByte(char byte, int delay_count)
{
    transmitLevel(false, delay_count);
    for (int i = 0; i < 8; i++)     // Transmits from least significant bit
        transmitLevel((byte >> i) & 0x01, delay_count);
    transmitLevel(true, delay_count);
}

/* ################################  Accelerometer functions  ################################## */

void initI2CTransaction(uint32_t register_value)
{
    NRF_TWI0->ENABLE = TWI_DISABLE;
    resetI2CFlags();
    NRF_TWI0->ENABLE = TWI_ENABLE;
    NRF_TWI0->TXD = register_value; // Set register to access
    NRF_TWI0->TASKS_STARTTX = 1;    // Start I2C Transaction
}

void waitEvent(volatile uint32_t* event)
{
    while (*event == 0);
    *event = 0;
}

void endI2CTransaction()
{
    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STOP = 1;
}

/* Reads a value from desired register in the accelerometer */
uint8_t readAccelerometerData(uint32_t read_register)
{
    uint8_t result = 0;

    delay(1);   // Prevents program from being stuck waiting for event
    initI2CTransaction(read_register);

    waitEvent(&NRF_TWI0->EVENTS_TXDSENT);
    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STARTRX = 1;

    waitEvent(&NRF_TWI0->EVENTS_RXDREADY);
    result = NRF_TWI0->RXD;
    endI2CTransaction();
    return result;
}

/* Writes a value to desired register in the accelerometer */
void writeAccelerometerData(uint32_t write_register, uint32_t byte)
{
    initI2CTransaction(write_register);

    waitEvent(&NRF_TWI0->EVENTS_TXDSENT);
    NRF_TWI0->TXD = byte;
    NRF_TWI0->TASKS_RESUME = 1;

    waitEvent(&NRF_TWI0->EVENTS_TXDSENT);
    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STARTRX = 1;

    waitEvent(&NRF_TWI0->EVENTS_RXDREADY);
    endI2CTransaction();
}

/* ################################  Configuration functions  ################################## */

void configureGPIO()
{
    NRF_P0->DIRSET = (1UL << SERIAL_TX_PIN);
    NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
}

void configureI2C()
{
    NRF_P0->PIN_CNF[SCL_PIN] = 0x00000600;
    NRF_P0->PIN_CNF[SDA_PIN] = 0x00000600;

    NRF_TWI0->PSEL.SCL = SCL_PIN;
    NRF_TWI0->PSEL.SDA = SDA_PIN;

    NRF_TWI0->ADDRESS = ACCEL_I2C_ADDRESS;
    NRF_TWI0->FREQUENCY = TWI_FREQ_250_KBPS;
    NRF_TWI0->SHORTS = 1;   // Suspend the peripheral after every byte
}

void configureAccelerometer()
{
    writeAccelerometerData(ACCEL_CTRL_REG1, 0x97);
    writeAccelerometerData(ACCEL_CTRL_REG4, 0x08);
}

/* #####################################  Task functions  ###################################### */

void bitBangSerial(char *str)
{
    configureGPIO();
    int delay_count = DELAY_11520_BAUD;

    while (*str)
        sendByte(*str++, delay_count);
}

void showAccelerometerSample()
{
    configureI2C();
    configureAccelerometer();

    char output[35];
    int16_t x, x_high, x_low;
    int16_t y, y_high, y_low;
    int16_t z, z_high, z_low;

    while (1) {
        x_high = readAccelerometerData(X_HIGH_AXIS_REGISTER) << 8;
        y_high = readAccelerometerData(Y_HIGH_AXIS_REGISTER) << 8;
        z_high = readAccelerometerData(Z_HIGH_AXIS_REGISTER) << 8;
        x_low = readAccelerometerData(X_LOW_AXIS_REGISTER);
        y_low = readAccelerometerData(Y_LOW_AXIS_REGISTER);
        z_low = readAccelerometerData(Z_LOW_AXIS_REGISTER);

        x = CONCAT_REGISTERS(x_high, x_low);
        y = CONCAT_REGISTERS(y_high, y_low);
        z = CONCAT_REGISTERS(z_high, z_low);

        formatOutput(x, y, z, output);

        bitBangSerial(output);
        delayMilliseconds(200);  // 1000ms/5 ~= 200ms
    }
}

#ifdef _DEBUG

#define STRING_LITERAL "findPatterns :: String -> [String] -> [[Int]]; findPatterns data' patterns = [ [idx | idx <- [1..length data' - length p + 1], p == take (length p) (drop (idx - 1) data')] | p <- patterns]"
#define SMALL_STRING "PHP my beloved..."
#define STRING_SIZE sizeof(STRING_LITERAL)
#define SMALL_SIZE sizeof(SMALL_STRING)

int main()
{
    char string[STRING_SIZE];
    strncpy(string, STRING_LITERAL, STRING_SIZE);

    char small[SMALL_SIZE];
    strncpy(small, SMALL_STRING, SMALL_SIZE);

    // bitBangSerial(small);
    showAccelerometerSample();
}

#endif