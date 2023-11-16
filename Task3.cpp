#include "Microbit.h"
#include "nrf52833.h"

#define _DEBUG

#define SCL_PIN 8
#define SDA_PIN 16

#define SERIAL_TX_PIN   6

#define TWI_FREQ_250_KBPS  0x04000000

#define ACCEL_I2C_ADDRESS   0x19

#define X_LOW_AXIS_REGISTER 0x28
#define Y_LOW_AXIS_REGISTER 0x2A
#define Z_LOW_AXIS_REGISTER 0x2C
#define X_HIGH_AXIS_REGISTER 0x29
#define Y_HIGH_AXIS_REGISTER 0x2B
#define Z_HIGH_AXIS_REGISTER 0x2D

#define TWI_DISABLE     0
#define TWI_ENABLE      5

/**
 * Rough approximation of number of loop iterations to approximate 1 ms.
 *
 * 10667 ~= 64 MHz / (1000 ms/s * (2(ldr) + 1(sub) + 2(str) + 1(cbnz)))
 * Source: https://www.cse.scu.edu/~dlewis/book3/docs/ARM_Cortex-M4F_Instruction_Summary.pdf
 */
#define DELAY_1MS_ITERATIONS    10667
#define DELAY_11520_BAUD        78

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

void delay(int delay_count)
{
    volatile int count = delay_count; // approx 75 to 81

    while (count--);
}

void delayMilliseconds(uint32_t delay_ms)
{
    volatile int count = delay_ms * DELAY_1MS_ITERATIONS;

    while (count--);
}

void transmitAndDelay(bool level, int delay_count)
{
    if (level)
        NRF_P0->OUTSET = (1UL << SERIAL_TX_PIN);
    else
        NRF_P0->OUTCLR = (1UL << SERIAL_TX_PIN);
    delay(delay_count);
}

/* ####################################  String manipulation functions  #################################### */

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
        if (num < 0) {
            isNegative = true;
            num = -num; // Make the number positive for processing
        }

        // Convert number to string
        for (; num > 0; num /= 10)
            str[i++] = (num % 10) + '0';

        if (isNegative)
            str[i++] = '-';
        reverse(str, i);
    }
    str[i] = '\0'; // Null-terminate the string
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

uint8_t readAccelerometerData(uint32_t read_register)
{
    uint8_t result = 0;

    delay(1);

    NRF_TWI0->ENABLE = TWI_DISABLE;
    resetI2CFlags();

    NRF_TWI0->ENABLE = TWI_ENABLE;
    NRF_TWI0->TXD = read_register;  // Desired axis register in accelerometer
    NRF_TWI0->TASKS_STARTTX = 1;    // Start an I2C write transaction

    // Wait for data to be sent
    while (NRF_TWI0->EVENTS_TXDSENT == 0);
    NRF_TWI0->EVENTS_TXDSENT = 0;

    // Start receiving
    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STARTRX = 1;

    // Wait for data to be received
    while (NRF_TWI0->EVENTS_RXDREADY == 0);
    NRF_TWI0->EVENTS_RXDREADY = 0;

    result = NRF_TWI0->RXD;     // Store the byte that has been received

    // Finish the transaction
    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STOP = 1;

    return result;
}

void writeAccelerometerData(uint32_t write_register, uint32_t byte)
{
    NRF_TWI0->ENABLE = TWI_DISABLE;
    resetI2CFlags();

    NRF_TWI0->ENABLE = TWI_ENABLE;
    NRF_TWI0->TXD = write_register; // Desired axis register in accelerometer
    NRF_TWI0->TASKS_STARTTX = 1;    // Start an I2C write transaction

    while (NRF_TWI0->EVENTS_TXDSENT == 0);
    NRF_TWI0->EVENTS_TXDSENT = 0;

    NRF_TWI0->TXD = byte;
    NRF_TWI0->TASKS_RESUME = 1;

    while (NRF_TWI0->EVENTS_TXDSENT == 0);
    NRF_TWI0->EVENTS_TXDSENT = 0;

    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STARTRX = 1;

    while (NRF_TWI0->EVENTS_RXDREADY == 0);
    NRF_TWI0->EVENTS_RXDREADY = 0;

    NRF_TWI0->TASKS_RESUME = 1;
    NRF_TWI0->TASKS_STOP = 1;
}

void configureAccelerometer()
{
    writeAccelerometerData(0x20, 0x97);
    writeAccelerometerData(0x23, 0x08);
}

void sendByte(char byte, int delay_count)
{
    transmitAndDelay(false, delay_count);
    for (int i = 0; i < 8; i++)
        transmitAndDelay((byte >> i) & 0x01, delay_count);
    transmitAndDelay(true, delay_count);
}

void bitBangSerial(char *str, int delay_count)
{
    initGPIO();

    while (*str)
        sendByte(*str++, delay_count);
}

void showAccelerometerSample()
{
    configureI2C();
    configureAccelerometer();

    char output[35];
    char numStr[6];
    int16_t x, x1, x2;
    int16_t y, y1, y2;
    int16_t z, z1, z2;

    while (1) {
        x1 = readAccelerometerData(X_HIGH_AXIS_REGISTER) << 8;
        y1 = readAccelerometerData(Y_HIGH_AXIS_REGISTER) << 8;
        z1 = readAccelerometerData(Z_HIGH_AXIS_REGISTER) << 8;
        x2 = readAccelerometerData(X_LOW_AXIS_REGISTER);
        y2 = readAccelerometerData(Y_LOW_AXIS_REGISTER);
        z2 = readAccelerometerData(Z_LOW_AXIS_REGISTER);

        x = (int16_t)((x1 | x2) >> 5);
        y = (int16_t)((y1 | y2) >> 5);
        z = (int16_t)((z1 | z2) >> 5);

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

        bitBangSerial(output, DELAY_11520_BAUD);
        delayMilliseconds(200);  // 200 ms delay
    }
}

#ifdef _DEBUG

#define STRING_LITERAL "1:1 In the beginning God created the heaven and the earth.\r\n1:2 And the earth was without form, and void; and darkness was upon\r\nthe face of the deep. And the Spirit of God moved upon the face of the\r\n\r\nwaters.\r\n\r\n1:3 And God said, Let there be light: and there was light.\r\n\r\n1:4 And God saw the light, that it was good: and God divided the light\r\nfrom the darkness.\r\n\r\n1:5 And God called the light Day, and the darkness he called Night.\r\nAnd the evening and the morning were the first day.\r\n\r\n1:6 And God said, Let there be a firmament in the midst of the waters,\r\nand let it divide the waters from the waters.\r\n\r\n1:7 And God made the firmament, and divided the waters which were\r\nunder the firmament from the waters which were above the firmament:\r\nand it was so.\r\n\r\n1:8 And God called the firmament Heaven. And the evening and the\r\nmorning were the second day.\r\n\r\n1:9 And God said, Let the waters under the heaven be gathered together\r\nunto one place, and let the dry land appear: and it was so.\r\n\r\n1:10 And God called the dry land Earth; and the gathering together of\r\nthe waters called he Seas: and God saw that it was good.\r\n\r\n1:11 And God said, Let the earth bring forth grass, the herb yielding\r\nseed, and the fruit tree yielding fruit after his kind, whose seed is\r\nin itself, upon the earth: and it was so.\r\n\r\n1:12 And the earth brought forth grass, and herb yielding seed after\r\nhis kind, and the tree yielding fruit, whose seed was in itself, after\r\nhis kind: and God saw that it was good.\r\n\r\n1:13 And the evening and the morning were the third day.\r\n\r\n1:14 And God said, Let there be lights in the firmament of the heaven\r\nto divide the day from the night; and let them be for signs, and for\r\nseasons, and for days, and years: 1:15 And let them be for lights in\r\nthe firmament of the heaven to give light upon the earth: and it was\r\nso.\r\n\r\n1:16 And God made two great lights; the greater light to rule the day,\r\nand the lesser light to rule the night: he made the stars also.\r\n\r\n1:17 And God set them in the firmament of the heaven to give light\r\nupon the earth, 1:18 And to rule over the day and over the night, and\r\nto divide the light from the darkness: and God saw that it was good.\r\n\r\n1:19 And the evening and the morning were the fourth day.\r\n\r\n1:20 And God said, Let the waters bring forth abundantly the moving\r\ncreature that hath life, and fowl that may fly above the earth in the\r\nopen firmament of heaven.\r\n\r\n1:21 And God created great whales, and every living creature that\r\nmoveth, which the waters brought forth abundantly, after their kind,\r\nand every winged fowl after his kind: and God saw that it was good.\r\n\r\n1:22 And God blessed them, saying, Be fruitful, and multiply, and fill\r\nthe waters in the seas, and let fowl multiply in the earth.\r\n\r\n1:23 And the evening and the morning were the fifth day.\r\n\r\n1:24 And God said, Let the earth bring forth the living creature after\r\nhis kind, cattle, and creeping thing, and beast of the earth after his\r\nkind: and it was so.\r\n\r\n1:25 And God made the beast of the earth after his kind, and cattle\r\nafter their kind, and every thing that creepeth upon the earth after\r\nhis kind: and God saw that it was good.\r\n\r\n1:26 And God said, Let us make man in our image, after our likeness:\r\nand let them have dominion over the fish of the sea, and over the fowl\r\nof the air, and over the cattle, and over all the earth, and over\r\nevery creeping thing that creepeth upon the earth.\r\n\r\n1:27 So God created man in his own image, in the image of God created\r\nhe him; male and female created he them.\r\n\r\n1:28 And God blessed them, and God said unto them, Be fruitful, and\r\nmultiply, and replenish the earth, and subdue it: and have dominion\r\nover the fish of the sea, and over the fowl of the air, and over every\r\nliving thing that moveth upon the earth.\r\n\r\n1:29 And God said, Behold, I have given you every herb bearing seed,\r\nwhich is upon the face of all the earth, and every tree, in the which\r\nis the fruit of a tree yielding seed; to you it shall be for meat.\r\n\r\n1:30 And to every beast of the earth, and to every fowl of the air,\r\nand to every thing that creepeth upon the earth, wherein there is\r\nlife, I have given every green herb for meat: and it was so.\r\n\r\n1:31 And God saw every thing that he had made, and, behold, it was\r\nvery good. And the evening and the morning were the sixth day."
#define SMALL_STRING "findPatterns :: String -> [String] -> [[Int]]; findPatterns data' patterns = [ [idx | idx <- [1..length data' - length p + 1], p == take (length p) (drop (idx - 1) data')] | p <- patterns]"
#define STRING_SIZE sizeof(STRING_LITERAL)

int main()
{
    char string[STRING_SIZE];
    strncpy(string, STRING_LITERAL, STRING_SIZE);

    // bitBangSerial(string, DELAY_11520_BAUD);
    showAccelerometerSample();
}

#endif