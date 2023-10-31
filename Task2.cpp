#include "Microbit.h"
#include "nrf52833.h"

#define _DEBUG

#define BIT_SHIFT(pin)  (uint32_t)(1 << pin)

#define BASE_FREQ_16M   16000000.0
#define MS_PER_SECOND   1000

#define PRESCALER_VALUE 8

#define LED_ROWS    5
#define LED_COLS    5
#define NUM_DIGITS  10
#define NUM_ITER    25

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

const Rows rows[] = {ROW1, ROW2, ROW3, ROW4, ROW5};
const Columns cols[] = {COL1, COL2, COL3, COL4, COL5};

const uint8_t led_digits[NUM_DIGITS][LED_ROWS] = {
    { 0b01110, 0b01010, 0b01010, 0b01010, 0b01110 },    // 0
    { 0b00100, 0b01100, 0b00100, 0b00100, 0b01110 },    // 1
    { 0b01110, 0b00010, 0b01110, 0b01000, 0b01110 },    // 2
    { 0b01110, 0b00010, 0b01110, 0b00010, 0b01110 },    // 3
    { 0b01010, 0b01010, 0b01110, 0b00010, 0b00010 },    // 4
    { 0b01110, 0b01000, 0b01110, 0b00010, 0b01110 },    // 5
    { 0b01110, 0b01000, 0b01110, 0b01010, 0b01110 },    // 6
    { 0b01110, 0b01010, 0b00010, 0b00010, 0b00010 },    // 7
    { 0b01110, 0b01010, 0b01110, 0b01010, 0b01110 },    // 8
    { 0b01110, 0b01010, 0b01110, 0b00010, 0b01110 }     // 9
};

const bool smiley_face[LED_ROWS][LED_COLS] = {
    {0, 0, 0, 0, 0},
    {0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0},
    {1, 0, 0, 0, 1},
    {0, 1, 1, 1, 0}
};

volatile int current_row = 0;
volatile bool led_buffer[LED_ROWS][LED_COLS] = { false };

/* ####################################  Timer functions  #################################### */

uint32_t convertMsToTicks(float delay_ms)
{
    float fTIMER = BASE_FREQ_16M / BIT_SHIFT(PRESCALER_VALUE);
    float ticks_per_ms = fTIMER / MS_PER_SECOND;

    return (uint32_t)(ticks_per_ms * delay_ms);
}

void initRegularTimer()
{
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_24Bit;
    NRF_TIMER1->PRESCALER = PRESCALER_VALUE;
    NRF_TIMER1->TASKS_START = 1;
}

uint32_t captureTime()
{
    NRF_TIMER1->TASKS_CAPTURE[0] = 1;
    NRF_TIMER1->TASKS_CAPTURE[0] = 0;   // Stops repeated captures
    return NRF_TIMER1->CC[0];
}

/**
 * Hardware-based delay waits until timer reaches specified absolute time.
 *
 * This status-driven control mechanism ensures higher accuracy by preventing
 * cumulative drift. The system pauses, on average, for the provided interval.
 */
void delayUntil(uint32_t next_time)
{
    NRF_TIMER1->CC[0] = next_time;
    while (!(NRF_TIMER1->EVENTS_COMPARE[0]));
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
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

/* ####################################  LED functions  #################################### */

void setLED(Rows row, Columns col)
{
    ((col == COL4) ? NRF_P1 : NRF_P0)->DIR |= BIT_SHIFT(col);
    NRF_P0->DIR |= BIT_SHIFT(row);
    NRF_P0->OUT = BIT_SHIFT(row);
}

void clearLEDs()
{
    NRF_P1->DIR = 0;
    NRF_P0->DIR = 0;
}

void updateLEDRow(int row)
{
    int col;

    for (col = 0; col < LED_COLS; col++)
        if (led_buffer[row][col])
            setLED(rows[row], cols[col]);
}

void updateLEDBuffer(const bool led_states[LED_ROWS][LED_COLS])
{
    int row, col;

    for (row = 0; row < LED_ROWS; row++)
        for (col = 0; col < LED_COLS; col++)
            led_buffer[row][col] = led_states[row][col];
}

void updateBufferWithDigit(int digit) {
    bool led_states[LED_ROWS][LED_COLS] = { false };
    int row, col;

    for (row = 0; row < LED_ROWS; ++row)
        for (col = 0; col < LED_COLS; ++col)
            if (led_digits[digit][row] & (1 << (LED_COLS - 1 - col)))
                led_states[row][col] = true;
    updateLEDBuffer(led_states);
}

/* ####################################  Interrupt functions  #################################### */

void initInterruptTimer(uint32_t interval)
{
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER1->PRESCALER = PRESCALER_VALUE;
    NRF_TIMER1->CC[0] = interval; // Set interval for compare match
}

void startInterruptTimer()
{
    NRF_TIMER1->TASKS_START = 1;
}

void TIMER1_IRQHandler()
{
    if (NRF_TIMER1->EVENTS_COMPARE[0] != 0) {
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;  // Clear match event

        clearLEDs();
        updateLEDRow(current_row);  // Update current row
        current_row = (current_row + 1) % LED_ROWS;  // Move to the next row
    }
}

void configureInterrupt()
{
    // Enable interrupt on compare match
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;

    // Enable interrupt in the NVIC
    NVIC_SetVector(TIMER1_IRQn, (uint32_t)TIMER1_IRQHandler);
    NVIC_EnableIRQ(TIMER1_IRQn);
    NVIC_SetPriority(TIMER1_IRQn, 0);
}

/* ####################################  String manipulation functions  #################################### */

void reverse(char *str, int length)
{
    int start = 0, end = length - 1;
    char temp;

    for (; start < end; start++, end--) {
        temp = str[start];
        str[start] = str[end];
        str[end] = temp;
    }
}

void intToStr(char *str, int num)
{
    int i = 0;

    if (num == 0) {
        str[i++] = '0';
    } else {
        while (num > 0) {
            str[i++] = (num % 10) + '0';  // Convert last digit to char, add to str
            num /= 10;  // Remove last digit
        }
        reverse(str, i); // Reverse string because digits are added in reverse
    }
    str[i] = '\0';
}

int strLength(char *str)
{
    int length = 0;

    while (str[length++] != '\0');
    return length;
}

void* my_memset(void* ptr, int value, size_t num)
{
    unsigned char* p = static_cast<unsigned char*>(ptr);

    while (num--)
        *p++ = static_cast<unsigned char>(value);
    return ptr;
}

/* ####################################  Number scrolling functions  #################################### */

void showSingleNumber(int n)
{
    updateBufferWithDigit(n);
    initInterruptTimer(convertMsToTicks(4));
    configureInterrupt();
    startInterruptTimer();
}

void showPartialDigit(int n, int start_col,
                        bool led_states[LED_ROWS][LED_COLS])
{
    if (start_col > LED_COLS) return;
    int row, col, led_col;
    uint8_t row_data;

    for (row = 0; row < LED_ROWS; ++row) {
        row_data = led_digits[n][row];

        for (col = 0; col < LED_COLS; ++col) {
            led_col = col + start_col;

            if (led_col < 0 || led_col >= LED_COLS)
                continue;
            led_states[row][led_col] =
                (row_data & (1 << (LED_COLS - 1 - col))) != 0;
        }
    }
    updateLEDBuffer(led_states);
}

void showScrollingNumber(int n)
{
    char num_str[12];  // Enough space for 32-bit integer
    intToStr(num_str, n);
    int length = strLength(num_str);
    bool led_states[LED_ROWS][LED_COLS] = { false };
    int pos, len;
    int digit_start_col;

    initInterruptTimer(4);
    configureInterrupt();
    startInterruptTimer();

    for (pos = length * LED_COLS; pos >= -LED_COLS; pos--) {
        my_memset(led_states, 0, sizeof(led_states));  // Clear the buffer
        for (len = 0; len < length; len++) {
            digit_start_col = pos - (length - 1 - len) * LED_COLS; // Reverse

            if (num_str[len] >= '0' && num_str[len] <= '9')
                showPartialDigit(num_str[len] - '0', digit_start_col, led_states);
        }
        delay(100);
    }
}

/* ####################################  Task functions  #################################### */

void beHappy()
{
    int row, col;

    while(1) {
        for (row = 0; row < LED_ROWS; row++) {
            for (col = 0; col < LED_COLS; col++)
                if (smiley_face[row][col])
                    setLED(rows[row], cols[col]);
            delay(convertMsToTicks(5));
            clearLEDs();
        }
    }
}

void beVeryHappy()
{
    int row, col;
    const uint32_t interval = convertMsToTicks(5);
    initRegularTimer();
    uint32_t next_time = captureTime() + interval;

    while(1) {
        for (row = 0; row < LED_ROWS; row++) {
            for (col = 0; col < LED_COLS; col++)
                if (smiley_face[row][col])
                    setLED(rows[row], cols[col]);
            delayUntil(next_time);
            next_time += interval;
            clearLEDs();
        }
    }
}
void beHappyAndFree()
{
    updateLEDBuffer(smiley_face);
    initInterruptTimer(convertMsToTicks(5));
    configureInterrupt();
    startInterruptTimer();
}

void showNumber(int n)
{
    if (n < 0) return;  // Out of range

    if (n >= 10)
        showScrollingNumber(n);
    else
        showSingleNumber(n);
}

#ifdef _DEBUG
#define TEST_UINT 239512897

int main()
{
    // beHappy();
    // beVeryHappy();
    // beHappyAndFree();
    showNumber(TEST_UINT);
}
#endif