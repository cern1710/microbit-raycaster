#include "Microbit.h"
#include "nrf52833.h"

// #define _DEBUG
// #define _LOOPY

#define BIT_SHIFT(n)  (uint32_t)(1 << n)

/* Milliseconds -> clock cycle conversion */
#define PRESCALER_VALUE 8
#define BASE_FREQ_16M   16000000.0
#define MS_PER_SECOND   1000

/* LED display macros */
#define LED_ROWS    5
#define LED_COLS    5
#define NUM_DIGITS  10
#define BUF_LEN     12  // Max length of character buffer for scrolling integer

/* Converts column of digit in led_digit[] to binary */
#define BIN_DIGIT(col)  (1 << (LED_COLS - 1 - col))

/* Delay milliseconds for timer/interrupts */
#define MS_TO_10HZ            4     // (4ms * 5rows) = 20 ms => 1/50 of a second
#ifdef _LOOPY
    #define SCROLL_SPEED_IN_MS  50
#else
    #define SCROLL_SPEED_IN_MS 200
#endif
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
    COL4 = 5,   // Note: COL4 is in P1
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

/* Global variables for modification by timer interrupt */
volatile int current_row = 0;
volatile bool led_buffer[LED_ROWS][LED_COLS] = { false };

/* ####################################  LED functions  #################################### */

void setLED(Rows row, Columns col)
{
    // We do this check because COL4 is in P1 (srsly, why?)
    ((col == COL4) ? NRF_P1 : NRF_P0)->DIR |= BIT_SHIFT(col);
    NRF_P0->DIR |= BIT_SHIFT(row);
    NRF_P0->OUT = BIT_SHIFT(row);
}

void clearLEDs()
{
    NRF_P1->DIR = 0;
    NRF_P0->DIR = 0;
}

/* Updates a row of the LED buffer; used by timer interrupt */
void updateLEDRow(int row)
{
    for (int col = 0; col < LED_COLS; col++)
        if (led_buffer[row][col])
            setLED(rows[row], cols[col]);
}

/* Updates the entire LED buffer with given array */
void updateLEDBuffer(const bool led_states[LED_ROWS][LED_COLS])
{
    for (int row = 0; row < LED_ROWS; row++)
        for (int col = 0; col < LED_COLS; col++)
            led_buffer[row][col] = led_states[row][col];
}

/* Updates the LED buffer with given digit by matchinig with BIN_DIGIT() */
void updateBufferWithDigit(int digit)
{
    bool led_states[LED_ROWS][LED_COLS] = { false };

    for (int row = 0; row < LED_ROWS; row++)
        for (int col = 0; col < LED_COLS; col++)
            if (led_digits[digit][row] & BIN_DIGIT(col))
                led_states[row][col] = true;
    updateLEDBuffer(led_states);
}

/* ####################################  Timer functions  #################################### */

uint32_t convertMsToTicks(float delay_ms)
{
    float fTIMER = BASE_FREQ_16M / BIT_SHIFT(PRESCALER_VALUE);
    float ticks_per_ms = fTIMER / MS_PER_SECOND;

    return ticks_per_ms * delay_ms;
}

void initRegularTimer()
{
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;
    NRF_TIMER0->PRESCALER = PRESCALER_VALUE;
    NRF_TIMER0->TASKS_START = 1;
}

void startRegularTimer()
{
    initRegularTimer();
    NRF_TIMER0->TASKS_START = 1;
}

uint32_t captureTime()
{
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    NRF_TIMER0->TASKS_CAPTURE[0] = 0;   // Stops repeated captures
    return NRF_TIMER0->CC[0];
}

/**
 * Hardware-based delay waits until timer reaches specified absolute time.
 *
 * This status-driven control mechanism ensures higher accuracy by preventing
 * cumulative drift. The system pauses, on average, for the provided interval.
 */
void delayUntil(uint32_t next_time)
{
    NRF_TIMER0->CC[0] = next_time;
    while (!(NRF_TIMER0->EVENTS_COMPARE[0]));
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
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

/* ####################################  Interrupt functions  #################################### */

void TIMER1_IRQHandler()
{
    if (NRF_TIMER1->EVENTS_COMPARE[0] == 0) return;

    NRF_TIMER1->EVENTS_COMPARE[0] = 0;           // Clear match event
    clearLEDs();
    updateLEDRow(current_row);                   // Update current row
    current_row = (current_row + 1) % LED_ROWS;  // Move to the next row
}

void initInterruptTimer(uint32_t interval)
{
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER1->PRESCALER = PRESCALER_VALUE;
    NRF_TIMER1->CC[0] = interval; // Set interval for compare match
}

/* Configure timer to enable interrupt on match */
void configureInterrupt()
{
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NVIC_SetVector(TIMER1_IRQn, (uint32_t)TIMER1_IRQHandler);
    NVIC_EnableIRQ(TIMER1_IRQn);
    NVIC_SetPriority(TIMER1_IRQn, 0);
}

void startInterruptTimer(uint32_t interval)
{
    initInterruptTimer(interval);
    configureInterrupt();
    NRF_TIMER1->TASKS_START = 1;
}

/* ####################################  String manipulation functions  #################################### */

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

/* Converts positive integers to string */
void intToStr(char *str, int num)
{
    int i = 0;

    if (num == 0) {
        str[i++] = '0'; // Special case where num == 0 (avoid division by zero)
    } else {
        for (; num > 0; num /= 10)        // Remove rightmost digit in each iteration
            str[i++] = (num % 10) + '0';  // Convert last digit to char, add to str
        reverse(str, i); // Reverse string because digits are added in reverse
    }
    str[i] = '\0';
}

int my_strlen(char *str)
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
    startInterruptTimer(convertMsToTicks(MS_TO_10HZ));
}

/**
 * Display a digit on the LED matrix starting at a specific column.
 *
 * The digit is displayed using the LED states from the `led_digits` array.
 */
void showPartialDigit(int n, int start_col, bool led_states[LED_ROWS][LED_COLS])
{
    int led_col;
    uint8_t row_data;

    for (int row = 0; row < LED_ROWS; row++) {
        row_data = led_digits[n][row];  // Digit binary representation of row

        for (int col = 0; col < LED_COLS; col++) {
            led_col = col + start_col;

            if (led_col < 0 || led_col >= LED_COLS) // Skip out-of-bounds columns
                continue;

            /* Set the LED state based on the binary representation of the digit */
            led_states[row][led_col] = (row_data & BIN_DIGIT(col));
        }
    }
    updateLEDBuffer(led_states);
}

/**
 * Convert a number to a string and display it as a scrolling text on the LED matrix.
 */
void showScrollingNumber(int n)
{
    int digit_start_col;
    const uint32_t scroll_speed = convertMsToTicks(SCROLL_SPEED_IN_MS);
    bool led_states[LED_ROWS][LED_COLS] = { false };

    /* Convert number to a string and store it to num_str[] buffer */
    char num_str[BUF_LEN];
    intToStr(num_str, n);
    int length = my_strlen(num_str);

    startInterruptTimer(convertMsToTicks(MS_TO_10HZ));
    startRegularTimer();
    uint32_t next_time = captureTime() + scroll_speed;

    for (int pos = length * LED_COLS; pos >= -LED_COLS; pos--) {
        my_memset(led_states, 0, sizeof(led_states));

        for (int len = 0; len < length; len++) {
            /* Calculate the starting column of the current digit on the LED matrix */
            digit_start_col = pos - (length - 1 - len) * LED_COLS;

            if (num_str[len] >= '0' && num_str[len] <= '9')
                showPartialDigit(num_str[len] - '0', digit_start_col, led_states);
        }
        delayUntil(next_time);
        next_time += scroll_speed;
    }
}

/* ####################################  Task functions  #################################### */

void beHappy()
{
    int row, col;

    while (1) {
        for (row = 0; row < LED_ROWS; row++) {
            for (col = 0; col < LED_COLS; col++)
                if (smiley_face[row][col])
                    setLED(rows[row], cols[col]);
            delay(MS_TO_10HZ);
            clearLEDs();
        }
    }
}

void beVeryHappy()
{
    int row, col;
    const uint32_t interval = convertMsToTicks(MS_TO_10HZ);

    startRegularTimer();
    uint32_t next_time = captureTime() + interval;

    while (1) {
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
    startInterruptTimer(convertMsToTicks(MS_TO_10HZ));
}

void showNumber(int n)
{
    if (n < 0)  // Return if digit is negative
        return;
    if (n >= 10)
        showScrollingNumber(n);
    else
        showSingleNumber(n);
}

#ifdef _DEBUG
#define TEST_UINT 1234567890

#ifdef _LOOPY
void speedyScroll(int n)
{
    showScrollingNumber(n);
}

void loopyBoi()
{
    for (int i = 1; i <= 536870912; i <<= 1)
        speedyScroll(i);
}
#endif

int main()
{
    beHappy();
    // beVeryHappy();
    // beHappyAndFree();
    // showNumber(TEST_UINT);

    /* Extra funny */
    // loopyBoi();
}
#endif