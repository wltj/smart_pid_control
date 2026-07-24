#include <uart_adc.h>

#define S3RI 0x01
#define S3TI 0x02
#define S4RI 0x01
#define S4TI 0x02

#define UART_ADC_ES3 0x08
#define UART_ADC_ES4 0x10
#define UART_ADC_T3R 0x08
#define UART_ADC_T3X12 0x02
#define UART_ADC_T4R 0x80
#define UART_ADC_T4X12 0x20

#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0xFF

/* 12 位 ADC 分辨率与满量程电压基准，用于将原始值换算成 0~5V 电压量。 */
#define UART_ADC_RESOLUTION 4096
#define UART_ADC_FULL_SCALE 5

/* 接收状态机状态 */
#define STATE_WAIT_HEAD  0
#define STATE_RECV_HIGH  1
#define STATE_RECV_LOW   2
#define STATE_WAIT_TAIL  3

volatile uint16_t xdata g_adc_voltage = 0;
volatile uint16_t xdata g_adc_current = 0;

static volatile unsigned char xdata volt_recv_state = STATE_WAIT_HEAD;
static unsigned char xdata volt_recv_high = 0;
static unsigned char xdata volt_recv_low = 0;
static volatile unsigned char xdata curr_recv_state = STATE_WAIT_HEAD;
static unsigned char xdata curr_recv_high = 0;
static unsigned char xdata curr_recv_low = 0;

static void uart3_init(unsigned long baudrate)
{
    unsigned long reload;
    S3CON = 0x50;
    reload = 65536UL - (11059200UL / 4 / baudrate);
    T3L = reload & 0xFF;
    T3H = (reload >> 8) & 0xFF;
    T4T3M = (T4T3M & 0xF0) | UART_ADC_T3R | UART_ADC_T3X12;
    IE2 |= UART_ADC_ES3;
}

static void uart4_init(unsigned long baudrate)
{
    unsigned long reload;
    S4CON = 0x50;
    reload = 65536UL - (11059200UL / 4 / baudrate);
    T4L = reload & 0xFF;
    T4H = (reload >> 8) & 0xFF;
    T4T3M = (T4T3M & 0x0F) | UART_ADC_T4R | UART_ADC_T4X12;
    IE2 |= UART_ADC_ES4;
}

void UART3_Isr(void) interrupt 17
{
    unsigned char ch;

    if (S3CON & S3RI) {
        S3CON &= ~S3RI;
        ch = S3BUF;
        switch (volt_recv_state) {
            case STATE_WAIT_HEAD:
                if (ch == FRAME_HEAD) {
                    volt_recv_state = STATE_RECV_HIGH;
                }
                break;
            case STATE_RECV_HIGH:
                volt_recv_high = ch;
                volt_recv_state = STATE_RECV_LOW;
                break;
            case STATE_RECV_LOW:
                volt_recv_low = ch;
                volt_recv_state = STATE_WAIT_TAIL;
                break;
            case STATE_WAIT_TAIL:
                if (ch == FRAME_TAIL) {
                    g_adc_voltage = ((uint16_t)volt_recv_high << 8) | volt_recv_low;
                }
                volt_recv_state = STATE_WAIT_HEAD;
                break;
            default:
                volt_recv_state = STATE_WAIT_HEAD;
                break;
        }
    }
}

void UART4_Isr(void) interrupt 18
{
    unsigned char ch;

    if (S4CON & S4RI) {
        S4CON &= ~S4RI;
        ch = S4BUF;
        switch (curr_recv_state) {
            case STATE_WAIT_HEAD:
                if (ch == FRAME_HEAD) {
                    curr_recv_state = STATE_RECV_HIGH;
                }
                break;
            case STATE_RECV_HIGH:
                curr_recv_high = ch;
                curr_recv_state = STATE_RECV_LOW;
                break;
            case STATE_RECV_LOW:
                curr_recv_low = ch;
                curr_recv_state = STATE_WAIT_TAIL;
                break;
            case STATE_WAIT_TAIL:
                if (ch == FRAME_TAIL) {
                    g_adc_current = ((uint16_t)curr_recv_high << 8) | curr_recv_low;
                }
                curr_recv_state = STATE_WAIT_HEAD;
                break;
            default:
                curr_recv_state = STATE_WAIT_HEAD;
                break;
        }
    }
}

void uart_adc_init(void)
{
    volt_recv_state = STATE_WAIT_HEAD;
    curr_recv_state = STATE_WAIT_HEAD;
    g_adc_voltage = 0;
    g_adc_current = 0;
    uart3_init(1200);
    uart4_init(1200);
    EA = 1;
}

/* 将串口接收的 0~4095 原始 ADC 值换算成 0~5V 电压量。 */
static uint16_t uart_adc_raw_to_voltage(uint16_t raw)
{
    if (raw >= UART_ADC_RESOLUTION) raw = UART_ADC_RESOLUTION - 1;
    return (uint16_t)((uint32_t)raw * UART_ADC_FULL_SCALE / (UART_ADC_RESOLUTION - 1));
}

uint16_t uart_adc_get_voltage(void)
{
    uint16_t value;
    unsigned char ea_state;

    ea_state = EA;
    EA = 0;
    value = uart_adc_raw_to_voltage(g_adc_voltage);
    EA = ea_state;
    return value;
}

uint16_t uart_adc_get_current(void)
{
    uint16_t value;
    unsigned char ea_state;

    ea_state = EA;
    EA = 0;
    value = uart_adc_raw_to_voltage(g_adc_current);
    EA = ea_state;
    return value;
}


