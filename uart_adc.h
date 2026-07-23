#ifndef __UART_ADC_H
#define __UART_ADC_H

#include "board.h"

extern volatile uint16_t xdata g_adc_voltage;
extern volatile uint16_t xdata g_adc_current;

void uart_adc_init(void);
uint16_t uart_adc_get_voltage(void);
uint16_t uart_adc_get_current(void);

#endif
