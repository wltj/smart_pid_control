#ifndef __ADC_SENSOR_H
#define __ADC_SENSOR_H

#include "board.h"

extern uint16_t xdata g_adc_temp1;
extern uint16_t xdata g_adc_temp2;
extern uint16_t xdata g_adc_temp3;
extern uint16_t xdata g_adc_temp4;
extern uint16_t xdata g_adc_ext1;
extern uint16_t xdata g_adc_ext2;

void adc_sensor_init(void);
void adc_sensor_read_all(void);
uint16_t adc_sensor_read_channel(unsigned char channel);
uint16_t adc_sensor_get_temp1(void);
uint16_t adc_sensor_get_temp2(void);
uint16_t adc_sensor_get_temp3(void);
uint16_t adc_sensor_get_temp4(void);
uint16_t adc_sensor_get_temp1_raw(void);
uint16_t adc_sensor_get_temp2_raw(void);
uint16_t adc_sensor_get_temp3_raw(void);
uint16_t adc_sensor_get_temp4_raw(void);
uint16_t adc_sensor_get_ext1(void);
uint16_t adc_sensor_get_ext2(void);

#endif
