#include <adc_sensor.h>
#include <stc8a8k64d4.h>
#include <board.h>
#define ADC_POWER  0x80
#define ADC_START  0x40
#define ADC_FLAG   0x20
#define ADC_FILTER_COUNT 8 // 每个通道采样8次取平均
uint16_t xdata g_adc_temp1 = 0;
uint16_t xdata g_adc_temp2 = 0;
uint16_t xdata g_adc_temp3 = 0;
uint16_t xdata g_adc_temp4 = 0;
uint16_t xdata g_adc_ext1 = 0;
uint16_t xdata g_adc_ext2 = 0;
// 滤波缓存
static uint16_t xdata adc_buf_temp1[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_temp2[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_temp3[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_temp4[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_ext1[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_ext2[ADC_FILTER_COUNT] = {0};
static uint8_t xdata adc_buf_index = 0;
static void delay_adc(void)
{
    unsigned char i;
    for (i = 0; i < 10; i++);
}
void adc_sensor_init(void)
{
    ADC_CONTR = 0x80;
    ADCCFG = 0x20 | 0x0F; // 12位精度，结果右对齐
    delay_adc();
}
uint16_t adc_sensor_read_channel(unsigned char channel)
{
    uint16_t result;
    unsigned char i;
    
    ADC_CONTR = ADC_POWER | (channel & 0x0F);
    delay_adc();
    
    ADC_CONTR |= ADC_START;
    
    for (i = 0; i < 255; i++) {
        if (ADC_CONTR & ADC_FLAG) {
            break;
        }
    }
    
    ADC_CONTR &= ~ADC_FLAG;
    
    result = ADC_RES;
    result = (result << 4) | (ADC_RESL & 0x0F);
    
    return result;
}
// 滑动平均滤波
static uint16_t adc_filter(uint16_t *buf, uint16_t new_val)
{
    uint32_t sum = 0;
    uint8_t i;
    buf[adc_buf_index] = new_val;
    for (i = 0; i < ADC_FILTER_COUNT; i++) {
        sum += buf[i];
    }
    return (uint16_t)(sum / ADC_FILTER_COUNT);
}
void adc_sensor_read_all(void)
{
    // 采样每个通道
    uint16_t t1 = adc_sensor_read_channel(ADC_TEMP1_CH);
    uint16_t t2 = adc_sensor_read_channel(ADC_TEMP2_CH);
    uint16_t t3 = adc_sensor_read_channel(ADC_TEMP3_CH);
    uint16_t t4 = adc_sensor_read_channel(ADC_TEMP4_CH);
    uint16_t e1 = adc_sensor_read_channel(ADC_EXT1_CH);
    uint16_t e2 = adc_sensor_read_channel(ADC_EXT2_CH);
    // 滤波
    g_adc_temp1 = adc_filter(adc_buf_temp1, t1);
    g_adc_temp2 = adc_filter(adc_buf_temp2, t2);
    g_adc_temp3 = adc_filter(adc_buf_temp3, t3);
    g_adc_temp4 = adc_filter(adc_buf_temp4, t4);
    g_adc_ext1 = adc_filter(adc_buf_ext1, e1);
    g_adc_ext2 = adc_filter(adc_buf_ext2, e2);
    // 更新缓存索引
    adc_buf_index = (adc_buf_index + 1) % ADC_FILTER_COUNT;
}
uint16_t adc_sensor_get_temp1(void) { return g_adc_temp1; }
uint16_t adc_sensor_get_temp2(void) { return g_adc_temp2; }
uint16_t adc_sensor_get_temp3(void) { return g_adc_temp3; }
uint16_t adc_sensor_get_temp4(void) { return g_adc_temp4; }
uint16_t adc_sensor_get_ext1(void) { return g_adc_ext1; }
uint16_t adc_sensor_get_ext2(void) { return g_adc_ext2; }
