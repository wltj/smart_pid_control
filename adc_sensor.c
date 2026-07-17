#include <adc_sensor.h>
#include <stc8a8k64d4.h>
#include <board.h>

#define ADC_POWER  0x80
#define ADC_START  0x40
#define ADC_FLAG   0x20
#define ADC_FILTER_COUNT 8
#define ADC_RESOLUTION 4096
#define ADC_CONVERT_TIMEOUT 10000U
#define EXT_INPUT_FULL_SCALE 5
#define ADC_RESULT_RIGHT_ALIGN 0x20  // 0x30
#define ADC_REF_MV 5000
#define NTC_SUPPLY_MV 5000
#define NTC_R0 10000
#define NTC_B 3950
#define NTC_T0_K 29815
#define PULL_UP_RES 10000

uint16_t xdata g_adc_temp1 = 0;
uint16_t xdata g_adc_temp2 = 0;
uint16_t xdata g_adc_temp3 = 0;
uint16_t xdata g_adc_temp4 = 0;
uint16_t xdata g_adc_ext1 = 0;
uint16_t xdata g_adc_ext2 = 0;

static uint16_t xdata adc_buf_temp1[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_temp2[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_temp3[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_temp4[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_ext1[ADC_FILTER_COUNT] = {0};
static uint16_t xdata adc_buf_ext2[ADC_FILTER_COUNT] = {0};
static uint8_t xdata adc_buf_index = 0;

/* ln查表表，ln_table[n] = ln(n) * 1000 (n=0..100) */
static const int16_t code ln_table[101] = {
    0, 0, 693, 1098, 1386, 1609, 1791, 1945, 2079, 2197,
    2302, 2397, 2484, 2564, 2639, 2708, 2772, 2833, 2890, 2944,
    2995, 3044, 3091, 3135, 3178, 3218, 3258, 3295, 3332, 3367,
    3401, 3433, 3465, 3496, 3526, 3555, 3583, 3610, 3637, 3663,
    3688, 3712, 3736, 3759, 3781, 3802, 3823, 3843, 3862, 3881,
    3900, 3918, 3936, 3954, 3971, 3988, 4004, 4020, 4035, 4050,
    4065, 4079, 4093, 4107, 4120, 4133, 4146, 4158, 4170, 4182,
    4193, 4204, 4215, 4226, 4236, 4247, 4257, 4267, 4277, 4286,
    4296, 4305, 4314, 4323, 4332, 4340, 4349, 4357, 4365, 4373,
    4381, 4388, 4396, 4403, 4410, 4418, 4425, 4431, 4438, 4445,
    4605
};

static int16_t int_ln(uint16_t x)
{
    uint8_t shift;
    uint16_t tmp;

    if (x == 0) return -32767;
    if (x == 1) return 0;

    shift = 0;
    tmp = x;

    while (tmp >= 100) {
        tmp /= 10;
        shift++;
    }

    return ln_table[tmp] + (shift * 2302);
}

static uint16_t ntc_adc_to_temp(uint16_t adc)
{
    uint32_t vadc;
    uint32_t v_diff;
    uint32_t r_ntc;
    uint32_t ratio_x100;
    int32_t ln_ratio;
    int32_t inv_t;
    uint32_t t_k;
    int32_t t_c;

    if (adc >= ADC_RESOLUTION) adc = ADC_RESOLUTION - 1;
    if (adc == 0) return 0;

    /* Board assumption: ADC reference and NTC divider pull-up are +5V. */
    vadc = (uint32_t)adc * ADC_REF_MV / ADC_RESOLUTION;

    if (vadc >= NTC_SUPPLY_MV || vadc == 0) return 0;

    v_diff = NTC_SUPPLY_MV - vadc;
    if (v_diff == 0) return 0;
    r_ntc = (uint32_t)PULL_UP_RES * vadc / v_diff;

    if (r_ntc >= NTC_R0) {
        ratio_x100 = (uint32_t)r_ntc * 100 / NTC_R0;
    } else {
        ratio_x100 = (uint32_t)NTC_R0 * 100 / r_ntc;
    }
    if (ratio_x100 > 60000) ratio_x100 = 60000;

    ln_ratio = int_ln((uint16_t)ratio_x100) - 4605;  /* ln(R/R0*100) - ln(100) = ln(R/R0) */
    if (r_ntc < NTC_R0) ln_ratio = -ln_ratio;

    inv_t = (int32_t)(100000000L / NTC_T0_K) + (ln_ratio * 1000L) / NTC_B;

    if (inv_t <= 0) return 0;

    t_k = 100000000UL / inv_t;
    t_c = (int32_t)t_k - 27315;

    if (t_c < 0) t_c = 0;
    if (t_c > 12500) t_c = 12500;

    return (uint16_t)(t_c / 100);  /* 转为整度数 */
}

static uint16_t ext_adc_to_voltage(uint16_t adc)
{
    if (adc >= ADC_RESOLUTION) adc = ADC_RESOLUTION - 1;
    return (uint16_t)((uint32_t)adc * EXT_INPUT_FULL_SCALE / (ADC_RESOLUTION - 1));
}

static void delay_adc(void)
{
    unsigned char i;
    for (i = 0; i < 10; i++);
}

void adc_sensor_init(void)
{
    ADC_CONTR = 0x80;
    ADCCFG = ADC_RESULT_RIGHT_ALIGN | 0x0F;
    delay_adc();
}

uint16_t adc_sensor_read_channel(unsigned char channel)
{
    uint16_t result;
    uint16_t i;

    ADC_CONTR = ADC_POWER | (channel & 0x0F);
    delay_adc();

    ADC_CONTR &= ~ADC_FLAG;
    ADC_CONTR |= ADC_START;

    for (i = 0; i < 300; i++)
        ;

    for (i = 0; i < ADC_CONVERT_TIMEOUT; i++) {
        if (ADC_CONTR & ADC_FLAG) {
            break;
        }
    }

    if ((ADC_CONTR & ADC_FLAG) == 0) {
        return 0;
    }

    ADC_CONTR &= ~ADC_FLAG;

    result = (((uint16_t)ADC_RES << 8) | ADC_RESL) & (ADC_RESOLUTION - 1);

    return result;
}

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
    uint16_t t1, t2, t3, t4, e1, e2;

    t1 = adc_sensor_read_channel(ADC_TEMP1_CH);
    t2 = adc_sensor_read_channel(ADC_TEMP2_CH);
    t3 = adc_sensor_read_channel(ADC_TEMP3_CH);
    t4 = adc_sensor_read_channel(ADC_TEMP4_CH);
    e1 = adc_sensor_read_channel(ADC_EXT1_CH);
    e2 = adc_sensor_read_channel(ADC_EXT2_CH);

    g_adc_temp1 = adc_filter(adc_buf_temp1, t1);
    g_adc_temp2 = adc_filter(adc_buf_temp2, t2);
    g_adc_temp3 = adc_filter(adc_buf_temp3, t3);
    g_adc_temp4 = adc_filter(adc_buf_temp4, t4);
    g_adc_ext1 = adc_filter(adc_buf_ext1, e1);
    g_adc_ext2 = adc_filter(adc_buf_ext2, e2);

    adc_buf_index = (adc_buf_index + 1) % ADC_FILTER_COUNT;
}

uint16_t adc_sensor_get_temp1(void) { return ntc_adc_to_temp(g_adc_temp1); }
uint16_t adc_sensor_get_temp2(void) { return ntc_adc_to_temp(g_adc_temp2); }
uint16_t adc_sensor_get_temp3(void) { return ntc_adc_to_temp(g_adc_temp3); }
uint16_t adc_sensor_get_temp4(void) { return ntc_adc_to_temp(g_adc_temp4); }
uint16_t adc_sensor_get_temp1_raw(void) { return g_adc_temp1; }
uint16_t adc_sensor_get_temp2_raw(void) { return g_adc_temp2; }
uint16_t adc_sensor_get_temp3_raw(void) { return g_adc_temp3; }
uint16_t adc_sensor_get_temp4_raw(void) { return g_adc_temp4; }
uint16_t adc_sensor_get_ext1(void) { return ext_adc_to_voltage(g_adc_ext1); }
uint16_t adc_sensor_get_ext2(void) { return ext_adc_to_voltage(g_adc_ext2); }
