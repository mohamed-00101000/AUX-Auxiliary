#ifndef INC_AS5600_H_
#define INC_AS5600_H_
/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdint.h>
#include "stm32f4xx_hal.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define AS5600_SLAVE_ADDRESS 0x36 << 1
/* AS5600 Configuration Registers */
#define AS5600_REGISTER_ZMCO 0x00
#define AS5600_REGISTER_ZPOS_HIGH 0x01
#define AS5600_REGISTER_ZPOS_LOW 0x02
#define AS5600_REGISTER_MPOS_HIGH 0x03
#define AS5600_REGISTER_MPOS_LOW 0x04
#define AS5600_REGISTER_MANG_HIGH 0x05
#define AS5600_REGISTER_MANG_LOW 0x06
#define AS5600_REGISTER_CONF_HIGH 0x07
#define AS5600_REGISTER_CONF_LOW 0x08
/* AS5600 Output Registers */
#define AS5600_REGISTER_RAW_ANGLE_HIGH 0x0C
#define AS5600_REGISTER_RAW_ANGLE_LOW 0x0D
#define AS5600_REGISTER_ANGLE_HIGH 0x0E
#define AS5600_REGISTER_ANGLE_LOW 0x0F
/* AS5600 Status Registers */
#define AS5600_REGISTER_STATUS 0x0B
#define AS5600_REGISTER_AGC 0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH 0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW 0x1C
#define AS5600_REGISTER_BURN 0xFF
/* AS5600 Configuration Settings */
#define AS5600_POWER_MODE_NOM 1
#define AS5600_POWER_MODE_LPM1 2
#define AS5600_POWER_MODE_LPM2 3
#define AS5600_POWER_MODE_LPM3 4
#define AS5600_POWER_MODE_DEFAULT AS5600_POWER_MODE_NOM
#define AS5600_HYSTERESIS_OFF 1
#define AS5600_HYSTERESIS_1LSB 2
#define AS5600_HYSTERESIS_2LSB 3
#define AS5600_HYSTERESIS_3LSB 4
#define AS5600_HYSTERESIS_DEFAULT AS5600_HYSTERESIS_OFF
#define AS5600_OUTPUT_STAGE_FULL                                               \
    1 /* Ratiometric analog output ranging from GND-VCC*/
#define AS5600_OUTPUT_STAGE_REDUCED                                            \
    2 /* Ratiometric analog output ranging from 10% to 90% of VCC */
#define AS5600_OUTPUT_STAGE_PWM 3 /* Digital PWM output */
#define AS5600_OUTPUT_STAGE_DEFAULT AS5600_OUTPUT_STAGE_FULL
#define AS5600_PWM_FREQUENCY_115HZ 1
#define AS5600_PWM_FREQUENCY_230HZ 2
#define AS5600_PWM_FREQUENCY_460HZ 3
#define AS5600_PWM_FREQUENCY_920HZ 4
#define AS5600_PWM_FREQUENCY_DEFAULT AS5600_PWM_FREQUENCY_115HZ
#define AS5600_SLOW_FILTER_16X 1
#define AS5600_SLOW_FILTER_8X 2
#define AS5600_SLOW_FILTER_4X 3
#define AS5600_SLOW_FILTER_2X 4
#define AS5600_SLOW_FILTER_DEFAULT AS5600_SLOW_FILTER_16X
#define AS5600_FAST_FILTER_SLOW_ONLY 1
#define AS5600_FAST_FILTER_6LSB 2
#define AS5600_FAST_FILTER_7LSB 3
#define AS5600_FAST_FILTER_9LSB 4
#define AS5600_FAST_FILTER_18LSB 5
#define AS5600_FAST_FILTER_21LSB 6
#define AS5600_FAST_FILTER_24LSB 7
#define AS5600_FAST_FILTER_10LSB 8
#define AS5600_FAST_FILTER_DEFAULT AS5600_FAST_FILTER_SLOW_ONLY
#define AS5600_WATCHDOG_OFF 1
#define AS5600_WATCHDOG_ON 2
#define AS5600_WATCHDOG_DEFAULT AS5600_WATCHDOG_ON
/* AS5600 Status Definitions */
#define AS5600_AGC_MIN_GAIN_OVERFLOW                                           \
    (uint8_t)(1UL << 3) /*Error bit indicates b-field is too string */
#define AS5600_AGC_MAX_GAIN_OVERFLOW                                           \
    (uint8_t)(1UL << 4) /*Error bit indicates b-field is too weak */
#define AS5600_MAGNET_DETECTED                                                 \
    (uint8_t)(1UL << 5) /*Status bit indicates b-field is detected */

#define AS5600_DIR_CW 1
#define AS5600_DIR_CCW 2

#define AS5600_12_BIT_MASK (uint16_t)4095
typedef struct {
	uint16_t Raw_Data;
	uint16_t Raw_sensor_Angle;
	uint16_t sensor_Angle;
	uint16_t Filtered_Angle;
	uint16_t Value_Magnitude;
	uint8_t Value_AGC;
	uint8_t Status_Magnet;
    /* Private */
    volatile uint8_t confRegister[2];

} AS5600_TypeDef;
typedef struct {
	uint8_t Burn_Angle;
	uint16_t ZPOS;
	uint16_t MPOS;
	uint16_t Maximum_angle;
	uint16_t Config;
} AS5600_Config_TypeDef;
/***********************    MEDİAN FİLTER    ***********************/
#define MEDIAN_FILTER_SIZE 5
#define MOVING_AVERAGE_SIZE 5
static uint16_t readings[5];
static uint8_t index = 0;
static uint16_t readings_m[MOVING_AVERAGE_SIZE];
static uint8_t index_m = 0;
static uint16_t sum = 0;
static uint8_t count = 0;

/***********************    FUNCTION PROTOTYPES    ***********************/
void AS5600_Init();
void AS5600_Set_Angle(AS5600_Config_TypeDef *Config);
void Get_Magnet_Status(AS5600_TypeDef *Sensor);
void Get_AGC(AS5600_TypeDef *Sensor);
void Get_MAGNITUDE(AS5600_TypeDef *Sensor);
void Get_Raw_Angle(AS5600_TypeDef *Sensor);
void Get_Angle(AS5600_TypeDef *Sensor);
void medianFilter(uint16_t *data, uint8_t size);
uint16_t getMedian(uint16_t *data, uint8_t size);
#endif /* INC_AS5600_H_ */
