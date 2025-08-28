/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define CAN_MSG_1  0x80
#define CAN_MSG_2  0x90
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* (You can use GPIO_PinState instead) */
typedef struct {
	uint8_t Front_Horn_state;
	uint8_t Front_Beam_state;
} FrontSide_BTNStates;

typedef struct {
	uint8_t Hazard_state;
	uint8_t Right_Flasher_state;
	uint8_t Left_Flasher_state;
	uint8_t Servo_state;
} Common_BtnStates;

typedef struct {
	uint8_t Back_Reverse_state;
	uint8_t Back_Brake_state;
} BackSide_BTNStates;

typedef struct {
	uint16_t Brake_Value;
	uint16_t Throttle_Value;
} Pedal_Values;

//typedef struct {
//    uint8_t status;   // Represents the first byte (e.g., status or flag)
//    uint8_t command;  // Represents the second byte (e.g., command or operation)
//    uint16_t value;   // Represents a 16-bit value (e.g., sensor data)
//    uint32_t id;      // Represents a 32-bit ID or extended data
//} CAN_Message;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define OFF 		0x0
#define ON  		0x1



/*BTN PINS*/
//front BTNs
#define SERVO_BTN					GPIO_PIN_2
#define HORN_BTN					GPIO_PIN_4
#define BEAM_BTN					GPIO_PIN_1

//common BTNs

#define HAZARD_BTN					GPIO_PIN_5
//#define HAZARD_BTN					GPIO_PIN_11
//#define BLINKER_R_BTN 				GPIO_PIN_5
#define BLINKER_R_BTN 				GPIO_PIN_6
#define BLINKER_L_BTN				GPIO_PIN_1

//back BTNs
#define REVERSE_BTN					GPIO_PIN_7 //test


/*BTN PORTS*/
//Front
#define SERVO_PORT					    GPIOB
#define HORN_BTN_PORT					GPIOB
#define BEAM_BTN_PORT					GPIOC


//Common
#define HAZARD_BTN_PORT					GPIOC
//#define HAZARD_BTN_PORT					GPIOB
//#define BLINKER_R_BTN_PORT				GPIOC
#define BLINKER_R_BTN_PORT				GPIOA
#define BLINKER_L_BTN_PORT				GPIOB

//Back
#define REVERSE_BTN_PORT				GPIOA


/* Front-side Lighting/Horn LIGHT PINS*/
#define FRONT_HORN_PIN             	GPIO_PIN_2
#define FRONT_BEAM_PIN             	GPIO_PIN_6
#define FRONT_BLINKER_L_PIN			GPIO_PIN_8
#define FRONT_BLINKER_R_PIN  		GPIO_PIN_4

//Front relay: PA8

/* Front-side Lighting/Horn LIGHT PORTS*/
#define FRONT_HORN_PORT				GPIOA
#define FRONT_BEAM_PORT				GPIOC
#define FRONT_BLINKER_L_PORT		GPIOC
#define FRONT_BLINKER_R_PORT	 	GPIOC


/* Back-side Lighting LIGHT PINS*/
#define BACK_BLINKER_R_PIN		    GPIO_PIN_0
#define BACK_BLINKER_L_PIN          GPIO_PIN_2
#define BACK_REVERSE_PIN    		GPIO_PIN_0
#define BACK_BRAKE_PIN				GPIO_PIN_0


/* Back-side Lighting LIGHT PORTS*/
#define BACK_BLINKER_R_PORT	        GPIOC
#define BACK_BLINKER_L_PORT         GPIOC
#define BACK_REVERSE_PORT   		GPIOB
#define BACK_BRAKE_PORT	     		GPIOA



#define LIMIT_SWITCH_BTN			GPIO_PIN_8
#define LIMIT_SWITCH_BTN_PORT  	 	GPIOA



#define MIN_BRAKE_VAL 200

#define INIT_COUNTER_PERIOD 		250 // time in ms between each check on max_throttle/brake values
#define BLINKER_PERIOD				1000	// time in ms

//#define SERVO_POSITION_0			0	/* in degrees */
//#define SERVO_POSITION_1			45
//#define SERVO_POSITION_2			90
//#define SERVO_POSITION_3			150

#define SERVO_MOTOR_PERIOD			25	// in milli sec
//#define SERVOMOTOR_CONST			1

#define ARR_SIZE 					75
#define PEAK_VALUE 					150
#define PI 							3.14159
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
