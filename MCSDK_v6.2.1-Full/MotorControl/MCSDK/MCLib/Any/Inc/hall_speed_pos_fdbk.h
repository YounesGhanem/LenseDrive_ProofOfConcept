/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Encoder Speed & Position Feedback component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup Encoder
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HALL_SPEEDNPOSFDBK_H
#define HALL_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"


#define HALL_SPEED_FIFO_SIZE  ((uint8_t)18)
#define HALL_ADC_SIZE    ((uint8_t)8)
/* HALL SENSORS PLACEMENT ----------------------------------------------------*/
#define DEGREES_120 0u
#define DEGREES_60 1u
#define HALL_SIZE   3u


typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  /* SW Settings */
  uint8_t  SensorPlacement; /*!< Define here the mechanical position of the sensors
                             with reference to an electrical cycle.
                             Allowed values are: DEGREES_120 or DEGREES_60.*/

  int16_t  PhaseShift;  /*!< Define here in s16degree the electrical phase shift
                             between the low to high transition of signal H1 and
                             the maximum of the Bemf induced on phase A.*/

  uint16_t SpeedSamplingFreqHz; /*!< Frequency (Hz) at which motor speed is to
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeedUnit
                             is called.*/

  uint8_t  SpeedBufferSize; /*!< Size of the buffer used to calculate the average
                             speed. It must be less than 18.*/

  /* HW Settings */
  uint32_t TIMClockFreq; /*!< Timer clock frequency express in Hz.*/

  TIM_TypeDef *TIMx;    /*!< Timer used for HALL sensor management.*/

  GPIO_TypeDef *H1Port;
  /*!< HALL sensor H1 channel GPIO input port (if used,
       after re-mapping). It must be GPIOx x= A, B, ...*/

  uint32_t  H1Pin;      /*!< HALL sensor H1 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  GPIO_TypeDef *H2Port;
  /*!< HALL sensor H2 channel GPIO input port (if used,
       after re-mapping). It must be GPIOx x= A, B, ...*/

  uint32_t  H2Pin;      /*!< HALL sensor H2 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  GPIO_TypeDef *H3Port;
  /*!< HALL sensor H3 channel GPIO input port (if used,
       after re-mapping). It must be GPIOx x= A, B, ...*/

  uint32_t H3Pin;      /*!< HALL sensor H3 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  bool SensorIsReliable;            /*!< Flag to indicate a wrong configuration
                                         of the Hall sensor signanls.*/

  volatile bool RatioDec;           /*!< Flag to avoid consecutive prescaler
                                         decrement.*/
  volatile bool RatioInc;           /*!< Flag to avoid consecutive prescaler
                                         increment.*/

  uint32_t rawAdc[HALL_ADC_SIZE];   /*!< Raw adc values of Hall sensors
                                        */
  int8_t Direction;          /*!< Instantaneous direction of rotor between two
                               captures*/

  int16_t AvrElSpeedDpp; /*!< It is the averaged rotor electrical speed express
                               in s16degree per current control period.*/

  int16_t CompSpeed;     /*!< Speed compensation factor used to syncronize
                               the current electrical angle with the target
                               electrical angle. */


  uint16_t SatSpeed;     /*!< Returned value if the measured speed is above the
                             maximum realistic.*/

  uint16_t PWMNbrPSamplingFreq; /*!< Number of current control periods inside
                             each speed control periods it will be:
                             (hMeasurementFrequency / hSpeedSamplingFreqHz) - 1.*/
  uint32_t rawAdcValues[HALL_ADC_SIZE];

  uint16_t el_angle ;    /* Electrical angle*/

  int16_t mech_Angle; 

  //SpeednTorqCtrl_Handle_t *pSTC;
  //VirtualSpeedSensor_Handle_t *pVSS;
                                                     
} HALL_Handle_t;


void HALL_Init(HALL_Handle_t *pHandle);


void HALL_Clear(HALL_Handle_t *pHandle);


int16_t HALL_CalcAngle(HALL_Handle_t *pHandle);

bool HALL_CalcAvrgMecSpeedUnit(HALL_Handle_t *pHandle, int16_t *pMecSpeedUnit);


void HALL_SetMecAngle(HALL_Handle_t *pHandle, int16_t hMecAngle);

#endif