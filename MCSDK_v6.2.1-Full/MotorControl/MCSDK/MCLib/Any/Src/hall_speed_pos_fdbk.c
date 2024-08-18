/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Encoder component of the Motor Control SDK:
  *           - computes and stores average mechanical speed
  *           - computes and stores average mechanical acceleration
  *           - computes and stores  the instantaneous electrical speed
  *           - calculates the rotor electrical and mechanical angle
  *
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

/* Includes ------------------------------------------------------------------*/
#include "hall_speed_pos_fdbk.h"
#include "mc_type.h"
#include "trajectory_ctrl.h"
#include "mc_math.h"
#include "math.h"


__weak void HALL_Init(HALL_Handle_t *pHandle)
{
  #ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
  if (NULL == pHandler)
  {
    /* Nothing to do */
  }
  else
  {
  #endif
    // Initialize the raw ADC array to zeros
    for (int i = 0; i < HALL_ADC_SIZE; i++)
    {
      pHandle->rawAdc[i] = 0;
    }

  #ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
  }
  #endif
}

/**
  * @brief  It set instantaneous rotor mechanical angle.
  *         As a consequence, timer counter is computed and updated.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  hMecAngle new value of rotor mechanical angle in [s16degree](measurement_units.md) format.
  */
 //TODO : Review this function
__weak void HALL_SetMecAngle(HALL_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    TIM_TypeDef *TIMx = pHandle->TIMx;

    uint16_t hAngleCounts;
    uint16_t hMecAngleuint;
    int16_t localhMecAngle = hMecAngle;

    pHandle->_Super.hMecAngle = localhMecAngle;
    pHandle->_Super.hElAngle = localhMecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
    if (localhMecAngle < 0)
    {
      localhMecAngle *= -1;
      hMecAngleuint = ((uint16_t)65535 - ((uint16_t)localhMecAngle));
    }
    else
    {
      hMecAngleuint = (uint16_t)localhMecAngle;
    }

    //TODO
    //hAngleCounts = (uint16_t)((((uint32_t)hMecAngleuint) * ((uint32_t)pHandle->PulseNumber)) / 65535U);

    TIMx->CNT = (uint16_t)hAngleCounts;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}
__weak int16_t HALL_CalcAngle(HALL_Handle_t *pHandle)
{
  int16_t retVal = 0;
  if (LL_ADC_IsActiveFlag_EOS(ADC2))
  {
    //adc data available
    LL_ADC_ClearFlag_EOS(ADC2);  
     pHandle->rawAdcValues[0] = LL_ADC_REG_ReadConversionData12(ADC2);  //PC0
     pHandle->rawAdcValues[1] = LL_ADC_REG_ReadConversionData12(ADC2);  //PC1
     pHandle->rawAdcValues[2] = LL_ADC_REG_ReadConversionData12(ADC2);  //PC2

    // Apply Clarke transformation
    float alpha =  pHandle->rawAdcValues[0] - 0.5f * (pHandle->rawAdcValues[1] + pHandle->rawAdcValues[2]);
    float beta  = (float)(MCM_Sqrt(3) / 2) * (pHandle->rawAdcValues[1] - pHandle->rawAdcValues[2]);

    // Calculate the electrical angle theta
    float theta = atan2(beta, alpha);

    float angle_degree = (int16_t)(theta * (180.0 / M_PI));  

    // Convert theta from radians to degrees, if necessary
    pHandle->el_angle = (int16_t)(theta * (180.0 / M_PI));  //elangle = 2 X mec angle
    pHandle->mech_Angle = pHandle->el_angle /2;
    
    
    retVal = angle_degree;
    

  }
  else
  {
    // no adc data available
    
  }

  return retVal;
  
}