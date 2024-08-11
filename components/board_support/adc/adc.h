/*
 * adc.h
 *
 *  Created on: 11 Aug 2024
 *      Author: Sicris
 */

#ifndef COMPONENTS_BOARD_SUPPORT_ADC_ADC_H_
#define COMPONENTS_BOARD_SUPPORT_ADC_ADC_H_

#include "autoconf_post.h"

#if CONFIG_USE_ADC
#include "stdint.h"

typedef struct {
    volatile uint16_t * pAdcResult_IphA;
    volatile uint16_t * pAdcResult_IphB;
    volatile uint16_t * pAdcResult_IphC;
    volatile uint16_t * pAdcResult_Vbus;
    volatile uint16_t * pAdcResult_VphA;
    volatile uint16_t * pAdcResult_VphB;
    volatile uint16_t * pAdcResult_VphC;
} ADC_RESULT_T;

void ADC_init(ADC_RESULT_T * pAdcResult);

#endif /* CONFIG_USE_ADC */
#endif /* COMPONENTS_BOARD_SUPPORT_ADC_ADC_H_ */
