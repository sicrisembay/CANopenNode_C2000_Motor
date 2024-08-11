/*
 * adc.c
 *
 *  Created on: 11 Aug 2024
 *      Author: Sicris
 */

#include "autoconf_post.h"

#if CONFIG_USE_ADC
#include "stdbool.h"
#include "stdint.h"
#include "stddef.h"
#include "FreeRTOS.h"
#include "DSP2833x_Device.h"
#include "adc.h"

void ADC_init(ADC_RESULT_T * pAdcResult)
{
    configASSERT(pAdcResult != NULL);

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
    EDIS;
    AdcRegs.ADCTRL3.all = 0x00E0;

    /***** ADC Sampling Sequencer ********************************************/
#if (CONFIG_ADC_IA >= 0)
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = CONFIG_ADC_IA;
#else
  #error "Invalid ADC channel"
#endif
#if (CONFIG_ADC_IB >= 0)
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = CONFIG_ADC_IB;
#else
  #error "Invalid ADC channel"
#endif
#if (CONFIG_ADC_IC >= 0)
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = CONFIG_ADC_IC;
#else
  #error "Invalid ADC channel"
#endif
#if (CONFIG_ADC_VBUS >= 0)
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = CONFIG_ADC_VBUS;
#else
  #error "Invalid ADC channel"
#endif
#if (CONFIG_ADC_VA >= 0)
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = CONFIG_ADC_VA;
#else
  #error "Invalid ADC channel"
#endif
#if (CONFIG_ADC_VB >= 0)
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = CONFIG_ADC_VB;
#else
  #error "Invalid ADC channel"
#endif
#if (CONFIG_ADC_VC >= 0)
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = CONFIG_ADC_VC;
#else
  #error "Invalid ADC channel"
#endif
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 6;
    pAdcResult->pAdcResult_IphA = &(AdcMirror.ADCRESULT0);
    pAdcResult->pAdcResult_IphB = &(AdcMirror.ADCRESULT1);
    pAdcResult->pAdcResult_IphC = &(AdcMirror.ADCRESULT2);
    pAdcResult->pAdcResult_Vbus = &(AdcMirror.ADCRESULT3);
    pAdcResult->pAdcResult_VphA = &(AdcMirror.ADCRESULT4);
    pAdcResult->pAdcResult_VphB = &(AdcMirror.ADCRESULT5);
    pAdcResult->pAdcResult_VphC = &(AdcMirror.ADCRESULT6);
    /***** Dual Sequencer Mode ***********************************************/
    AdcRegs.ADCTRL1.all = 0x2880;
    AdcRegs.ADCTRL2.all = 0x4100;
    /***** Sequential Sampling mode ******************************************/
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0x0;
}

#endif /* CONFIG_USE_ADC */
