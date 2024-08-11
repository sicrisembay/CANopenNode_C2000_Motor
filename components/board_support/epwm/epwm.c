/*
 * epwm.c
 *
 *  Created on: 11 Aug 2024
 *      Author: Sicris
 */

#include "autoconf_post.h"

#if CONFIG_USE_EPWM
#include "stdbool.h"
#include "stdint.h"
#include "stddef.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_EPwm_defines.h"
#include "epwm.h"

#define PWMGEN_PERIODMAX        (uint16_t)((CONFIG_SYSTEM_FREQ_MHZ * 1000000U * \
                                  CONFIG_FLOAT_VALUE_PWM_INT_PERIOD) / (2 * CONFIG_PWM_FREQ_PRESCALE))
#define PWMGEN_DEADBAND(x)      (uint16_t)(((uint32_t)CONFIG_SYSTEM_FREQ_MHZ * (uint32_t) (x)) / 1000)
#define PWMGEN_HALFPERIOD       (PWMGEN_PERIODMAX / 2)

static bool bInit = false;
static epwm_isr_cb isrCallback = NULL;

#pragma CODE_SECTION(EPWM_isr, "ramfuncs")
interrupt void EPWM_isr(void)
{
    if(isrCallback != NULL) {
        isrCallback();
    }
    /* Clear Interrupt Flag */
    EPwm1Regs.ETCLR.bit.INT = 1;
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK3 = 1;
}
void EPWM_init(epwm_isr_cb cb)
{
    if(bInit != true) {
        isrCallback = cb;

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the ePWM
        SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
        SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
        SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
        EDIS;

        EPwm1Regs.CMPA.half.CMPA = 0xFFFF;
        EPwm2Regs.CMPA.half.CMPA = 0xFFFF;
        EPwm3Regs.CMPA.half.CMPA = 0xFFFF;
        /***** TBCLK *********************************************************/
        /* Channel 1 */
        EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
        EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
        EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
        EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
        EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;
        /* Channel 2 */
        EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
        EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
        EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
        EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
        EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;
        /* Channel 3 */
        EPwm3Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
        EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
        EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
        EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
        EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
        /***** Setup shadow register load on ZERO ****************************/
        /* Channel 1 */
        EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
        EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
        EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
        /* Channel 2 */
        EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
        EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
        EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
        /* Channel 3 */
        EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
        EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
        EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
        /***** Set actions ***************************************************/
        /* Channel 1 */
        EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
        EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;
        EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;
        /* Channel 2 */
        EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
        EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;
        EPwm2Regs.AQCTLB.bit.CAD = AQ_CLEAR;
        /* Channel 3 */
        EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
        EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;
        EPwm3Regs.AQCTLB.bit.CAD = AQ_CLEAR;
        /***** Dead-Band Control *********************************************/
        /* Channel 1 */
        EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
        EPwm1Regs.DBFED = PWMGEN_DEADBAND(CONFIG_PWM_DEADBAND);
        EPwm1Regs.DBRED = PWMGEN_DEADBAND(CONFIG_PWM_DEADBAND);
        /* Channel 2 */
        EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
        EPwm2Regs.DBFED = PWMGEN_DEADBAND(CONFIG_PWM_DEADBAND);
        EPwm2Regs.DBRED = PWMGEN_DEADBAND(CONFIG_PWM_DEADBAND);
        /* Channel 3 */
        EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
        EPwm3Regs.DBFED = PWMGEN_DEADBAND(CONFIG_PWM_DEADBAND);
        EPwm3Regs.DBRED = PWMGEN_DEADBAND(CONFIG_PWM_DEADBAND);
        /***** Reset Counter and Load Period *********************************/
        EPwm1Regs.TBCTR = 0;
        EPwm2Regs.TBCTR = 0;
        EPwm3Regs.TBCTR = 0;
        EPwm1Regs.TBPRD = PWMGEN_PERIODMAX;
        EPwm2Regs.TBPRD = PWMGEN_PERIODMAX;
        EPwm3Regs.TBPRD = PWMGEN_PERIODMAX;
        /***** Disable Trip Zone *********************************************/
        EALLOW;
        EPwm1Regs.TZSEL.all = 0;
        EPwm2Regs.TZSEL.all = 0;
        EPwm3Regs.TZSEL.all = 0;
        EPwm1Regs.TZCTL.all = 0;
        EPwm2Regs.TZCTL.all = 0;
        EPwm3Regs.TZCTL.all = 0;
        EDIS;
        /***** Synchronization ***********************************************/
        EPwm1Regs.TBPHS.half.TBPHS = 0;
        EPwm2Regs.TBPHS.half.TBPHS = 0;
        EPwm3Regs.TBPHS.half.TBPHS = 0;
        /* Sync: ePWM1 (master) --> ePWM2,3 (slaves) */
        EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
        EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
        EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
        EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
        EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
        EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
        EDIS;
        /***** Interrupt Generation ******************************************/
        EALLOW;
        PieVectTable.EPWM1_INT = EPWM_isr;
        EDIS;
        EPwm1Regs.ETSEL.bit.INTEN = 1;
        EPwm1Regs.ETCLR.bit.INT = 1;
        EPwm1Regs.ETSEL.bit.INTSEL = 1;           /* Enable interrupt on count Zero event */
        EPwm1Regs.ETSEL.bit.SOCAEN = 1;           /* Enable SOC on A group */
        EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; /* Select SOC from Timebase counter equal to period */
        EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;      /* Generate pulse on 1st event */
#if (CONFIG_PWM_FREQUENCY_10KHZ)
        EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;       /* Generate interrupt on 1st event */
#elif (CONFIG_PWM_FREQUENCY_20KHZ)
        EPwm1Regs.ETPS.bit.INTPRD = ET_2ND;       /* Generate interrupt on 2nd event */
#elif (CONFIG_PWM_FREQUENCY_30KHZ)
        EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;       /* Generate interrupt on 3rd event */
#endif /* CONFIG_PWM_FREQUENCY_XXKHZ */
        /***** GPIO **********************************************************/
        EALLOW;
        GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0x01;
        GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0x01;
        GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0x01;
        GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0x01;
        GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0x01;
        GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0x01;
        EDIS;
        /***** Enable Interrupt **********************************************/
        PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
        IER |= M_INT3;

        bInit = true;
    }
}


#pragma CODE_SECTION(EPWM_setDutyCycle, "ramfuncs")
void EPWM_setDutyCycle(float phA, float phB, float phC)
{
    EPwm1Regs.CMPA.half.CMPA = (phA * PWMGEN_HALFPERIOD) + PWMGEN_HALFPERIOD;
    EPwm2Regs.CMPA.half.CMPA = (phB * PWMGEN_HALFPERIOD) + PWMGEN_HALFPERIOD;
    EPwm3Regs.CMPA.half.CMPA = (phC * PWMGEN_HALFPERIOD) + PWMGEN_HALFPERIOD;
}

#endif /* CONFIG_USE_EPWM */

