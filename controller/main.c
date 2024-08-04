/*
 * main.c
 *
 *  Created on: 4 Aug 2024
 *      Author: Sicris
 */
#include "DSP2833x_Device.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"


extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;
extern unsigned int EbssStart;
extern unsigned int EbssEnd;


static void configure_core_pll(uint16_t val)
{
    /* Make sure the PLL is not running in limp mode */
    if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
    {
       /*
        * Missing external clock has been detected
        * Replace this line with a call to an appropriate
        * SystemShutdown(); function.
        */
       asm("        ESTOP0");
    }

    /* Change the PLLCR */
    if (SysCtrlRegs.PLLCR.bit.DIV != val)
    {

       EALLOW;
       /* Before setting PLLCR turn off missing clock detect logic */
       SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
       SysCtrlRegs.PLLCR.bit.DIV = val;
       EDIS;

       /*
        * Wait for PLL to lock.
        * During this time the CPU will switch to OSCCLK/2 until
        * the PLL is stable.  Once the PLL is stable the CPU will
        *
        * Wait for the PLL lock bit to be set.
        */
       while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS == 0) {
           /*
            * Note: The watchdog should be fed within
            * the loop via ServiceDog().
            */
       }

       EALLOW;
       SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
       EDIS;
     }
}


#pragma CODE_SECTION(InitFlashWaitState, "ramfuncs");
static void InitFlashWaitState(void)
{
    EALLOW;

    //
    // Enable Flash Pipeline mode to improve performance
    // of code executed from Flash.
    //
    FlashRegs.FOPT.bit.ENPIPE = 1;

    //
    //                CAUTION
    // Minimum waitstates required for the flash operating
    // at a given CPU rate must be characterized by TI.
    // Refer to the datasheet for the latest information.
    //

    //
    // Set the Paged Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;

    //
    // Set the Random Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;

    //
    // Set the Waitstate for the OTP
    //
    FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;
    //
    //                CAUTION
    // ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
    //
    FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
    FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;

    EDIS;

    //
    // Force a pipeline flush to ensure that the write to
    // the last register configured occurs before returning.
    //
    asm(" RPT #7 || NOP");
}


interrupt void DefualtISR(void)
{
    asm ("      ESTOP0");
    for(;;);
}


void InitPieCtrl(void)
{
    //
    // Disable Interrupts at the CPU level
    //
    DINT;

    //
    // Disable the PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    //
    // Clear all PIEIER registers
    //
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    //
    // Clear all PIEIFR registers
    //
    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    volatile Uint32 *dest = (void *)&PieVectTable;
    EALLOW;
    for (uint16_t i = 0; i < 128; i++) {
        *dest++ = (Uint32)&DefualtISR;
    }
    EDIS;

    /* Enable the PIE Vector Table */
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
}


#define BLINKY_STACK_SIZE       (512)
static TaskHandle_t blinkyTaskHandle = NULL;
static StaticTask_t blinkyTaskStruct;
static StackType_t blinkyStack[BLINKY_STACK_SIZE];

static void blinkyTask(void * pvParam)
{
    TickType_t xLastWakeTime;

    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    EDIS;

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        /* ON LED */
        GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
        vTaskDelayUntil(&xLastWakeTime, 500);
        /* OFF LED */
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;
        vTaskDelayUntil(&xLastWakeTime, 1500);
    }
}


void main(void)
{
    InitPieCtrl();

    /*
     * Zero ebss section
     */
    memset(&EbssStart, 0, &EbssEnd - &EbssStart);

    /*
     * Copy ramfuncs section
     */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
            &RamfuncsLoadEnd - &RamfuncsLoadStart);

    InitFlashWaitState();

    /* Configure Core Frequency */
    configure_core_pll(0xA);

    blinkyTaskHandle = xTaskCreateStatic(blinkyTask,
                                         "blinky",
                                         BLINKY_STACK_SIZE,
                                         (void *)0,
                                         1,
                                         blinkyStack,
                                         &blinkyTaskStruct);
    vTaskStartScheduler();

    while(1);
}


