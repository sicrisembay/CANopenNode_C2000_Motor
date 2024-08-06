/*
 * hooks.c
 *
 *  Created on: 4 Aug 2024
 *      Author: Sicris
 */
#include "autoconf_post.h"
#include "qpc.h"

uint16_t const l_TickHook = 0;

#if CONFIG_QPC_QSPY_ENABLE
static uint8_t qsTxBuf[CONFIG_QSPY_TX_BUFFER_SIZE];
static uint8_t qsRxBuf[CONFIG_QSPY_RX_BUFFER_SIZE];
#endif /* CONFIG_QPC_QSPY_ENABLE */

void vApplicationTickHook(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    QTIMEEVT_TICK_FROM_ISR(0U, &xHigherPriorityTaskWoken, &l_TickHook);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void vApplicationIdleHook(void)
{
}


void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /*
     * If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits.
     */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /*
     * Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored.
     */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /*
     * Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes.
     */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}


void QF_onStartup(void)
{
}


void QF_onCleanup(void)
{
}


void Q_onAssert(char const *module, int loc)
{
    while(1) {
        /// TODO
    }
}


#if CONFIG_QPC_QSPY_ENABLE
#define QSPY_RX_NOTIFY_TIMEOUT_MS   (2)
#define QSPY_RX_NOTIFY_TIMEOUT      (QSPY_RX_NOTIFY_TIMEOUT_MS / portTICK_PERIOD_MS)

static StaticTask_t xQspyWorkerTCB;
static StackType_t xQspyWorkerStackSto[CONFIG_QSPY_WORKER_STACK_SIZE];
static TaskHandle_t xQspyWorkerTaskHandle = NULL;

static void QS_workerTask(void * pvParam)
{
    uint8_t const * pBlock = NULL;
    uint16_t txLen = 0;
    uint16_t rxLen = 0;
    uint8_t rxBuf[16] = {0};

    while(1) {
//        rxLen = BSP_UART_receive(rxBuf, sizeof(rxBuf), QSPY_RX_NOTIFY_TIMEOUT);
        if(rxLen > 0) {
            for(uint16_t i = 0; i < rxLen; i++) {
                QS_RX_PUT((rxBuf[i] & 0x00FF));
            }
            QS_rxParse();
        }
//        txLen = BSP_UART_TX_FIFO_SIZE;
        taskENTER_CRITICAL();
        pBlock = QS_getBlock(&txLen);
        taskEXIT_CRITICAL();
        if(txLen > 0) {
//            BSP_UART_send(pBlock, txLen);
        }
    }
}
uint8_t QS_onStartup(void const *arg)
{
    (void)arg;  // unused parameter
    if(NULL == xQspyWorkerTaskHandle) {
        QS_initBuf(qsTxBuf, sizeof(qsTxBuf));
        QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));
//        BSP_UART_init();
        xQspyWorkerTaskHandle = xTaskCreateStatic(
                QS_workerTask,
                "QSpyWorker",
                CONFIG_QSPY_WORKER_STACK_SIZE,
                (void *)0,
                CONFIG_QPSY_WORKER_TASK_PRIORITY,
                xQspyWorkerStackSto,
                &xQspyWorkerTCB);
    }
    return (uint8_t)1; /* return success */
}


void QS_onCleanup(void)
{
    /// TODO
}


QSTimeCtr QS_onGetTime(void)
{
    /* Use freeRTOS tick count */
    return(xTaskGetTickCount());
}


void QS_onFlush(void)
{
    /* QS buffer flushing to HW peripheral */
    uint8_t const * pBlock;
    uint16_t txLen;

    while(1) {
//        txLen = BSP_UART_TX_FIFO_SIZE;
        taskENTER_CRITICAL();
        pBlock = QS_getBlock(&txLen);
        taskEXIT_CRITICAL();
        if(txLen == 0) {
            break;
        }
//        (void)BSP_UART_send(pBlock, txLen);
//        while(!BSP_UART_sendBufferEmpty());
    }

}


void QS_onReset(void)
{
    /* Tickle dog */
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;

    /* Enable watchdog */
    EALLOW;
    SysCtrlRegs.WDCR = 0x0000;  /* writing value other than b101 to WDCHK will immediately resets the device */
    EDIS;

    /* Should not reach here */
    while(1);
}


void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;
}

#endif /* CONFIG_QPC_QSPY_ENABLE */
