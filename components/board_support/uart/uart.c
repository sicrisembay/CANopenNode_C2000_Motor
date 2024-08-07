/*
 * uart.c
 *
 *  Created on: 6 Aug 2024
 *      Author: Sicris
 */

#include "autoconf_post.h"

#if CONFIG_USE_UART
#include "stdbool.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "uart.h"

#define CONCAT(x, y)        x##y
#define CONCAT_L1(x, y)     CONCAT(x, y)

#define SCI_CLKIN_FREQ_HZ   ((CONFIG_SYSTEM_FREQ_MHZ * 1000000U) / 4U)

#if (CONFIG_ENABLE_UARTA)
#if (CONFIG_UARTA_TX_GPIO29)
#define UARTA_TX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0; \
                            GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1
#elif (CONFIG_UARTA_TX_GPIO35)
#else
#error "Invalid UART-A Tx GPIO"
#endif
#if (CONFIG_UARTA_RX_GPIO28)
#define UARTA_RX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0; \
                            GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; \
                            GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1
#elif (CONFIG_UARTA_RX_GPIO36)
#else
#error "Invalid UART-A Rx GPIO"
#endif
#endif /* CONFIG_ENABLE_UARTA */

#if (CONFIG_ENABLE_UARTB)
#if (CONFIG_UARTB_TX_GPIO9)
#define UARTB_TX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0; \
                            GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2
#elif (CONFIG_UARTB_TX_GPIO14)
#define UARTB_TX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0; \
                            GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2
#elif (CONFIG_UARTB_TX_GPIO18)
#define UARTB_TX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0; \
                            GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2
#elif (CONFIG_UARTB_TX_GPIO22)
#define UARTB_TX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0; \
                            GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3
#else
#errro "Invalid UART-B Tx GPIO"
#endif /* CONFIG_UARTB_TX_GPIOxx */
#if (CONFIG_UARTB_RX_GPIO11)
#define UARTB_RX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0; \
                            GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3; \
                            GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2
#elif (CONFIG_UARTB_RX_GPIO15)
#define UARTB_RX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0; \
                            GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3; \
                            GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2
#elif (CONFIG_UARTB_RX_GPIO19)
#define UARTB_RX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0; \
                            GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; \
                            GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2
#elif (CONFIG_UARTB_RX_GPIO23)
#define UARTB_RX_CONFIG     GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0; \
                            GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3; \
                            GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3
#else
#errro "Invalid UART-B Rx GPIO"
#endif /* CONFIG_UARTB_RX_GPIOxx */
#endif /* CONFIG_ENABLE_UARTB */

#if (CONFIG_ENABLE_UARTC)
#define UARTC_TX_CONFIG     GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; \
                            GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1
#define UARTC_RX_CONFIG     GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0; \
                            GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3; \
                            GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1
#endif /* CONFIG_ENABLE_UARTC */

static uint16_t const DEFAULT_UART_BRR[N_UART] = {
#if (CONFIG_ENABLE_UARTA)
    ((SCI_CLKIN_FREQ_HZ / (CONFIG_BAUDRATE_UARTA * 8)) - 1),
#endif
#if (CONFIG_ENABLE_UARTB)
    ((SCI_CLKIN_FREQ_HZ / (CONFIG_BAUDRATE_UARTB * 8)) - 1),
#endif
#if (CONFIG_ENABLE_UARTC)
    ((SCI_CLKIN_FREQ_HZ / (CONFIG_BAUDRATE_UARTC * 8)) - 1),
#endif
};

volatile struct SCI_REGS * SCI_REG_PTR[N_UART] = {
#if (CONFIG_ENABLE_UARTA)
    &SciaRegs,
#endif
#if (CONFIG_ENABLE_UARTB)
    &ScibRegs,
#endif
#if (CONFIG_ENABLE_UARTC)
    &ScicRegs,
#endif
};

static bool bInit = false;
static StreamBufferHandle_t txStreamBufHandle[N_UART];
static StaticStreamBuffer_t txStreamBufStruct[N_UART];
static uint8_t txStreamBufSto[N_UART][CONFIG_TX_QUEUE_BUFF_SZ + 1];
static StreamBufferHandle_t rxStreamBufHandle[N_UART];
static StaticStreamBuffer_t rxStreamBufStruct[N_UART];
static uint8_t rxStreamBufSto[N_UART][CONFIG_RX_QUEUE_BUFF_SZ + 1];


#pragma CODE_SECTION(txIsrHandler, "ramfuncs")
static BaseType_t txIsrHandler(UART_ID_T id)
{
    uint8_t data[UART_FIFO_SZ];
    size_t count;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t fifoTxCnt = SCI_REG_PTR[id]->SCIFFTX.bit.TXFFST;
    uint16_t fifoTxFree = UART_FIFO_SZ - fifoTxCnt;

    count = xStreamBufferReceiveFromISR(txStreamBufHandle[id],
                                        (void *)data,
                                        fifoTxFree,
                                        &xHigherPriorityTaskWoken);
    if((count == 0) && (fifoTxCnt == 0)) {
        /* Nothing to transmit.  Disable TX FIFO interrupt */
        SCI_REG_PTR[id]->SCIFFTX.bit.TXFFIENA = 0;
    } else {
        if(count > 0) {
            for(uint16_t i = 0; i < count; i++) {
                SCI_REG_PTR[id]->SCITXBUF = data[i] & 0x00FF;
            }
        }
    }

    return xHigherPriorityTaskWoken;
}

#pragma CODE_SECTION(rxIsrHandler, "ramfuncs")
static BaseType_t rxIsrHandler(UART_ID_T id)
{
    uint8_t data[UART_FIFO_SZ];
    size_t nWritten;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t rxCount = SCI_REG_PTR[id]->SCIFFRX.bit.RXFFST;

    if(rxCount > 0) {
        for(uint16_t i = 0; i < rxCount; i++) {
            data[i] = SCI_REG_PTR[id]->SCIRXBUF.bit.RXDT;
        }
        nWritten = xStreamBufferSendFromISR(rxStreamBufHandle[id],
                                            (void *)data,
                                            rxCount,
                                            &xHigherPriorityTaskWoken);
        if(nWritten != rxCount) {
            /* Was not able to write all bytes to stream buffer */
            /// TODO: Handle this.
        }
    }

    return xHigherPriorityTaskWoken;
}

#if CONFIG_ENABLE_UARTA
#pragma CODE_SECTION(UARTA_tx_isr, "ramfuncs")
interrupt void UARTA_tx_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    BaseType_t xHigherPriorityTaskWoken = txIsrHandler(UART_A);
    /* Clear Interrupt Flag */
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


#pragma CODE_SECTION(UARTA_rx_isr, "ramfuncs")
interrupt void UARTA_rx_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    BaseType_t xHigherPriorityTaskWoken = rxIsrHandler(UART_A);
    /* Clear Interrupt Flag */
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif /* CONFIG_ENABLE_UARTA */

#if CONFIG_ENABLE_UARTB
#pragma CODE_SECTION(UARTB_tx_isr, "ramfuncs")
interrupt void UARTB_tx_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    BaseType_t xHigherPriorityTaskWoken = txIsrHandler(UART_B);
    /* Clear Interrupt Flag */
    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#pragma CODE_SECTION(UARTB_rx_isr, "ramfuncs")
interrupt void UARTB_rx_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    BaseType_t xHigherPriorityTaskWoken = rxIsrHandler(UART_B);
    /* Clear Interrupt Flag */
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif /* CONFIG_ENABLE_UARTB */

#if CONFIG_ENABLE_UARTC
#pragma CODE_SECTION(UARTC_tx_isr, "ramfuncs")
interrupt void UARTC_tx_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK8 = 1;
    BaseType_t xHigherPriorityTaskWoken = txIsrHandler(UART_C);
    /* Clear Interrupt Flag */
    ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#pragma CODE_SECTION(UARTC_rx_isr, "ramfuncs")
interrupt void UARTC_rx_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK8 = 1;
    BaseType_t xHigherPriorityTaskWoken = rxIsrHandler(UART_C);
    /* Clear Interrupt Flag */
    ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif /* CONFIG_ENABLE_UARTC */


void UART_init(void)
{
    if(bInit != true) {
#if CONFIG_ENABLE_UARTA
        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
        UARTA_TX_CONFIG;
        UARTA_RX_CONFIG;
        EDIS;
#endif /* CONFIG_ENABLE_UARTA */
#if CONFIG_ENABLE_UARTB
        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 1;
        UARTB_TX_CONFIG;
        UARTB_RX_CONFIG;
        EDIS;
#endif /* CONFIG_ENABLE_UARTB */
#if CONFIG_ENABLE_UARTC
        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.SCICENCLK = 1;
        UARTC_TX_CONFIG;
        UARTC_RX_CONFIG;
        EDIS;
#endif /* CONFIG_ENABLE_UARTC */

        for(uint16_t i = 0; i < N_UART; i++) {
            /*
             * Stream Buffer Initialization
             */
            txStreamBufHandle[i] = xStreamBufferCreateStatic(CONFIG_TX_QUEUE_BUFF_SZ,
                                                             1,
                                                             &(txStreamBufSto[i][0]),
                                                             &(txStreamBufStruct[i]));
            configASSERT(txStreamBufHandle[i]);
            rxStreamBufHandle[i] = xStreamBufferCreateStatic(CONFIG_RX_QUEUE_BUFF_SZ,
                                                             1,
                                                             &(rxStreamBufSto[i][0]),
                                                             &(rxStreamBufStruct[i]));
            configASSERT(rxStreamBufHandle[i]);
            /*
             * SCI and FIFO Initialization
             */
            SCI_REG_PTR[i]->SCICCR.all = 0x0007;
            SCI_REG_PTR[i]->SCICTL1.all = 0x0003;
            SCI_REG_PTR[i]->SCICTL2.bit.TXINTENA = 1;
            SCI_REG_PTR[i]->SCICTL2.bit.RXBKINTENA = 1;
            SCI_REG_PTR[i]->SCIHBAUD = (DEFAULT_UART_BRR[i] >> 8) & 0x00FF;
            SCI_REG_PTR[i]->SCILBAUD = DEFAULT_UART_BRR[i] & 0x00FF;
            SCI_REG_PTR[i]->SCIFFTX.all = 0xC000;
            SCI_REG_PTR[i]->SCIFFRX.all = 0x0021;
            SCI_REG_PTR[i]->SCIFFCT.all = 0x0000;
            /* Release from reset and Enable FIFO */
            SCI_REG_PTR[i]->SCICTL1.bit.SWRESET = 1;
            SCI_REG_PTR[i]->SCIFFTX.bit.TXFIFOXRESET = 1;
            SCI_REG_PTR[i]->SCIFFRX.bit.RXFIFORESET = 1;
        }

#if CONFIG_ENABLE_UARTA
        EALLOW;
        PieVectTable.SCITXINTA = UARTA_tx_isr;
        PieVectTable.SCIRXINTA = UARTA_rx_isr;
        EDIS;
        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
        IER |= M_INT9;
#endif /* CONFIG_ENABLE_UARTA */
#if CONFIG_ENABLE_UARTB
        EALLOW;
        PieVectTable.SCITXINTB = UARTB_tx_isr;
        PieVectTable.SCIRXINTB = UARTB_rx_isr;
        EDIS;
        PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx4 = 1;
        IER |= M_INT9;
#endif /* CONFIG_ENABLE_UARTB */
#if CONFIG_ENABLE_UARTC
        EALLOW;
        PieVectTable.SCITXINTC = UARTC_tx_isr;
        PieVectTable.SCIRXINTC = UARTC_rx_isr;
        EDIS;
        PieCtrlRegs.PIEIER8.bit.INTx5 = 1;
        PieCtrlRegs.PIEIER8.bit.INTx6 = 1;
        IER |= M_INT8;
#endif /* CONFIG_ENABLE_UARTC */

        bInit = true;
    }
}


bool UART_init_done(void)
{
    return bInit;
}


bool UART_txBufferEmpty(const UART_ID_T id)
{
    bool retval = false;
    if(id < N_UART) {
        retval = (SCI_REG_PTR[id]->SCIFFTX.bit.TXFFST == 0) && xStreamBufferIsEmpty(txStreamBufHandle[id]);
    }
    return retval;
}


static uint16_t Send(const UART_ID_T id, const uint8_t * pBuf, const uint16_t count, const bool bFromISR)
{
    uint16_t j = 0;
    bool fifoFirst = false;
    uint16_t fifoStatus;
    uint16_t bytesSent = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if((pBuf == NULL) || (count == 0) || (bInit != true) ||
       (id >= N_UART)) {
        return 0;
    }

    /*
     * Disable Tx interrupt to prevent race condition while
     * setting up the buffer
     */
    SCI_REG_PTR[id]->SCIFFTX.bit.TXFFIENA = 0;

    fifoStatus = SCI_REG_PTR[id]->SCIFFTX.bit.TXFFST;
    if((fifoStatus == 0) && (xStreamBufferIsEmpty(txStreamBufHandle[id]))) {
        /* fill uart FIFO first before the stream buffer */
        fifoFirst = true;
        j = 0;
    }

    for(bytesSent = 0; bytesSent < count; bytesSent++) {
        if(fifoFirst && (j < UART_FIFO_SZ)) {
            SCI_REG_PTR[id]->SCITXBUF = pBuf[bytesSent];
            j++;
        } else {
            if(bFromISR) {
                if(1 != xStreamBufferSendFromISR(txStreamBufHandle[id], &(pBuf[bytesSent]), 1, &xHigherPriorityTaskWoken)) {
                    break;
                }
            } else {
                if(1 != xStreamBufferSend(txStreamBufHandle[id], &(pBuf[bytesSent]), 1, 0)) {
                    break;
                }
            }
        }
    }
    /* Enable Tx interrupt */
    SCI_REG_PTR[id]->SCIFFTX.bit.TXFFIENA = 1;

    if(bFromISR) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    return bytesSent;
}


static uint16_t Receive(const UART_ID_T id, uint8_t * const pBuf, const uint16_t size, const bool bFromISR, const TickType_t xTicksToWait)
{
    size_t count = 0;

    if((id >= N_UART) || (pBuf == NULL) || (size == 0)) {
        return 0;
    }

    if(bFromISR) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        count = xStreamBufferReceiveFromISR(rxStreamBufHandle[id],
                                            (void *)pBuf,
                                            size,
                                            &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        count = xStreamBufferReceive(rxStreamBufHandle[id], (void *)pBuf, size, xTicksToWait);
    }

    return count;
}

uint16_t UART_send(const UART_ID_T id, const uint8_t * pBuf, const uint16_t count)
{
    return Send(id, pBuf, count, false);
}


uint16_t UART_sendFromISR(const UART_ID_T id, const uint8_t * pBuf, const uint16_t count)
{
    return Send(id, pBuf, count, true);
}


uint16_t UART_receive(const UART_ID_T id, uint8_t * const pBuf, const uint16_t count, const TickType_t xTicksToWait)
{
    return Receive(id, pBuf, count, false, xTicksToWait);
}


uint16_t UART_receiveFromISR(const UART_ID_T id, uint8_t * const pBuf, const uint16_t count)
{
    return Receive(id, pBuf, count, true, 0);
}
#endif /* CONFIG_USE_UART */

