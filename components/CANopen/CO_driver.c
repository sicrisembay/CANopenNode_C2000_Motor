/*!
 * @file        CO_driver.c
 * @author      Janez Paternoster
 * @author      Sicris Rey Embay
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "autoconf_post.h"
#if CONFIG_USE_CANOPEN

#include "CANopen.h"
#include "OD.h"
#include "301/CO_driver.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define CANOPEN_F82335_MAILBOX_CNT      (32)    // F28335 has 32 mailboxes in each CAN peripheral instance
#define CANOPEN_F28335_TX_MB_CNT        (1)     // 1 mailbox is allocated for transmission
#define CANOPEN_F28335_TX_MB_IDX        (31)    // mailbox31 is used for transmission
#define CANOPEN_F28335_TX_MB_MASK       (1UL << CANOPEN_F28335_TX_MB_IDX)
#define CANOPEN_F28335_RX_MB_CNT        (CANOPEN_F82335_MAILBOX_CNT - CANOPEN_F28335_TX_MB_CNT) // 31 mailboxes are allocated for reception

// Similar definition found in CANopen.c -->
// This is used to find the TX buffer index used by TPDOs.  This
// useful when clearing Pending Sync TPDO in TX buffer.
#if (CO_CONFIG_NMT) & CO_CONFIG_NMT_MASTER
 #define CO_TX_CNT_NMT_MST 1
#else
 #define CO_TX_CNT_NMT_MST 0
#endif

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_PRODUCER
  #define CO_TX_CNT_SYNC OD_CNT_SYNC
#else
  #define CO_TX_CNT_SYNC 0
#endif

#if (CO_CONFIG_EM) & CO_CONFIG_EM_PRODUCER
 #if OD_CNT_EM_PROD == 1
  #define CO_TX_CNT_EM_PROD OD_CNT_EM_PROD
 #else
  #error wrong OD_CNT_EM_PROD
 #endif
#else
 #define CO_TX_CNT_EM_PROD 0
#endif

#if (CO_CONFIG_TIME) & CO_CONFIG_TIME_PRODUCER
  #define CO_TX_CNT_TIME OD_CNT_TIME
#else
  #define CO_TX_CNT_TIME 0
#endif

#if (CO_CONFIG_GFC) & CO_CONFIG_GFC_ENABLE
 #define CO_TX_CNT_GFC OD_CNT_GFC
#else
 #define CO_TX_CNT_GFC 0
#endif

#if (CO_CONFIG_SRDO) & CO_CONFIG_SRDO_ENABLE
 #define CO_TX_CNT_SRDO OD_CNT_SRDO
#else
 #define CO_TX_CNT_SRDO 0
#endif

#define CANOPEN_TPDO_BUF_IDX    (CO_TX_CNT_NMT_MST + CO_TX_CNT_SYNC + OD_CNT_EM_PROD + CO_TX_CNT_TIME + \
                                 CO_TX_CNT_GFC + (CO_TX_CNT_SRDO * 2))
// <-- Similar definition found in CANopen.c

typedef struct {
    uint16_t bit_rate_kbps;
    uint16_t brp;
    uint16_t tseg1;
    uint16_t tseg2;
} BTC_CONFIG_ENTRY_T;


static BTC_CONFIG_ENTRY_T const BTC_CONFIG_TABLE[] = {
#if CONFIG_CANOPEN_BITRATE_20KBPS
    /* 20kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 20,
        .brp = 249,
#if CONFIG_CORE_FREQ_150MHZ
        .tseg1 = 10,
        .tseg2 = 2
#elif CONFIG_CORE_FREQ_100MHZ
        .tseg1 = 6,
        .tseg2 = 1
#else
#error "Invalid Core Freq for CANopen"
#endif
    },
#endif
#if CONFIG_CANOPEN_BITRATE_50KBPS
    /* 50kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 50,
        .brp = 99,
#if CONFIG_CORE_FREQ_150MHZ
        .tseg1 = 10,
        .tseg2 = 2
#elif CONFIG_CORE_FREQ_100MHZ
        .tseg1 = 6,
        .tseg2 = 1
#else
#error "Invalid Core Freq for CANopen"
#endif
    },
#endif
#if CONFIG_CANOPEN_BITRATE_125KBPS
    /* 125kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 125,
        .brp = 39,
#if CONFIG_CORE_FREQ_150MHZ
        .tseg1 = 10,
        .tseg2 = 2
#elif CONFIG_CORE_FREQ_100MHZ
        .tseg1 = 6,
        .tseg2 = 1
#else
#error "Invalid Core Freq for CANopen"
#endif
    },
#endif
#if CONFIG_CANOPEN_BITRATE_250KBPS
    /* 250kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 250,
        .brp = 19,
#if CONFIG_CORE_FREQ_150MHZ
        .tseg1 = 10,
        .tseg2 = 2
#elif CONFIG_CORE_FREQ_100MHZ
        .tseg1 = 6,
        .tseg2 = 1
#else
#error "Invalid Core Freq for CANopen"
#endif
    },
#endif
#if CONFIG_CANOPEN_BITRATE_500KBPS
    /* 500kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 500,
        .brp = 9,
#if CONFIG_CORE_FREQ_150MHZ
        .tseg1 = 10,
        .tseg2 = 2
#elif CONFIG_CORE_FREQ_100MHZ
        .tseg1 = 6,
        .tseg2 = 1
#else
#error "Invalid Core Freq for CANopen"
#endif
    },
#endif
#if CONFIG_CANOPEN_BITRATE_1000KBPS
    /* 1000kbps, sampling at 73.3% */
    {
        .bit_rate_kbps = 1000,
        .brp = 4,
#if CONFIG_CORE_FREQ_150MHZ
        .tseg1 = 10,
        .tseg2 = 2
#elif CONFIG_CORE_FREQ_100MHZ
        .tseg1 = 6,
        .tseg2 = 1
#else
#error "Invalid Core Freq for CANopen"
#endif
    },
#endif
};


#define N_BTC_CONFIG_ENTRY      (sizeof(BTC_CONFIG_TABLE) / sizeof(BTC_CONFIG_TABLE[0]))

volatile CO_CANtx_t * pCurrentCanTxObj = NULL;
static CO_CANmodule_t * pCANmodule = NULL;

#pragma CODE_SECTION(CAN_isrHandler, "ramfuncs")
static BaseType_t CAN_isrHandler(CO_CANmodule_t * CANmodule)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    union CANGIF0_REG shadow_cangif0;
    union CANTA_REG shadow_canta;
    union CANTRS_REG shadow_cantrs;
    union CANRMP_REG shadow_canrmp;
    union CANME_REG shadow_canme;
    union CANMSGID_REG shadow_canmsgid;
    union CANMSGCTRL_REG shadow_canmsgctrl;
    union CANMDL_REG shadow_canmdl;
    union CANMDH_REG shadow_canmdh;

    CO_CANrxMsg_t canRxMsg;

    configASSERT(CANmodule != NULL);

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);
    if(ECanRegPtr == &ECanaRegs) {
        MBoxPtr = &(ECanaMboxes.MBOX0);
    } else if(ECanRegPtr == &ECanbRegs) {
        MBoxPtr = &(ECanbMboxes.MBOX0);
    } else {
        configASSERT(false);  // This should not happen.
    }

    /* Clear Global interrupt flag */
    shadow_cangif0.all = ECanRegPtr->CANGIF0.all;
    if(shadow_cangif0.bit.RMLIF0) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRRX_OVERFLOW;
    }
    ECanRegPtr->CANGIF0.all = shadow_cangif0.all;

    /* receive interrupt */
    shadow_canrmp.all = ECanRegPtr->CANRMP.all & ECanRegPtr->CANMIM.all;

    if(shadow_canrmp.all != 0) {
        CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
        uint16_t index;             /* index of received message */
        uint32_t rcvMsgIdent;       /* identifier of the received message */
        CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        /* clear Rx pending flag */
        ECanRegPtr->CANRMP.all = shadow_canrmp.all;

        for(index = 0; index < CANmodule->rxSize; index++) {
            if(shadow_canrmp.all & (1UL << index)) {
                /* get message from module here */
                canRxMsg.ident = MBoxPtr[index].MSGID.bit.STDMSGID;
                canRxMsg.DLC = MBoxPtr[index].MSGCTRL.bit.DLC;
                canRxMsg.data[0] = MBoxPtr[index].MDL.byte.BYTE0;
                canRxMsg.data[1] = MBoxPtr[index].MDL.byte.BYTE1;
                canRxMsg.data[2] = MBoxPtr[index].MDL.byte.BYTE2;
                canRxMsg.data[3] = MBoxPtr[index].MDL.byte.BYTE3;
                canRxMsg.data[4] = MBoxPtr[index].MDH.byte.BYTE4;
                canRxMsg.data[5] = MBoxPtr[index].MDH.byte.BYTE5;
                canRxMsg.data[6] = MBoxPtr[index].MDH.byte.BYTE6;
                canRxMsg.data[7] = MBoxPtr[index].MDH.byte.BYTE7;

                rcvMsg = &canRxMsg;
                rcvMsgIdent = rcvMsg->ident;
                if(CANmodule->useCANrxFilters){
                    /* CAN module filters are used. Message with known 11-bit identifier has */
                    /* been received */
                    buffer = &CANmodule->rxArray[index];
                    /* verify also RTR */
                    if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                        msgMatched = true;
                    }
                }
                else{
                    /* CAN module filters are not used, message with any standard 11-bit identifier */
                    /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
                    buffer = &CANmodule->rxArray[0];
                    uint16_t i;
                    for(i = CANmodule->rxSize; i > 0U; i--){
                        if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                            msgMatched = true;
                            break;
                        }
                        buffer++;
                    }
                }

                /* Call specific function, which will process the message */
                if(msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)){
                    buffer->CANrx_callback(buffer->object, (void*) rcvMsg);
                }
            }
        }
    }

    /* transmit interrupt */
    shadow_cantrs.all = ECanRegPtr->CANTRS.all;
    shadow_canta.all = ECanRegPtr->CANTA.all & ECanRegPtr->CANMIM.all;
    if(shadow_canta.all != 0){
        // CO_LOCK_CAN_SEND(CANmodule);  Note: Can't use mutex in ISR

        CO_CANtx_t * pTxBuffer = NULL;

        /* Clear interrupt flag */
        ECanRegPtr->CANTA.all = shadow_canta.all;

        if(shadow_canta.all & CANOPEN_F28335_TX_MB_MASK) {
            uint16_t index = 0;
            CANmodule->firstCANtxMessage = false;
            if(pCurrentCanTxObj != NULL) {
                pCurrentCanTxObj->bufferInhibitFlag = false;
            }
            /* Check any pending Tx buffers */
            for(index = 0; index < CANmodule->txSize; index++) {
                pTxBuffer = &(CANmodule->txArray[index]);
                if(pTxBuffer->bufferFull) {
                    pTxBuffer->bufferFull = false;
                    pTxBuffer->bufferInhibitFlag = pTxBuffer->syncFlag;
                    /* CAN Send */
                    /* Disable mailbox to be able to modify MSGID */
                    shadow_canme.all = ECanRegPtr->CANME.all;
                    shadow_canme.all &= ~CANOPEN_F28335_TX_MB_MASK;
                    ECanRegPtr->CANME.all = shadow_canme.all;
                    /* Configure MSGID */
                    shadow_canmsgid.all = 0;
                    shadow_canmsgid.bit.STDMSGID = pTxBuffer->ident & 0x07FFU;
                    MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MSGID.all = shadow_canmsgid.all;
                    /* MSGCTRL */
                    shadow_canmsgctrl.all = 0;
                    shadow_canmsgctrl.bit.DLC = pTxBuffer->DLC;
                    if(pTxBuffer->ident & 0x0800U) {
                        shadow_canmsgctrl.bit.RTR = 1;
                    }
                    /* DATA */
                    shadow_canmdl.byte.BYTE0 = pTxBuffer->data[0];
                    shadow_canmdl.byte.BYTE1 = pTxBuffer->data[1];
                    shadow_canmdl.byte.BYTE2 = pTxBuffer->data[2];
                    shadow_canmdl.byte.BYTE3 = pTxBuffer->data[3];
                    shadow_canmdh.byte.BYTE4 = pTxBuffer->data[4];
                    shadow_canmdh.byte.BYTE5 = pTxBuffer->data[5];
                    shadow_canmdh.byte.BYTE6 = pTxBuffer->data[6];
                    shadow_canmdh.byte.BYTE7 = pTxBuffer->data[7];
                    MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MSGCTRL.all = shadow_canmsgctrl.all;
                    MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MDL.all = shadow_canmdl.all;
                    MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MDH.all = shadow_canmdh.all;

                    /* Re-enable mailbox */
                    shadow_canme.all = ECanRegPtr->CANME.all;
                    shadow_canme.all |= CANOPEN_F28335_TX_MB_MASK;
                    ECanRegPtr->CANME.all = shadow_canme.all;

                    shadow_cantrs.all = CANOPEN_F28335_TX_MB_MASK;
                    ECanRegPtr->CANTRS.all = shadow_cantrs.all;

                    pCurrentCanTxObj = pTxBuffer;
                    break;
                }
            }
            if(index >= CANmodule->txSize) {
                /* Nothing to Transmit */
                pCurrentCanTxObj = NULL;
            }
        }
        // CO_UNLOCK_CAN_SEND(CANmodule); Note: Can't use mutex in ISR
    }

    return xHigherPriorityTaskWoken;
}


#if CONFIG_CANOPEN_USE_CAN_A
#pragma CODE_SECTION(CANA_isr, "ramfuncs")
interrupt void CANA_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    BaseType_t xHigherPriorityTaskWoken = false;
    if(pCANmodule != NULL) {
        if(pCANmodule->CANptr == (void *)&ECanaRegs) {
            xHigherPriorityTaskWoken = CAN_isrHandler(pCANmodule);
        } else {
            configASSERT(false);  // This should not happen
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif /* CONFIG_CANOPEN_USE_CAN_A */

#if CONFIG_CANOPEN_USE_CAN_B
#pragma CODE_SECTION(CANB_isr, "ramfuncs")
interrupt void CANB_isr(void)
{
    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    BaseType_t xHigherPriorityTaskWoken = false;
    if(pCANmodule != NULL) {
        if(pCANmodule->CANptr == (void *)&ECanbRegs) {
            xHigherPriorityTaskWoken = CAN_isrHandler(pCANmodule);
        } else {
            configASSERT(false);  // This should not happen
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif /* CONFIG_CANOPEN_USE_CAN_B */

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    /* Put CAN module in configuration mode */
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;

    if(CANptr == NULL) return;

    ECanRegPtr = (volatile struct ECAN_REGS *)CANptr;

    EALLOW;
    shadow_canmc.all = ECanRegPtr->CANMC.all;
    shadow_canmc.bit.CCR = 1;
    ECanRegPtr->CANMC.all = shadow_canmc.all;

    shadow_canes.all = ECanRegPtr->CANES.all;
    do {
        shadow_canes.all = ECanRegPtr->CANES.all;
    } while(shadow_canes.bit.CCE != 1);

    EDIS;
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;

    if(CANmodule == NULL) return;

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);

    EALLOW;
    shadow_canmc.all = ECanRegPtr->CANMC.all;
    shadow_canmc.bit.CCR = 0;
    ECanRegPtr->CANMC.all = shadow_canmc.all;

    shadow_canes.all = ECanRegPtr->CANES.all;
    do {
        shadow_canes.all = ECanRegPtr->CANES.all;
    } while(shadow_canes.bit.CCE != 0);
    EDIS;

    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    /* Local variable used for CAN configuration */
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    volatile union CANLAM_REG * LamPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;
    union CANBTC_REG shadow_canbtc;
    union CANMSGID_REG shadow_canmsgid;
    union CANLAM_REG shadow_canlam;
    union CANGIM_REG shadow_cangim;

    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL ||
       CANptr==NULL || ((CANOPEN_F28335_TX_MB_CNT + rxSize) > CANOPEN_F82335_MAILBOX_CNT)){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    pCANmodule = CANmodule;

    ECanRegPtr = (volatile struct ECAN_REGS *)CANptr;

    /* Create Mutex for CAN Send */
    if(CANmodule->mutexCanSend == NULL) {
        CANmodule->mutexCanSend = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->mutexBufferCanSend));
        configASSERT(CANmodule->mutexCanSend);
    }

    /* Create Mutex for CAN OD */
    if(CANmodule->mutexCanOD == NULL) {
        CANmodule->mutexCanOD = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->mutexBufferCanOD));
        configASSERT(CANmodule->mutexCanOD);
    }

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = (rxSize <= CANOPEN_F28335_RX_MB_CNT) ? true : false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->errOld = 0U;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
        txArray[i].bufferInhibitFlag = false;
    }

    /* Configure CAN module registers */
    if(ECanRegPtr == &ECanaRegs) {
        MBoxPtr = &(ECanaMboxes.MBOX0);
        LamPtr = &(ECanaLAMRegs.LAM0);
    } else if(ECanRegPtr == &ECanbRegs) {
        MBoxPtr = &(ECanbMboxes.MBOX0);
        LamPtr = &(ECanbLAMRegs.LAM0);
    } else {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    EALLOW;
    shadow_canmc.all = ECanRegPtr->CANMC.all;
    shadow_canmc.bit.DBO = 0;
    shadow_canmc.bit.SCB = 1;
    shadow_canmc.bit.CCR = 1;
    shadow_canmc.bit.ABO = 1;
    ECanRegPtr->CANMC.all = shadow_canmc.all;

    do {
        shadow_canes.all = ECanRegPtr->CANES.all;
    } while(shadow_canes.bit.CCE != 1);

    for(i = 0; i < 32; i++) {
        MBoxPtr[i].MSGCTRL.all = 0x00000000;
    }

    ECanRegPtr->CANGIM.all = 0x00000000;        // disable all interrupts
    ECanRegPtr->CANMIM.all = 0x00000000;        // disable all mailbox interrupts
    ECanRegPtr->CANMIL.all = 0x00000000;        // mailbox interrupts on interrupt line0
    ECanRegPtr->CANTA.all  = 0xFFFFFFFF;        // Clears CANTA by writing 1's
    ECanRegPtr->CANRMP.all = 0xFFFFFFFF;        // Clears RMP by writing 1's
    ECanRegPtr->CANGIF0.all = 0xFFFFFFFF;       // Clears Line0 Global interrupt flags by writing 1's
    ECanRegPtr->CANGIF1.all = 0xFFFFFFFF;       // Clears Line1 Global interrupt flags by writing 1's
    ECanRegPtr->CANME.all  = 0x00000000;        // disable all mailbox
    ECanRegPtr->CANMD.all  = 0x00000000;        // Default all mailbox as transmit.

    /* Configure CAN timing */
    for(i = 0; i < N_BTC_CONFIG_ENTRY; i++) {
        if(CANbitRate == BTC_CONFIG_TABLE[i].bit_rate_kbps) {
            shadow_canbtc.all = 0;
            shadow_canbtc.bit.BRPREG = BTC_CONFIG_TABLE[i].brp;
            shadow_canbtc.bit.TSEG1REG = BTC_CONFIG_TABLE[i].tseg1;
            shadow_canbtc.bit.TSEG2REG = BTC_CONFIG_TABLE[i].tseg2;
            shadow_canbtc.bit.SAM = 1;
            ECanRegPtr->CANBTC.all = shadow_canbtc.all;
            break;
        }
    }
    if(i == N_BTC_CONFIG_ENTRY) {
        /* Invalid Bit Rate */
        EDIS; // <- closes the EALLOW before returning
        return CO_ERROR_ILLEGAL_BAUDRATE;
    }

    /* Configure CAN module hardware filters */
    if(CANmodule->useCANrxFilters){
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.IDE = 0;    // Uses 11-bit Standard identifier
        shadow_canmsgid.bit.STDMSGID = 0;
        shadow_canmsgid.bit.AME = 1;    // Use acceptance mask
        for(i = 0; i < 32; i++) {
            MBoxPtr[i].MSGID.all = shadow_canmsgid.all;
        }

        /* Configure all masks so, that received message must match filter */
        shadow_canlam.all = 0;
        shadow_canlam.bit.LAMI = 1;         // Uses local acceptance mask
        shadow_canlam.bit.LAM_L = 0;
        shadow_canlam.bit.LAM_H = 0x0000;   // Received 11-bit Standard Identifier
                                            // must match the corresponding MSGID register
        for(i = 0; i < 32; i++) {
            LamPtr[i].all = shadow_canlam.all;
        }
    } else {
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.IDE = 0;    // Uses 11-bit Standard identifier
        shadow_canmsgid.bit.STDMSGID = 0;
        shadow_canmsgid.bit.AME = 1;    // Use acceptance mask
        for(i = 0; i < 32; i++) {
            MBoxPtr[i].MSGID.all = shadow_canmsgid.all;
        }

        /* Configure mask 0 so, that all messages with standard identifier are accepted */
        shadow_canlam.all = 0;
        shadow_canlam.bit.LAMI = 1;         // Use local acceptance mask
        shadow_canlam.bit.LAM_L = 0;
        shadow_canlam.bit.LAM_H = 0x1FFC;   // Don't care on 11-bit Standard Identifier, LAM[28:18]
                                            // Accepts all standard identifier
        for(i = 0; i < 32; i++) {
            LamPtr[i].all = shadow_canlam.all;
        }

    }

#if CONFIG_CANOPEN_USE_CAN_A
    PieVectTable.ECAN0INTA = CANA_isr;
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;
    IER |= M_INT9;
#endif /* CONFIG_CANOPEN_USE_CAN_A */

#if CONFIG_CANOPEN_USE_CAN_B
    PieVectTable.ECAN0INTB = CANB_isr;
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;
    IER |= M_INT9;
#endif /* CONFIG_CANOPEN_USE_CAN_B */

    /* configure CAN interrupt registers */
    shadow_cangim.all = ECanRegPtr->CANGIM.all;
    shadow_cangim.bit.I0EN = 1;
    ECanRegPtr->CANGIM.all = shadow_cangim.all;

    EDIS;

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;

    if (CANmodule != NULL) {
        /*
         * Set CAN module to power down mode
         */
        ECanRegPtr = (volatile struct ECAN_REGS *)CANmodule->CANptr;

        /*
         * Allow transmission of any packet in progess to complete
         * and enter power-down mode.
         */
        EALLOW;
        shadow_canmc.all = ECanRegPtr->CANMC.all;
        shadow_canmc.bit.PDR = 1;
        shadow_canmc.bit.WUBA = 0;
        ECanRegPtr->CANMC.all = shadow_canmc.all;

        /* Wait for Power-down mode acknowledge */
        do {
            shadow_canes.all = ECanRegPtr->CANES.all;
        } while (shadow_canes.bit.PDA != 1);

        /*
         * Turn off the Clock to CAN peripheral
         */
        /// TODO
        EDIS;
    }
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t *CANmodule,
        uint16_t       index,
        uint16_t       ident,
        uint16_t       mask,
        bool_t         rtr,
        void           *object,
        void           (*CANrx_callback)(void *object, void *message))
{
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    volatile union CANLAM_REG * LamPtr;
    union CANGIM_REG shadow_cangim;
    union CANME_REG shadow_canme;
    union CANMSGID_REG shadow_canmsgid;
    union CANMSGCTRL_REG shadow_canmsgctrl;
    union CANLAM_REG shadow_canlam;
    union CANMD_REG shadow_canmd;
    union CANMIM_REG shadow_canmim;

    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
        /*
         * Note: In C2000 MCU, Higher Mailbox number takes higher priority
         * Change index, i.e. NMT (index0) has highest receive mailbox priority
         */
        index = CANmodule->rxSize - index - 1;
        ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);

        if(ECanRegPtr == &ECanaRegs) {
            MBoxPtr = &(ECanaMboxes.MBOX0);
            LamPtr = &(ECanaLAMRegs.LAM0);
        } else if(ECanRegPtr == &ECanbRegs) {
            MBoxPtr = &(ECanbMboxes.MBOX0);
            LamPtr = &(ECanbLAMRegs.LAM0);
        } else {
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }

        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* Disable CAN global interrupt */
        EALLOW;
        shadow_cangim.all = ECanRegPtr->CANGIM.all;
        shadow_cangim.bit.I0EN = 0;
        ECanRegPtr->CANGIM.all = shadow_cangim.all;
        EDIS;

        /* Disable Mailbox Interrupt */
        EALLOW;
        shadow_canmim.all = ECanRegPtr->CANMIM.all;
        shadow_canmim.all &= ~(1UL << index);
        ECanRegPtr->CANMIM.all = shadow_canmim.all;
        EDIS;

        /* Disable Mailbox to allow configuration */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all &= ~(1UL << index);
        ECanRegPtr->CANME.all = shadow_canme.all;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.AME = 1;
        shadow_canmsgid.bit.STDMSGID = buffer->ident;
        MBoxPtr[index].MSGID.all = shadow_canmsgid.all;

        shadow_canmsgctrl.all = 0;
        if(rtr){
            buffer->ident |= 0x0800U;
            shadow_canmsgctrl.bit.RTR = 1;
        }
        MBoxPtr[index].MSGCTRL.all = shadow_canmsgctrl.all;

        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){
            /* Set local acceptance mask */
            shadow_canlam.all = 0;
            shadow_canlam.bit.LAMI = 1;
            shadow_canlam.bit.LAM_L = 0;
            shadow_canlam.bit.LAM_H = 0;
            LamPtr[index].all = shadow_canlam.all;  // Received 11-bit Standard Identifier
                                                    // must match the corresponding MSGID register
        }

        /* Configure mailbox direction */
        shadow_canmd.all = ECanRegPtr->CANMD.all;
        shadow_canmd.all |= (1UL << index);
        ECanRegPtr->CANMD.all = shadow_canmd.all;

        /* Re-enable mailbox */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all |= (1UL << index);
        ECanRegPtr->CANME.all = shadow_canme.all;

        /* Re-enable mailbox interrupt */
        EALLOW;
        shadow_canmim.all = ECanRegPtr->CANMIM.all;
        shadow_canmim.all |= (1UL << index);
        ECanRegPtr->CANMIM.all = shadow_canmim.all;
        EDIS;

        /* Re-enable interrupt */
        EALLOW;
        shadow_cangim.all = ECanRegPtr->CANGIM.all;
        shadow_cangim.bit.I0EN = 1;
        ECanRegPtr->CANGIM.all = shadow_cangim.all;
        EDIS;
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t *CANmodule,
        uint16_t       index,
        uint16_t       ident,
        bool_t         rtr,
        uint8_t        noOfBytes,
        bool_t         syncFlag)
{
    CO_CANtx_t *buffer = NULL;
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANMIM_REG shadow_canmim;
    /* CAN Tx uses fixed mailbox 31 */
    uint16_t txMailboxIndex = CANOPEN_F28335_TX_MB_IDX;

    if((CANmodule != NULL) && (index < CANmodule->txSize)) {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        buffer->mailboxIdx = txMailboxIndex;

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = (uint32_t)ident & 0x07FFU;
        if(rtr) {
            buffer->ident |= 0x0800U;
        }
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    if(CANmodule != NULL) {
        ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);
        EALLOW;
        shadow_canmim.all = ECanRegPtr->CANMIM.all;
        shadow_canmim.all |= CANOPEN_F28335_TX_MB_MASK;
        ECanRegPtr->CANMIM.all = shadow_canmim.all;
        EDIS;
    }
    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(
        CO_CANmodule_t *CANmodule,
        CO_CANtx_t *buffer)
{
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    union CANTRS_REG shadow_cantrs;
    union CANME_REG shadow_canme;
    union CANMSGID_REG shadow_canmsgid;
    union CANMSGCTRL_REG shadow_canmsgctrl;
    union CANMDL_REG shadow_canmdl;
    union CANMDH_REG shadow_canmdh;
    union CANGIM_REG shadow_cangim;

    CO_ReturnError_t err = CO_ERROR_NO;

    if((CANmodule == NULL) || (buffer == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);
    if(ECanRegPtr == &ECanaRegs) {
        MBoxPtr = &(ECanaMboxes.MBOX0);
    } else if(ECanRegPtr == &ECanbRegs) {
        MBoxPtr = &(ECanbMboxes.MBOX0);
    } else {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    /*
     * Critical Section (START)
     *   In CAN ISR, we can't use mutex.
     *   In task context, we must disable the CAN ISR to prevent race condition.
     */
    shadow_cangim.all = ECanRegPtr->CANGIM.all;
    uint16_t i0en = shadow_cangim.bit.I0EN;
    shadow_cangim.bit.I0EN = 0;
    EALLOW;
    ECanRegPtr->CANGIM.all = shadow_cangim.all;
    EDIS;
    CO_LOCK_CAN_SEND(CANmodule);

    shadow_cantrs.all = ECanRegPtr->CANTRS.all;

    /* if CAN TX mailbox is free, copy message to it */
    if(((shadow_cantrs.all & CANOPEN_F28335_TX_MB_MASK) == 0) && (buffer->bufferFull == false)){
        pCurrentCanTxObj = buffer;
        buffer->bufferInhibitFlag = buffer->syncFlag;
        /*
         * copy message and txRequest
         */
        /* Disable mailbox to be able to modify MSGID */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all &= ~CANOPEN_F28335_TX_MB_MASK;
        ECanRegPtr->CANME.all = shadow_canme.all;
        /* Configure MSGID */
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.STDMSGID = buffer->ident & 0x07FFU;
        MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MSGID.all = shadow_canmsgid.all;
        /* MSGCTRL */
        shadow_canmsgctrl.all = 0;
        shadow_canmsgctrl.bit.DLC = buffer->DLC;
        if(buffer->ident & 0x0800U) {
            shadow_canmsgctrl.bit.RTR = 1;
        }
        /* DATA */
        shadow_canmdl.byte.BYTE0 = buffer->data[0];
        shadow_canmdl.byte.BYTE1 = buffer->data[1];
        shadow_canmdl.byte.BYTE2 = buffer->data[2];
        shadow_canmdl.byte.BYTE3 = buffer->data[3];
        shadow_canmdh.byte.BYTE4 = buffer->data[4];
        shadow_canmdh.byte.BYTE5 = buffer->data[5];
        shadow_canmdh.byte.BYTE6 = buffer->data[6];
        shadow_canmdh.byte.BYTE7 = buffer->data[7];

        MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MSGCTRL.all = shadow_canmsgctrl.all;
        MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MDL.all = shadow_canmdl.all;
        MBoxPtr[CANOPEN_F28335_TX_MB_IDX].MDH.all = shadow_canmdh.all;

        /* Re-enable mailbox */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all |= CANOPEN_F28335_TX_MB_MASK;
        ECanRegPtr->CANME.all = shadow_canme.all;

        shadow_cantrs.all = CANOPEN_F28335_TX_MB_MASK;
        ECanRegPtr->CANTRS.all = shadow_cantrs.all;
    } else {
        /* CAN Mailbox31 is not free, message will be sent by interrupt */
        buffer->bufferFull = true;
    }

    /*
     * Critical Section (END)
     */
    CO_UNLOCK_CAN_SEND(CANmodule);
    shadow_cangim.all = ECanRegPtr->CANGIM.all;
    shadow_cangim.bit.I0EN = i0en;
    EALLOW;
    ECanRegPtr->CANGIM.all = shadow_cangim.all;
    EDIS;


    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANGIM_REG shadow_cangim;
    CO_CANtx_t *buffer = NULL;
    uint32_t tpdoDeleted = 0U;

    if(CANmodule == NULL) {
        return;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);

    /*
     * Critical Section (START)
     *   In CAN ISR, we can't use mutex.
     *   In task context, we must disable the CAN ISR to prevent race condition.
     */
    shadow_cangim.all = ECanRegPtr->CANGIM.all;
    uint16_t i0en = shadow_cangim.bit.I0EN;
    shadow_cangim.bit.I0EN = 0;
    EALLOW;
    ECanRegPtr->CANGIM.all = shadow_cangim.all;
    EDIS;
    CO_LOCK_CAN_SEND(CANmodule);

#if 0 /// TODO: further reading on F28335 CAN peripheral when aborting.
      /// i.e. Check ISR handling CANAA and existing implementation is on CANTA based ISR
    /*
     * Check if synchronous TPDO message is in progress.
     */
    shadow_cantrs.all = ECanRegPtr->CANTRS.all;
    if((shadow_cantrs.all & CANOPEN_F28335_TX_MB_MASK) &&
        pCurrentCanTxObj->bufferInhibitFlag) {
        /*
         * Synchronous TPDO message is already in progress of transmission.
         * Abort the transmission. Take special care with this functionality.
         */
        ECanRegPtr->CANTRR.all = CANOPEN_F28335_TX_MB_MASK;
        pCurrentCanTxObj->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
#endif

    /*
     * Delete all pending TPDOs in the TX buffers
     */
    for(uint16_t i = CANOPEN_TPDO_BUF_IDX; i < OD_CNT_TPDO; i++) {
        buffer = &(CANmodule->txArray[i]);
        if(buffer->bufferFull) {
            if(buffer->syncFlag) {
                buffer->bufferFull = false;
                tpdoDeleted = 2U;
            }
        }
    }

    /*
     * Critical Section (END)
     */
    CO_UNLOCK_CAN_SEND(CANmodule);
    shadow_cangim.all = ECanRegPtr->CANGIM.all;
    shadow_cangim.bit.I0EN = i0en;
    EALLOW;
    ECanRegPtr->CANGIM.all = shadow_cangim.all;
    EDIS;

    if(tpdoDeleted != 0U) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANES_REG shadow_canes;
    union CANTEC_REG shadow_cantec;
    union CANREC_REG shadow_canrec;

    uint32_t err;

    if(CANmodule == NULL) {
        return;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);
    shadow_canes.all = ECanRegPtr->CANES.all;
    shadow_cantec.all = ECanRegPtr->CANTEC.all;
    shadow_canrec.all = ECanRegPtr->CANREC.all;

    err = (shadow_canes.all & 0x01FF0000U) |
          ((shadow_cantec.bit.TEC << 8) & 0x0000FF00U) |
          (shadow_canrec.bit.REC & 0x000000FFU);

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        /* bus off */
        if(shadow_canes.bit.BO) {
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            uint16_t rxErrors = shadow_canrec.bit.REC;
            uint16_t txErrors = shadow_cantec.bit.TEC;

            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (txErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        CANmodule->CANerrorStatus = status;
    }
}

#endif /* CONFIG_USE_CANOPEN */
