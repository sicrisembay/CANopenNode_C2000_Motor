/*!
 * @file        CO_c28x.c
 * @author      Sicris Rey Embay
 * @copyright   2024 Sicris Rey Embay
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
#include "FreeRTOS.h"
#include "task.h"
#include "CANopen.h"
#include "OD.h"
#include "storage/CO_storage.h"
#include "board.h"

/* default values for CO_CANopenInit() */
#define NMT_CONTROL             (CO_NMT_control_t) (CO_NMT_STARTUP_TO_OPERATIONAL \
                                    | CO_NMT_ERR_ON_ERR_REG         \
                                    | CO_ERR_REG_GENERIC_ERR        \
                                    | CO_ERR_REG_COMMUNICATION)

#define FIRST_HB_TIME           CONFIG_CANOPEN_FIRST_HB_TIME
#define SDO_SRV_TIMEOUT_TIME    CONFIG_CANOPEN_SDO_SRV_TIMEOUT
#define SDO_CLI_TIMEOUT_TIME    CONFIG_CANOPEN_SDO_CLIENT_TIMEOUT
#define SDO_CLI_BLOCK           false
#define OD_STATUS_BITS          NULL

#define CANOPEN_TIMER_INTERVAL_US   (CONFIG_CANOPEN_TIMER_INTERVAL_MS * 1000UL)
#define CANOPEN_MAIN_INTERVAL_US    (CONFIG_CANOPEN_MAIN_INTERVAL_MS * 1000UL)

typedef struct {
    uint16_t pendingBitRate;    // Pending CAN bit rate, can be set via LSS
    uint16_t pendingNodeId;     // Pending CANopen NodeId, can be set via LSS
} mainStorage_t;


static TaskHandle_t taskHdl_CO = NULL;
static StaticTask_t taskBuffer_CO;
static StackType_t stack_CO[CONFIG_CANOPEN_MAIN_TASK_STACK];
static TaskHandle_t taskHdl_CO_timer = NULL;
static StaticTask_t taskBuffer_CO_timer;
static StackType_t stack_CO_timer[CONFIG_CANOPEN_TIMER_TASK_STACK];

static CO_t * CO = NULL;
static void * CANptr = NULL;

static mainStorage_t mainStorage = {
    .pendingBitRate = CONFIG_CANOPEN_DEFAULT_BITRATE,
#if (CONFIG_CANOPEN_DEFAULT_NODEID > 0x7F)
    .pendingNodeId = CO_LSS_NODE_ID_ASSIGNMENT,
#else
    .pendingNodeId = CONFIG_CANOPEN_DEFAULT_NODEID,
#endif
};


static bool LSScfgStoreCallback(void *object, uint8_t id, uint16_t bitRate)
{
    ODR_t odRet;
    OD_entry_t * pEntry;
    OD_IO_t io;
    OD_size_t bytesWritten;
    uint8_t buf[4] = {0};
    mainStorage_t * pStorage = (mainStorage_t *)object;
    pStorage->pendingBitRate = bitRate;
    pStorage->pendingNodeId = id;
    pEntry = OD_ENTRY_H1010_storeParameters;
    odRet = OD_getSub(pEntry, 4, &io, false);   // Sub-index 4 (Manufacturer Defined Parameter)
    if(odRet != ODR_OK) {
        return false;
    }
    buf[0] = 0x73; // 's'
    buf[1] = 0x61; // 'a'
    buf[2] = 0x76; // 'v'
    buf[3] = 0x65; // 'e'
    odRet = io.write(&io.stream, buf, 4, &bytesWritten);
    if((odRet != ODR_OK) || (bytesWritten != 4)) {
        return false;
    }

    return true;
}


static void taskCO_timer(void * pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime;

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, CONFIG_CANOPEN_TIMER_INTERVAL_MS);
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, CANOPEN_TIMER_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, CANOPEN_TIMER_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, CANOPEN_TIMER_INTERVAL_US, NULL);
#endif
#if (CONFIG_CANOPEN_402)
            CO_process_402(CO, CANOPEN_TIMER_INTERVAL_US);
#endif

            /* Further I/O or nonblocking application code may go here. */
        }
    }
}


static void taskCO_main(void * pvParameters)
{
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    uint8_t activeNodeId = mainStorage.pendingNodeId; /* Copied from CO_pendingNodeId in the communication reset section */
    TickType_t xLastWakeTime;
    (void)pvParameters;

    /* Allocate CANopen object */
    CO = CO_new(NULL, &heapMemoryUsed);
    configASSERT(CO != NULL);

    while(reset != CO_RESET_APP) {
        /* CANopen communication reset - initialize CANopen objects *******************/
        CANOPEN_LOG_DEBUG("CANopenNode - Reset communication...\n");

        /* Wait rt_thread. */
        CO->CANmodule->CANnormal = false;

        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode(CANptr);

        /* Initialize CANopen */
        err = CO_CANinit(CO, CANptr, mainStorage.pendingBitRate);
        configASSERT(err == CO_ERROR_NO);

        /*
         * Use C2000 Unique Device Number
         * Refer to SPRACD0B
         */
        uint16_t uid_lsw = *(((uint16_t *)0x000900));
        uint32_t uid_msw = *(((uint16_t *)0x000901));
        OD_PERSIST_COMM.x1018_identity.serialNumber = (uid_msw << 16) | uid_lsw;

        CO_LSS_address_t lssAddress = {
            .identity = {
                 .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                 .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                 .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                 .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
            }
        };

        err = CO_LSSinit(CO, &lssAddress, (uint8_t *)&(mainStorage.pendingNodeId), &(mainStorage.pendingBitRate));
        configASSERT(err == CO_ERROR_NO);

        activeNodeId = mainStorage.pendingNodeId;
        uint32_t errInfo = 0;

        err = CO_CANopenInit(CO,                /* CANopen object */
                             NULL,              /* alternate NMT */
                             NULL,              /* alternate em */
                             OD,                /* Object dictionary */
                             OD_STATUS_BITS,    /* Optional OD_statusBits */
                             NMT_CONTROL,       /* CO_NMT_control_t */
                             FIRST_HB_TIME,     /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                             activeNodeId,
                             &errInfo);

        if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                CANOPEN_LOG_ERROR("Error: Object Dictionary entry 0x%x\n", errInfo);
            }
            else {
                CANOPEN_LOG_ERROR("Error: CANopen initialization failed: %d\n", err);
            }
            configASSERT(false);
        }

        CO_LSSslave_initCfgStoreCallback(CO->LSSslave, &mainStorage, LSScfgStoreCallback);

        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        configASSERT(err == CO_ERROR_NO);

        /*
         * Create CANopen Timer Task
         * Note: taskCO_timer priority must be higher than taskCO_main priority
         */
        if(taskHdl_CO_timer == NULL) {
            taskHdl_CO_timer = xTaskCreateStatic(
                                    taskCO_timer,
                                    "CO_timer",
                                    CONFIG_CANOPEN_TIMER_TASK_STACK,
                                    (void *)0,
                                    CONFIG_CANOPEN_TIMER_TASK_PRIORITY,
                                    stack_CO_timer,
                                    &taskBuffer_CO_timer);
            configASSERT(taskHdl_CO_timer != NULL);
        }

        /* Configure CAN transmit and receive interrupt */


        /* Configure CANopen callbacks, etc */

#if (CO_CONFIG_TIME & CO_CONFIG_TIME_ENABLE)
        /* Configure Time */
#if CONFIG_CANOPEN_TIME_PRODUCER_INTERVAL_MS
        CO_TIME_set(CO->TIME, xTaskGetTickCount(), 0, CONFIG_CANOPEN_TIME_PRODUCER_INTERVAL_MS);
#else
        CO_TIME_set(CO->TIME, xTaskGetTickCount(), 0, 0);
#endif
#endif

        /* start CAN */
        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        CANOPEN_LOG_INFO("CANopenNode - Running...\r\n");

        xLastWakeTime = xTaskGetTickCount();

        while(reset == CO_RESET_NOT) {
            vTaskDelayUntil(&xLastWakeTime, CONFIG_CANOPEN_MAIN_INTERVAL_MS);

            /* loop for normal program execution ******************************************/

            /* CANopen process */
            reset = CO_process(CO, false, CANOPEN_MAIN_INTERVAL_US, NULL);

            /* Nonblocking application code may go here. */


            /* Process automatic storage */


            /* optional sleep for short time */


        }

    };

    /* Complete Device Reset */
    BOARD_reset();

    /* Must not reach here */
    while(1);
}


void CANOPEN_c28x_init(void)
{
    union CANTIOC_REG shadow_cantioc;
    union CANRIOC_REG shadow_canrioc;

    /*
     * Initialize CAN GPIO
     */
    EALLOW;
    /*
     * TX GPIO
     */
#if CONFIG_CANOPEN_TX_GPIO8
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;      // Enable pull-up for GPIO8(CANTXB)
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 2;     // Configure GPIO8 for CANTXB
#elif CONFIG_CANOPEN_TX_GPIO12
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;     // Enable pull-up for GPIO12(CANTXB)
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;    // Configure GPIO12 for CANTXB
#elif CONFIG_CANOPEN_TX_GPIO16
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;     // Enable pull-up for GPIO16(CANTXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 2;    // Configure GPIO16 for CANTXB
#elif CONFIG_CANOPEN_TX_GPIO20
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;     // Enable pull-up for GPIO20(CANTXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 3;    // Configure GPIO20 for CANTXB
#elif CONFIG_CANOPEN_TX_GPIO19
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;     //Enable pull-up for GPIO19 (CANTXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;    // Configure GPIO19 for CANTXA
#elif CONFIG_CANOPEN_TX_GPIO31
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     //Enable pull-up for GPIO31 (CANTXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA
#else
    #error "Invalid Tx GPIO"
#endif

    /*
     * RX GPIO
     */
#if CONFIG_CANOPEN_RX_GPIO10
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;     // Enable pull-up for GPIO10(CANRXB)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 3;   // Asynch qual for GPIO10 (CANRXB)
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 2;    // Configure GPIO10 for CANRXB
#elif CONFIG_CANOPEN_RX_GPIO13
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;     // Enable pull-up for GPIO13(CANRXB)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;   // Asynch qual for GPIO13 (CANRXB)
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 2;    // Configure GPIO13 for CANRXB
#elif CONFIG_CANOPEN_RX_GPIO17
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;     // Enable pull-up for GPIO17(CANRXB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;   // Asynch qual for GPIO17 (CANRXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 2;    // Configure GPIO17 for CANRXB
#elif CONFIG_CANOPEN_RX_GPIO21
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;     // Enable pull-up for GPIO21(CANRXB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3;   // Asynch qual for GPIO21 (CANRXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 3;    // Configure GPIO21 for CANRXB
#elif CONFIG_CANOPEN_RX_GPIO18
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;     // Enable pull-up for GPIO18 (CANRXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;   // Asynch qual for GPIO18 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;    // Configure GPIO18 for CANRXA
#elif CONFIG_CANOPEN_RX_GPIO30
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA
#else
    #error "Invalid Rx GPIO"
#endif

#if CONFIG_CANOPEN_USE_CAN_A
    SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 1;
    CANptr = (void *)(&ECanaRegs);
    shadow_cantioc.all = ECanaRegs.CANTIOC.all;
    shadow_cantioc.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = shadow_cantioc.all;

    shadow_canrioc.all = ECanaRegs.CANRIOC.all;
    shadow_canrioc.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = shadow_canrioc.all;
#elif CONFIG_CANOPEN_USE_CAN_B
    SysCtrlRegs.PCLKCR0.bit.ECANBENCLK = 1;
    CANptr = (void *)(&ECanbRegs);
    shadow_cantioc.all = ECanbRegs.CANTIOC.all;
    shadow_cantioc.bit.TXFUNC = 1;
    ECanbRegs.CANTIOC.all = shadow_cantioc.all;

    shadow_canrioc.all = ECanbRegs.CANRIOC.all;
    shadow_canrioc.bit.RXFUNC = 1;
    ECanbRegs.CANRIOC.all = shadow_canrioc.all;
#else
    #error "Invalid CAN clock"
#endif

    EDIS;

    taskHdl_CO = xTaskCreateStatic(
                            taskCO_main,
                            "CO_main",
                            CONFIG_CANOPEN_MAIN_TASK_STACK,
                            (void *)0,
                            CONFIG_CANOPEN_MAIN_TASK_PRIORITY,
                            stack_CO,
                            &taskBuffer_CO);
    configASSERT(taskHdl_CO != NULL);

}


#endif /* CONFIG_USE_CANOPEN */
