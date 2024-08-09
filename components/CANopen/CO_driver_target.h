/*!
 * @file        CO_driver_target.h
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

#ifndef COMPONENTS_CANOPEN_CO_DRIVER_TARGET_H_
#define COMPONENTS_CANOPEN_CO_DRIVER_TARGET_H_

#include "autoconf_post.h"

#if CONFIG_USE_CANOPEN

#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define TAG_CANOPEN "CO"
#define CANOPEN_LOG_DEBUG(x, ...)
#define CANOPEN_LOG_INFO(x, ...)
#define CANOPEN_LOG_WARN(x, ...)
#define CANOPEN_LOG_ERROR(x, ...)

#ifndef uint8_t
#define uint8_t uint16_t
#endif

#ifndef int8_t
#define int8_t int16_t
#endif

#ifndef bool_t
#define bool_t uint16_t
#endif

#ifndef float32_t
#define float32_t float
#endif

#ifndef float64_t
#define float64_t double
#endif

#define CO_USE_GLOBALS

/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#define CO_CONFIG_LEDS  0

#if CONFIG_CANOPEN_STORAGE_EEPROM
  #define CO_CONFIG_STORAGE                 (CO_CONFIG_STORAGE_ENABLE)
  #define CO_CONFIG_CRC16                   (CO_CONFIG_CRC16_ENABLE)
  #define CO_CONFIG_STORAGE_MAX_ENTRIES     CONFIG_CANOPEN_STORAGE_EEPROM_MAX_ENTRIES
#else
  #define CO_CONFIG_STORAGE   (0)
  #define CO_CONFIG_CRC16     (0)
#endif

#if CONFIG_CANOPEN_LSS_SLAVE_ENABLE
  #define CO_CONFIG_LSS         (CO_CONFIG_LSS_SLAVE | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE)
#endif

#if CONFIG_CANOPEN_TIME_ENABLE
#if CONFIG_CANOPEN_TIME_PRODUCER
#define CO_CONFIG_TIME (CO_CONFIG_TIME_ENABLE | \
                        CO_CONFIG_TIME_PRODUCER | \
                        CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#else
#define CO_CONFIG_TIME (CO_CONFIG_TIME_ENABLE | \
                        CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif
#else
#define CO_CONFIG_TIME  (0)
#endif /* CONFIG_CANOPEN_TIME_ENABLE */

#if CONFIG_CANOPEN_SYNC_ENABLE
#if CONFIG_CANOPEN_SYNC_PRODUCER
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE | \
                        CO_CONFIG_SYNC_PRODUCER | \
                        CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | \
                        CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#else
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE | \
                        CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE | \
                        CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | \
                        CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif
#else
#define CO_CONFIG_SYNC  (0)
#define CO_CONFIG_PDO_SYNC_ENABLE (0)
#endif /* CONFIG_CANOPEN_SYNC_ENABLE */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x


/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg)  \
            ((uint16_t)(((CO_CANrxMsg_t *)msg)->ident))

#define CO_CANrxMsg_readDLC(msg)    \
            ((uint8_t)(((CO_CANrxMsg_t *)msg)->DLC))

#define CO_CANrxMsg_readData(msg)   \
            ((uint8_t *)&(((CO_CANrxMsg_t *)msg)->data[0]))

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

/* Transmit message object */
typedef struct {
    uint32_t mailboxIdx;
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
    volatile bool_t bufferInhibitFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t firstCANtxMessage;
    uint32_t errOld;
    SemaphoreHandle_t mutexCanSend;
    StaticSemaphore_t mutexBufferCanSend;
    SemaphoreHandle_t mutexCanOD;
    StaticSemaphore_t mutexBufferCanOD;
} CO_CANmodule_t;


/* Data storage object for one entry */
typedef struct {
    void *addr;
    size_t len; // in bytes
    uint8_t subIndexOD;
    uint8_t attr;
    /* Additional variables (target specific) */
    void *storageModule;
    uint16_t crc;
    size_t eepromAddr;
    size_t eepromAddrSignature;
    size_t offset;
} CO_storage_entry_t;

/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)    \
                xSemaphoreTakeRecursive(CAN_MODULE->mutexCanSend, portMAX_DELAY)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)  \
                xSemaphoreGiveRecursive(CAN_MODULE->mutexCanSend)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)          \
                xSemaphoreTakeRecursive(CAN_MODULE->mutexCanOD, portMAX_DELAY)
#define CO_UNLOCK_OD(CAN_MODULE)        \
                xSemaphoreGiveRecursive(CAN_MODULE->mutexCanOD)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) {CO_MemoryBarrier(); rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew) {CO_MemoryBarrier(); rxNew = NULL;}

#endif /* CONFIG_USE_CANOPEN */
#endif /* COMPONENTS_CANOPEN_CO_DRIVER_TARGET_H_ */
