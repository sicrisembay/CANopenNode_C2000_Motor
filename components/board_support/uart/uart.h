/*
 * uart.h
 *
 *  Created on: 6 Aug 2024
 *      Author: Sicris
 */

#ifndef COMPONENTS_BOARD_SUPPORT_UART_UART_H_
#define COMPONENTS_BOARD_SUPPORT_UART_UART_H_

#include "autoconf_post.h"

#if CONFIG_USE_UART
#include "stdint.h"

#define UART_FIFO_SZ        (16)

#ifndef uint8_t
#define uint8_t uint16_t
#endif

typedef enum {
#if CONFIG_ENABLE_UARTA
    UART_A,                 /*!< ID for UART-A */
#endif
#if CONFIG_ENABLE_UARTB
    UART_B,                 /*!< ID for UART-B */
#endif
#if CONFIG_ENABLE_UARTC
    UART_C,                 /*!< ID for UART-C */
#endif
    N_UART                  /*!< Number of UART instance (always last in enumeration) */
} UART_ID_T;

void UART_init(void);
bool UART_init_done(void);
bool UART_txBufferEmpty(const UART_ID_T id);
uint16_t UART_send(const UART_ID_T id, const uint8_t * pBuf, const uint16_t count);
uint16_t UART_sendFromISR(const UART_ID_T id, const uint8_t * pBuf, const uint16_t count);
uint16_t UART_receive(const UART_ID_T id, uint8_t * const pBuf, const uint16_t count, const TickType_t xTicksToWait);
uint16_t UART_receiveFromISR(const UART_ID_T id, uint8_t * const pBuf, const uint16_t count);


#endif /* CONFIG_USE_UART */
#endif /* COMPONENTS_BOARD_SUPPORT_UART_UART_H_ */
