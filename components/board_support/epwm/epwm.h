/*
 * epwm.h
 *
 *  Created on: 11 Aug 2024
 *      Author: Sicris
 */

#ifndef COMPONENTS_BOARD_SUPPORT_EPWM_EPWM_H_
#define COMPONENTS_BOARD_SUPPORT_EPWM_EPWM_H_

#include "autoconf_post.h"

#if CONFIG_USE_EPWM

typedef void (*epwm_isr_cb)(void);

void EPWM_init(epwm_isr_cb cb);

#pragma CODE_SECTION(EPWM_setDutyCycle, "ramfuncs")
void EPWM_setDutyCycle(float phA, float phB, float phC);

#endif /* CONFIG_USE_EPWM */
#endif /* COMPONENTS_BOARD_SUPPORT_EPWM_EPWM_H_ */
