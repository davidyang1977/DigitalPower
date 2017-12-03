/*-----------------------------------------------------------------------------
 * Name:    LED_LPC824-LPCXpresso.c
 * Purpose: LED interface for LPC82x-LPCXpresso evaluation board
 * Rev.:    1.01
 *----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2016 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include "LPC8xx.h"                     // Device header
#include "Board.h"

#define LED_NUM             (3)

const uint32_t led_mask[] = { 1lu << 27, 1UL << 16, 1UL << 12 };

/**
  \fn          int32_t LED_Initialize (void)
  \brief       Initialize LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Initialize (void) {

  LPC_SYSCON->SYSAHBCLKCTRL |= ( (1 << DMA_CLK_EN) |    /* Enable Clock for DMA */
																 (1 << ADC_CLK_EN) |    /* Enable Clock for ADC */
																 (1 << CMP_CLK_EN) |    /* Enable Clock for Analog Compare*/
																 (1 << UART0_CLK_EN) | 	/* Enable Clock for UART0 */
																 (1 << UART1_CLK_EN) | 	/* Enable Clock for UART1 */
																 (1 << MRT_CLK_EN) | 		/* Enable Clock for MRT */
																 (1 << GPIO_CLK_EN) |   /* Enable Clock for GPIO */
																 (1 << I2C0_CLK_EN) | 	/* Enable Clock for I2C0 */
																 (1 << FLASH_CLK_EN) | 	/* Enable Clock for FLASH */
																 (1 << FLASHREG_CLK_EN) |  /* Enable Clock for FLASHREG */
																 (1 << RAM01_CLK_EN) |	   /* Enable Clock for RAM0/1 */
																 (1 << ROM_CLK_EN) );      /* Enable Clock for ROM */

  LPC_GPIO_PORT->DIR0 |= (led_mask[0] |           /* configure GPIO as output */
                          led_mask[1] |
                          led_mask[2]  );

  LED_SetOut (0);                                 /* switch LEDs off          */

  return 0;
}

/**
  \fn          int32_t LED_On (uint32_t num)
  \brief       Turn on requested LED
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_On (uint32_t num) {

  if (num < LED_NUM) {
    LPC_GPIO_PORT->CLR0 = led_mask[num];          /* LED On                   */
  }
  return 0;
}

/**
  \fn          int32_t LED_Off (uint32_t num)
  \brief       Turn off requested LED
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Off (uint32_t num) {

  if (num < LED_NUM) {
    LPC_GPIO_PORT->SET0 = led_mask[num];          /* LED Off                  */
  }
  return 0;
}

/**
  \fn          int32_t LED_SetOut (uint32_t val)
  \brief       Write value to LEDs
  \param[in]   val  value to be displayed on LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_SetOut(uint32_t value) {
  int i;

  for (i = 0; i < LED_NUM; i++) {
    if (value & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
  return 0;
}

/**
  \fn          uint32_t LED_GetCount (void)
  \brief       Get number of LEDs
  \return      Number of available LEDs
*/
uint32_t LED_GetCount (void) {
  return LED_NUM;
}

