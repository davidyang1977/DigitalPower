/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    Blinky.c
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2014 - 2016 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "LPC8xx.h"                     // Device header
#include "Board.h"                  // ::Board Support:LED

osThreadId t_ledOn;                     /* assigned task id of task: ledOn   */
osThreadId t_ledOff;                    /* assigned task id of task: ledOff  */

/*----------------------------------------------------------------------------
  Task 1 'ledOn': switches the LED on
 *---------------------------------------------------------------------------*/
void ledOn (void const *argument) {
  for (;;) {
    LED_On(0);                          /* Turn LED On                       */
    osSignalSet (t_ledOff, 0x0001);     /* send event to task 'ledoff'       */
    osDelay (500);                      /* delay 50 clock ticks              */
  }
}

/*----------------------------------------------------------------------------
  Task 2 'ledOff': switches the LED off
 *---------------------------------------------------------------------------*/
void ledOff (void const *argument) {
  for (;;) {
    osSignalWait (0x0001, osWaitForever); /* wait for an event flag 0x0001   */
    osDelay (80);                       /* delay 8 clock ticks               */
    LED_Off(0);                         /* Turn LED Off                      */
  }
}


osThreadDef(ledOn,  osPriorityNormal, 1, 0);
osThreadDef(ledOff, osPriorityNormal, 1, 0);

/*----------------------------------------------------------------------------
  Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

  SystemCoreClockUpdate();

	LED_Initialize();                                   /* Initialize LEDs     */

  t_ledOn  = osThreadCreate(osThread(ledOn),  NULL);  /* start task 'ledOn'  */
  t_ledOff = osThreadCreate(osThread(ledOff), NULL);  /* start task 'ledOff' */
  osDelay(osWaitForever);
}
