/*
  Copyright (c) 2011 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Board initialization
 *
 * Good to know:
 *   - At reset, Reset_Handler did the system clock configuration. Core is running at F_CPU MHz.
 *   - Watchdog may be disabled by default in variant_init.cpp
 *   - During reset, all GPIO lines are configured as outputs.
 */
void init( void )
{
  // Set Systick to 1ms interval, common to all Cortex-M variants
  if ( SysTick_Config( SystemCoreClock / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }

  /* We disable the watchdog for this system */
  WDT->WDT_MR=WDT_MR_WDDIS;

  /* Disable pull-up on every pin */
#if 0
  for (unsigned i = 0; i < PINS_COUNT; i++)
  {
    digitalWrite(i, LOW);
  }
#endif // 0

  // Enable parallel access on PIO output data registers
#if 0
#ifdef PIOA
  PIOA->PIO_OWER = 0xFFFFFFFF;
#endif /* PIOA */
#ifdef PIOB
  PIOB->PIO_OWER = 0xFFFFFFFF;
#endif /* PIOB */
#ifdef PIOC
  PIOC->PIO_OWER = 0xFFFFFFFF;
#endif /* PIOC */
#ifdef PIOD
  PIOD->PIO_OWER = 0xFFFFFFFF;
#endif /* PIOD */
#endif // 0
}

#ifdef __cplusplus
}
#endif
