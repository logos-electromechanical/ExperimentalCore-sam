/*
  Copyright (c) 2015 Arduino LLC & Thibaut VIARD.  All right reserved.

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

#ifndef _ARDUINO_CORE_PRIVATE_
#define _ARDUINO_CORE_PRIVATE_

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

// Includes Atmel CMSIS headers
#include "sam.h"

#include "core_constants.h"
/*
 * \brief Allocates a pin to a specific peripheral (GPIO mutiplexer).
 *
 * \param ulPin         pin number
 * \param ulPeripheral  peripheral
 */

int pinPeripheral( uint32_t ulPin, EPioType ulPeripheral );

#ifdef __cplusplus
} // extern "C"

#include "HardwareSerial.h"

#endif // _ARDUINO_CORE_PRIVATE_