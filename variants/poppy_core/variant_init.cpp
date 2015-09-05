/*
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

#ifndef _ARDUINO_CORE_INIT_H_
#define _ARDUINO_CORE_INIT_H_

void initVariant( void )
{
  // Initialize Serial port Flexcom6 pins
  pinPeripheral(PIN_SERIAL_RX, GPIO_PERIPH_B);
  digitalWrite(PIN_SERIAL_RX, HIGH); // Enable pullup for RXD6
  pinPeripheral(PIN_SERIAL_TX, GPIO_PERIPH_B);

  // TODO: move this to Serial class!
  FLEXCOM6->FLEXCOM_MR=FLEXCOM_MR_OPMODE_USART;
}

#endif // _ARDUINO_CORE_INIT_H_
