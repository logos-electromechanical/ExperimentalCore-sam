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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "CoreSerial.hpp"
#include "core_cortex_vectors.h"

// Constructors ////////////////////////////////////////////////////////////////

SAMSerial::SAMSerial(Usart *pUsart, IRQn_Type irq, void (*irq_handler)(void), uint32_t isUART)
{
  _pUsart=pUsart;
  _dwIrq=irq;
  _dwId=(uint32_t)irq;

  // Dynamic assignment of IRQ handler
  vectorAssign(irq, irq_handler);
}

// Public Methods //////////////////////////////////////////////////////////////

void SAMSerial::begin(const uint32_t ulBaudrate)
{
  init(ulBaudrate, SERIAL_8N1);
}

void SAMSerial::init(const uint32_t ulBaudrate, const uint32_t ulMode)
{
  uint32_t ulRegister=0;

  /* Activate Serial peripheral clock
   * All UART/USART peripheral ids are below 32, so on PCER0
   */
  PMC->PMC_PCER0 = 1 << _dwId;

#if SAMG55_SERIES
  ((Flexcom*)((uint32_t)_pUsart-(0x200)))->FLEXCOM_MR=FLEXCOM_MR_OPMODE_USART;
#endif

  // Disable PDC channel
  _pUsart->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;

  // Reset and disable receiver and transmitter
  _pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;

  // Configure mode
  switch ( ulMode & HARDSER_PARITY_MASK)
  {
    case HARDSER_PARITY_EVEN:
      ulRegister|=US_MR_PAR_EVEN;
    break;

    case HARDSER_PARITY_ODD:
      ulRegister|=US_MR_PAR_ODD;
    break;

    case HARDSER_PARITY_NONE:
      ulRegister|=US_MR_PAR_NO;
   break;
  }

  switch ( ulMode & HARDSER_STOP_BIT_MASK)
  {
    case HARDSER_STOP_BIT_1:
      ulRegister|=US_MR_NBSTOP_1_BIT;
    break;

    case HARDSER_STOP_BIT_1_5:
      ulRegister|=US_MR_NBSTOP_1_5_BIT;
    break;

    case HARDSER_STOP_BIT_2:
      ulRegister|=US_MR_NBSTOP_2_BIT;
    break;
  }

  /* UART has CHaracter Length fixed to 8bits */
  if ( _isUART == 0)
  {
    switch ( ulMode & HARDSER_DATA_MASK)
    {
      case HARDSER_DATA_5:
        ulRegister|=US_MR_CHRL_5_BIT;
      break;

      case HARDSER_DATA_6:
        ulRegister|=US_MR_CHRL_6_BIT;
      break;

      case HARDSER_DATA_7:
        ulRegister|=US_MR_CHRL_7_BIT;
      break;

      case HARDSER_DATA_8:
        ulRegister|=US_MR_CHRL_8_BIT;
      break;
    }
  }

  _pUsart->US_MR = ulRegister;

  // Configure baudrate (asynchronous, no oversampling)
  // CD = (Peripheral clock) / (baudrate * 16)
  _pUsart->US_BRGR = (SystemCoreClock / ulBaudrate) >> 4;

  // Configure interrupts
  _pUsart->US_IDR = 0xFFFFFFFF;
  _pUsart->US_IER = US_IER_RXRDY | US_IER_OVRE | US_IER_FRAME;

  // Enable UART interrupt in NVIC
  NVIC_EnableIRQ(_dwIrq);

  // Make sure both ring buffers are initialized back to empty.
  _rx_buffer.clear();
  _tx_buffer.clear();

  // Enable receiver and transmitter
  _pUsart->US_CR = US_CR_RXEN | US_CR_TXEN;
}

void SAMSerial::end( void )
{
  // Clear any received data
  _rx_buffer.clear();

  // Wait for any outstanding data to be sent
  flush();

  // Disable UART interrupt in NVIC
  NVIC_DisableIRQ( _dwIrq );

  /* Remove clock of Serial peripheral
   * All UART/USART peripheral ids are below 32, so on PCDR0
   */
  PMC->PMC_PCDR0 = 1 << _dwId;
}

void SAMSerial::setInterruptPriority(uint32_t priority)
{
  NVIC_SetPriority(_dwIrq, priority & 0x0F);
}

uint32_t SAMSerial::getInterruptPriority()
{
  return NVIC_GetPriority(_dwIrq);
}

int SAMSerial::available(void)
{
  return _rx_buffer.available();
}

int SAMSerial::availableForWrite(void)
{
  int head = _tx_buffer._iHead;
  int tail = _tx_buffer._iTail;

  if (head >= tail)
  {
    return SERIAL_BUFFER_SIZE - 1 - head + tail;
  }
  else
  {
    return tail - head - 1;
  }
}

int SAMSerial::peek( void )
{
  return _rx_buffer.peek();
}

int SAMSerial::read( void )
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ( _rx_buffer._iHead == _rx_buffer._iTail )
    return -1;

  uint8_t uc = _rx_buffer._aucBuffer[_rx_buffer._iTail];
  _rx_buffer._iTail = (unsigned int)(_rx_buffer._iTail + 1) % SERIAL_BUFFER_SIZE;

  return uc;
}

void SAMSerial::flush( void )
{
  while (_tx_buffer._iHead != _tx_buffer._iTail); //wait for transmit data to be sent
  // Wait for transmission to complete
  while ((_pUsart->US_CSR & US_CSR_TXRDY) != US_CSR_TXRDY)
   ;
}

size_t SAMSerial::write( const uint8_t uc_data )
{
  // Is the hardware currently busy?
  if (((_pUsart->US_CSR & US_CSR_TXRDY) != US_CSR_TXRDY) |
      (_tx_buffer._iTail != _tx_buffer._iHead))
  {
    // If busy we buffer
    unsigned int l = (_tx_buffer._iHead + 1) % SERIAL_BUFFER_SIZE;
    while (_tx_buffer._iTail == l)
      ; // Spin locks if we're about to overwrite the buffer. This continues once the data is sent

    _tx_buffer._aucBuffer[_tx_buffer._iHead] = uc_data;
    _tx_buffer._iHead = l;
    // Make sure TX interrupt is enabled
    _pUsart->US_IER = US_IER_TXRDY;
  }
  else
  {
     // Bypass buffering and send character directly
     _pUsart->US_THR = uc_data;
  }
  return 1;
}

void SAMSerial::IrqHandler( void )
{
  uint32_t status = _pUsart->US_CSR;

  // Did we receive data?
  if ((status & US_CSR_RXRDY) == US_CSR_RXRDY)
    _rx_buffer.store_char(_pUsart->US_RHR);

  // Do we need to keep sending data?
  if ((status & US_CSR_TXRDY) == US_CSR_TXRDY)
  {
    if (_tx_buffer._iTail != _tx_buffer._iHead)
    {
      _pUsart->US_THR = _tx_buffer._aucBuffer[_tx_buffer._iTail];
      _tx_buffer._iTail = (unsigned int)(_tx_buffer._iTail + 1) % SERIAL_BUFFER_SIZE;
    }
    else
    {
      // Mask off transmit interrupt so we don't get it anymore
      _pUsart->US_IDR = US_IDR_TXRDY;
    }
  }

  // Acknowledge errors
  if ((status & US_CSR_OVRE) == US_CSR_OVRE || (status & US_CSR_FRAME) == US_CSR_FRAME)
  {
    // TODO: error reporting outside ISR
    _pUsart->US_CR |= US_CR_RSTSTA;
  }
}

