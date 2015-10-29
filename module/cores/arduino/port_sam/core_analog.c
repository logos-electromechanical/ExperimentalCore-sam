/*
 Copyright (c) 2011 Arduino LLC.  All right reserved.

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
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

static int _readResolution = 10;
static int _writeResolution = 8;

eAnalogReference analog_reference = AR_DEFAULT;
static uint8_t 	PWMEnabled = 0;
static uint8_t 	pinEnabled[PINS_COUNT];
static uint8_t 	TCChanEnabled[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static bool 	ADCenabled = false;

static uint16_t FindClockConfiguration(uint32_t frequency, uint32_t mck);

void analogReadResolution(int res)
{
  _readResolution = res;
}

void analogWriteResolution(int res)
{
  _writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to)
    return value;

  if (from > to)
    return value >> (from-to);
  else
    return value << (to-from);
}

void analogReference(eAnalogReference ulMode)
{
  analog_reference = ulMode;
}

uint32_t analogRead(uint32_t ulPin)
{
  uint32_t ulValue = 0;

  uint32_t ulChannel;

//  if (ulPin < A0)
//    ulPin += A0;

  ulChannel = g_aPinMap[ulPin].ulADCChannelNumber ;
  
#if (defined ADC)
	if (!ADCenabled) {
		ADC->ADC_CR = ADC_CR_SWRST;								// Reset the controller
		ADC->ADC_MR = 0;										// Reset mode register
		ADC->ADC_PTCR = (ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS);	// Reset PDC transfer
		ADC->ADC_RCR = 0;
		ADC->ADC_RNCR = 0;
		#if (defined ADC_PRESCALER)
		ADC->ADC_MR |= ADC_MR_PRESCAL(ADC_PRESCALER);
		#endif
		#if (defined ADC_STARTUP)
		ADC->ADC_MR |= ((ADC_STARTUP << ADC_MR_STARTUP_Pos) & ADC_MR_STARTUP_Msk);
		#endif
		#if (defined ADC_TRACKING)
		ADC->ADC_MR |= ADC_MR_TRACKTIM(ADC_TRACKING);
		#endif
		ADCenabled = true;
	}
	ADC->ADC_CHER = 1 << ulChannel;	// enable channel
	ADC->ADC_CR = ADC_CR_START;		// start conversion
	while ((ADC->ADC_ISR & ADC_ISR_DRDY) != ADC_ISR_DRDY);		// wait for conversion to complete
	ulValue = ADC->ADC_LCDR;
	ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);
	ADC->ADC_CHDR = 1 << ulChannel;	// disable channel
#endif

#if defined __SAM3U4E__
  switch ( g_aPinMap[ulPin].ulAnalogChannel )
  {
    // Handling ADC 10/12 bits channels
    case ADC0 :
    case ADC1 :
    case ADC2 :
    case ADC3 :
    case ADC4 :
    case ADC5 :
    case ADC6 :
    case ADC7 :
      // Enable the corresponding channel
      adc_enable_channel( ADC, ulChannel );

      // Start the ADC
      adc_start( ADC );

      // Wait for end of conversion
      while ((adc_get_status(ADC) & ADC_SR_DRDY) != ADC_SR_DRDY)
        ;

      // Read the value
      ulValue = adc_get_latest_value(ADC);
      ulValue = mapResolution(ulValue, 10, _readResolution);

      // Disable the corresponding channel
      adc_disable_channel( ADC, ulChannel );

      // Stop the ADC
      //      adc_stop( ADC ) ; // never do adc_stop() else we have to reconfigure the ADC each time
      break;

    // Handling ADC 12 bits channels
    case ADC8 :
    case ADC9 :
    case ADC10 :
    case ADC11 :
    case ADC12 :
    case ADC13 :
    case ADC14 :
    case ADC15 :
      // Enable the corresponding channel
      adc12b_enable_channel( ADC12B, ulChannel );

      // Start the ADC12B
      adc12b_start( ADC12B );

      // Wait for end of conversion
      while ((adc12b_get_status(ADC12B) & ADC12B_SR_DRDY) != ADC12B_SR_DRDY)
        ;

      // Read the value
      ulValue = adc12b_get_latest_value(ADC12B) >> 2;
      ulValue = mapResolution(ulValue, 12, _readResolution);

      // Stop the ADC12B
      //      adc12_stop( ADC12B ) ; // never do adc12_stop() else we have to reconfigure the ADC12B each time

      // Disable the corresponding channel
      adc12b_disable_channel( ADC12B, ulChannel );
      break;

    // Compiler could yell because we don't handle DAC pins
    default :
      ulValue=0;
      break;
  }
#endif

#if defined __SAM3X8E__ || defined __SAM3X8H__
  static uint32_t latestSelectedChannel = -1;
  switch ( g_aPinMap[ulPin].ulAnalogChannel )
  {
    // Handling ADC 12 bits channels
    case ADC0 :
    case ADC1 :
    case ADC2 :
    case ADC3 :
    case ADC4 :
    case ADC5 :
    case ADC6 :
    case ADC7 :
    case ADC8 :
    case ADC9 :
    case ADC10 :
    case ADC11 :

      // Enable the corresponding channel
      if (ulChannel != latestSelectedChannel) {
        adc_enable_channel( ADC, ulChannel );
        if ( latestSelectedChannel != (uint32_t)-1 )
          adc_disable_channel( ADC, latestSelectedChannel );
        latestSelectedChannel = ulChannel;
      }

      // Start the ADC
      adc_start( ADC );

      // Wait for end of conversion
      while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY)
        ;

      // Read the value
      ulValue = adc_get_latest_value(ADC);
      ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);

      break;

    // Compiler could yell because we don't handle DAC pins
    default :
      ulValue=0;
      break;
  }
#endif

  return ulValue;
}

void analogOutputInit(void)
{
  uint8_t i;

  for (i=0; i<PINS_COUNT; i++)
  {
    pinEnabled[i] = 0;
  }
}

static void analogWriteDAC(uint32_t ulPin, uint32_t ulValue)
{
#if 0
  EAnalogChannel channel = g_aPinMap[ulPin].ulADCChannelNumber;
  if (channel == DA0 || channel == DA1)
  {
    uint32_t chDACC = ((channel == DA0) ? 0 : 1);
    if (dacc_get_channel_status(DACC_INTERFACE) == 0)
    {
      /* Enable clock for DACC_INTERFACE */
      pmc_enable_periph_clk(DACC_INTERFACE_ID);

      /* Reset DACC registers */
      dacc_reset(DACC_INTERFACE);

      /* Half word transfer mode */
      dacc_set_transfer_mode(DACC_INTERFACE, 0);

      /* Power save:
       * sleep mode  - 0 (disabled)
       * fast wakeup - 0 (disabled)
       */
      dacc_set_power_save(DACC_INTERFACE, 0, 0);
      /* Timing:
       * refresh        - 0x08 (1024*8 dacc clocks)
       * max speed mode -    0 (disabled)
       * startup time   - 0x10 (1024 dacc clocks)
       */
      dacc_set_timing(DACC_INTERFACE, 0x08, 0, 0x10);

      /* Set up analog current */
      dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02) |
                    DACC_ACR_IBCTLCH1(0x02) |
                    DACC_ACR_IBCTLDACCORE(0x01));
    }

    /* Disable TAG and select output channel chDACC */
    dacc_set_channel_selection(DACC_INTERFACE, chDACC);

    if ((dacc_get_channel_status(DACC_INTERFACE) & (1 << chDACC)) == 0)
    {
      dacc_enable_channel(DACC_INTERFACE, chDACC);
    }

    // Write user value
    ulValue = mapResolution(ulValue, _writeResolution, DACC_RESOLUTION);
    dacc_write_conversion_data(DACC_INTERFACE, ulValue);
    while ((dacc_get_interrupt_status(DACC_INTERFACE) & DACC_ISR_EOC) == 0);
    return;
  }
#endif // 0
}

static void analogWritePWM(uint32_t ulPin, uint32_t ulValue)
{
  ulValue = mapResolution(ulValue, _writeResolution, PWM_RESOLUTION);

  if (!PWMEnabled)
  {
#if (defined _SAM4S_)
    // PWM Startup code
    PMC->PMC_PCER0 = 1 << PWM_INTERFACE_ID;		// start the PWM peripheral clock
	uint32_t result = FindClockConfiguration(PWM_FREQUENCY * PWM_MAX_DUTY_CYCLE, VARIANT_MCK);
    assert( result != 0 );
	PWM_INTERFACE->PWM_CLK = result;
	PWMEnabled = 1;
#endif
  }

  uint32_t chan = g_aPinMap[ulPin].ulPWMChannel;
  if (!pinEnabled[ulPin])
  {
#if (defined _SAM4S_)
	Pio* port = Ports[g_aPinMap[ulPin].iPort].pGPIO;
	uint32_t dwSR0 = port->PIO_ABCDSR[0];
	uint32_t dwSR1 = port->PIO_ABCDSR[1];
    // Setup PWM for this pin
	switch (g_aPinMap[ulPin].ulPWMChannel) {
		// This nested switch is an ugly hack, but I can't think of a better way that doesn't involved major surgery

		case (PWM_CHL0):
			switch (g_aPinMap[ulPin].ulPin) {
				case (PIO_PA11):
				#if (defined PIO_PA23)
				case (PIO_PA23):	// Peripheral B
				#endif
					port->PIO_ABCDSR[0] &= (g_aPinMap[ulPin].ulPin | dwSR0);
					port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
					break;
				case (PIO_PB0|PIO_PA0):		// Peripheral A
					port->PIO_ABCDSR[0] &= (~g_aPinMap[ulPin].ulPin & dwSR0);
					port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
					break;
				default:
					return;
			}
			break;
		case (PWM_CHL1):
			switch (g_aPinMap[ulPin].ulPin) {
				case (PIO_PA12):
				#if (defined PIO_PA24)
				case (PIO_PA24):	// Peripheral B
				#endif
					port->PIO_ABCDSR[0] &= (g_aPinMap[ulPin].ulPin | dwSR0);
					port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
					break;
				case (PIO_PB1|PIO_PA1):		// Peripheral A
					port->PIO_ABCDSR[0] &= (~g_aPinMap[ulPin].ulPin & dwSR0);
					port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
					break;
				default:
					return;
			}
			break;
		case (PWM_CHL2):
			switch (g_aPinMap[ulPin].ulPin) {
				case (PIO_PA13):
				#if (defined PIO_PA25)
				case (PIO_PA25):
				#endif
				case (PIO_PB4):		// Peripheral B
					port->PIO_ABCDSR[0] &= (g_aPinMap[ulPin].ulPin | dwSR0);
					port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
					break;
				case (PIO_PA2):		// Peripheral A
					port->PIO_ABCDSR[0] &= (~g_aPinMap[ulPin].ulPin & dwSR0);
					port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
					break;
				default:
					return;
			}
			break;
		case (PWM_CHL3):
			switch (g_aPinMap[ulPin].ulPin) {
				case (PIO_PA7):
				#if (defined PIO_PB14)
				case (PIO_PB14|PIO_PA14):
				#else
				case (PIO_PA14):		// Peripheral B
				#endif
					port->PIO_ABCDSR[0] &= (g_aPinMap[ulPin].ulPin | dwSR0);
					port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
					break;
				case (PIO_PA17):	// Peripheral C
					port->PIO_ABCDSR[0] &= (~g_aPinMap[ulPin].ulPin & dwSR0);
					port->PIO_ABCDSR[1] &= (g_aPinMap[ulPin].ulPin | dwSR1);
					break;
				default:
					return;
			}
			break;

		default:
			return;
	}
	port->PIO_IDR = g_aPinMap[ulPin].ulPin;								// disable interrupts
	PWM_INTERFACE->PWM_CH_NUM[chan].PWM_CMR = PWM_CMR_CPRE_CLKA;		// configure the PWM channel
    PWM_INTERFACE->PWM_CH_NUM[chan].PWM_CPRD = PWM_MAX_DUTY_CYCLE;		// set the PWM period
    assert(ulValue <= PWM_INTERFACE->PWM_CH_NUM[chan].PWM_CPRD);		// check the duty cycle is in bounds
	PWM_INTERFACE->PWM_CH_NUM[chan].PWM_CDTY = ulValue;					// set duty cycl
    PWM_INTERFACE->PWM_ENA = 1 << chan;									// enable the channel
    pinEnabled[ulPin] = 1;
#endif
  }
#if (defined PWM_INTERFACE)
  assert(ulValue <= PWM_INTERFACE->PWM_CH_NUM[chan].PWM_CPRD);
  PWM_INTERFACE->PWM_CH_NUM[chan].PWM_CDTYUPD = ulValue;
#endif
}

static void analogWriteTimer(uint32_t ulPin, uint32_t ulValue)
{
  // We use MCLK/2 as clock.
  const uint32_t TC = VARIANT_MCK / 2 / TC_FREQUENCY;
  
  // channel mappings
  static const uint32_t channelToChNo[] = { 
	0, 0, 1, 1, 2, 2 
	#if (defined TC1)
	, 0, 0, 1, 1, 2, 2
	#endif 
	#if (defined TC2)
	, 0, 0, 1, 1, 2, 2
	#endif
	};
  static const uint32_t channelToAB[]   = { 
	1, 0, 1, 0, 1, 0 
	#if (defined TC1)
	, 1, 0, 1, 0, 1, 0 
	#endif 
	#if (defined TC2)
	, 1, 0, 1, 0, 1, 0 
	#endif
	};
  static Tc *channelToTC[] = {
    TC0, TC0, TC0, TC0, TC0, TC0
	#if (defined TC1)
    , TC1, TC1, TC1, TC1, TC1, TC1
	#endif 
	#if (defined TC2)
    , TC2, TC2, TC2, TC2, TC2, TC2 
	#endif
	};
  static const uint32_t channelToId[] = { 
	0, 0, 1, 1, 2, 2
	#if (defined TC1) 
	, 3, 3, 4, 4, 5, 5 
	#endif 
	#if (defined TC2)
	, 6, 6, 7, 7, 8, 8 
	#endif
	};

  // Map value to Timer ranges 0..255 => 0..TC
  ulValue = mapResolution(ulValue, _writeResolution, TC_RESOLUTION);
  ulValue = ulValue * TC;
  ulValue = ulValue / TC_MAX_DUTY_CYCLE;

  // Setup Timer for this pin
  ETimerChannel channel = g_aPinMap[ulPin].ulTimerChannel;
  uint32_t chNo = channelToChNo[channel];
  uint32_t chA  = channelToAB[channel];
  Tc *chTC = channelToTC[channel];
  uint32_t interfaceID = channelToId[channel];
  assert( chNo < (sizeof( chTC->TC_CHANNEL )/sizeof( chTC->TC_CHANNEL[0] )) ) ;
  TcChannel* pTcCh = chTC->TC_CHANNEL+chNo;
  
  if (!TCChanEnabled[interfaceID])
  {
#if (defined _SAM4S_)
	PMC->PMC_PCER0 = 1 << (TC_INTERFACE_ID + interfaceID);	// configure the TC peripheral clock
	pTcCh->TC_CCR = TC_CCR_CLKDIS ;							// disable TC clock
	pTcCh->TC_IDR = 0xFFFFFFFF ;							// disable interrupts
	pTcCh->TC_SR ;											// clear status register
	pTcCh->TC_CMR =											// set mode
      (TC_CMR_TCCLKS_TIMER_CLOCK1 |
      TC_CMR_WAVE |         // Waveform mode
      TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
      TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
      TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
      TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
	chTC->TC_CHANNEL[chNo].TC_RC = TC;
#endif
  }

  if (ulValue == 0)
  {
#if (defined _SAM4S_)
    if (chA)
      chTC->TC_CHANNEL[chNo].TC_CMR = (chTC->TC_CHANNEL[chNo].TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR;
    else
      chTC->TC_CHANNEL[chNo].TC_CMR = (chTC->TC_CHANNEL[chNo].TC_CMR & 0xF0FFFFFF) | TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR;
  }
  else
  {
    if (chA)
    {
	  chTC->TC_CHANNEL[chNo].TC_RA = ulValue;
      chTC->TC_CHANNEL[chNo].TC_CMR = (chTC->TC_CHANNEL[chNo].TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET;
    }
    else
    {
      chTC->TC_CHANNEL[chNo].TC_RB = ulValue;
      chTC->TC_CHANNEL[chNo].TC_CMR = (chTC->TC_CHANNEL[chNo].TC_CMR & 0xF0FFFFFF) | TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET;
    }
#endif
  }

  if (!pinEnabled[ulPin])
  {
#if (defined _SAM4S_)
    Pio* port = Ports[g_aPinMap[ulPin].iPort].pGPIO;
	uint32_t dwSR0 = port->PIO_ABCDSR[0];
	uint32_t dwSR1 = port->PIO_ABCDSR[1];
	port->PIO_ABCDSR[0] &= (g_aPinMap[ulPin].ulPin | dwSR0);
	port->PIO_ABCDSR[1] &= (~g_aPinMap[ulPin].ulPin & dwSR1);
	port->PIO_IDR = g_aPinMap[ulPin].ulPin;								// disable interrupts
    pinEnabled[ulPin] = 1;
#endif
  }

  if (!TCChanEnabled[interfaceID])
  {
#if (defined _SAM4S_)
    pTcCh->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;						// start timer/counter
    TCChanEnabled[interfaceID] = 1;
#endif
  }
}

/* PWM output only works on the pins with hardware support.
 * These are defined in the appropriate variant.cpp file.
 * For the rest of the pins, we default to digital output.
 */
void analogWrite(uint32_t ulPin, uint32_t ulValue)
{
  if (g_aPinMap[ulPin].ulADCChannelNumber != NOT_ON_ANALOG)
  {
    analogWriteDAC(ulPin, ulValue);
  }
  else
  {
    if (g_aPinMap[ulPin].ulPWMChannel != NOT_ON_PWM)
    {
      analogWritePWM(ulPin, ulValue);
    }
    else
    {
      if (g_aPinMap[ulPin].ulTimerChannel != NOT_ON_TIMER)
      {
        analogWriteTimer(ulPin, ulValue);
      }
      else // Defaults to digital write
      {
        pinMode(ulPin, OUTPUT);
        ulValue = mapResolution(ulValue, _writeResolution, 8);
        if (ulValue < 128)
        {
          digitalWrite(ulPin, LOW);
        }
        else
        {
          digitalWrite(ulPin, HIGH);
        }
      }
    }
  }
}

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Finds a prescaler/divisor couple to generate the desired frequency
 * from MCK.
 *
 * Returns the value to enter in PWM_CLK or 0 if the configuration cannot be
 * met.
 *
 * \param frequency  Desired frequency in Hz.
 * \param mck  Master clock frequency in Hz.
 */
static uint16_t FindClockConfiguration(
    uint32_t frequency,
    uint32_t mck)
{
    uint32_t divisors[11] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
    uint8_t divisor = 0;
    uint32_t prescaler;

    assert(frequency < mck);

    /* Find prescaler and divisor values */
    prescaler = (mck / divisors[divisor]) / frequency;
    while ((prescaler > 255) && (divisor < 11)) {

        divisor++;
        prescaler = (mck / divisors[divisor]) / frequency;
    }

    /* Return result */
    if ( divisor < 11 )
    {
//        TRACE_DEBUG( "Found divisor=%u and prescaler=%u for freq=%uHz\n\r", divisors[divisor], prescaler, frequency ) ;

        return prescaler | (divisor << 8) ;
    }
    else
    {
        return 0 ;
    }
}

#ifdef __cplusplus
}
#endif
