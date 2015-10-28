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

#ifndef _WIRING_ANALOG_
#define _WIRING_ANALOG_

#ifdef __cplusplus
extern "C" {
#endif

// If we don't have all the things we need defined, define them here to prevent compilation errors
#ifndef PWM_RESOLUTION
#define PWM_RESOLUTION 		8
#endif
#ifndef TC_RESOLUTION
#define TC_RESOLUTION		8
#endif
#ifndef PWM_INTERFACE_ID
#define PWM_INTERFACE_ID 	ID_PWM
#endif
#ifndef TC_INTERFACE_ID
#define TC_INTERFACE_ID     ID_TC0
#endif
#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY		1000
#endif
#ifndef TC_FREQUENCY
#define TC_FREQUENCY		1000
#endif
#ifndef PWM_MAX_DUTY_CYCLE
#define PWM_MAX_DUTY_CYCLE	255
#endif
#ifndef TC_MAX_DUTY_CYCLE
#define TC_MAX_DUTY_CYCLE	255
#endif 

/*
 * \brief SAM3 products have only one reference for ADC
 */
typedef enum _eAnalogReference
{
  AR_DEFAULT,
} eAnalogReference ;

/*
 * \brief Configures the reference voltage used for analog input (i.e. the value used as the top of the input range).
 * This function is kept only for compatibility with existing AVR based API.
 *
 * \param ulMmode Should be set to AR_DEFAULT.
 */
extern void analogReference( eAnalogReference ulMode ) ;

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
extern void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;

/*
 * \brief Reads the value from the specified analog pin.
 *
 * \param ulPin
 *
 * \return Read value from selected pin, if no error.
 */
extern uint32_t analogRead( uint32_t ulPin ) ;

/*
 * \brief Set the resolution of analogRead return values. Default is 10 bits (range from 0 to 1023).
 *
 * \param res
 */
extern void analogReadResolution(int res);

/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
extern void analogWriteResolution(int res);

extern void analogOutputInit( void ) ;

#ifdef __cplusplus
}
#endif

#endif /* _WIRING_ANALOG_ */
