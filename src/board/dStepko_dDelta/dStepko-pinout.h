/*
 * g2v9k-pinout.h - board pinout specification
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2016 Robert Giseburt
 * Copyright (c) 2013 - 2016 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef g2v9_pinout_h
#define g2v9_pinout_h

/*
 * USAGE NOTES
 *
 * Read this first:
 * https://github.com/synthetos/g2/wiki/Adding-a-new-G2-board-(or-revision)-to-G2#making-a-new-pin-assignment
 *
 *  USAGE:
 *
 *  This file is lays out all the pin capabilities of the SAM3X8C organized by pin number.
 *  Each pin has its associated functions listed at the bottom of the file, and is essentially
 *  immutable for each processor.
 *
 *  To use, assign Motate pin numbers to the first value in the _MAKE_MOTATE_PIN() macro.
 *  ALL PINS MUST BE ASSIGNED A NUMBER, even if they are not used. There will NOT be a
 *  code-size or speed penalty for unused pins, but the WILL be a compiler-failure for
 *  unassigned pins. This new restriction allows for simplification of linkages deep in
 *  Motate.
 */
/*  See motate_pin_assignments.h for pin names to be used int he rest of the G2 code.
 *  EXAMPLES:
 *
 *  *** Vanilla pin example ***
 *
 *      _MAKE_MOTATE_PIN(4, A, 'A', 27);	// SPI0_SCKPinNumber
 *
 *      This assigns Motate pin 4 to Port A, pin 27 (A27)
 *      Look in motate_pin_assignments.h to see that this is kSPI_SCKPinNumber
 *
 *  ** Other pin functions ***
 *
 *      Please look in <Motate>/platform/atmel_sam/motate_chip_pin_functions.h
 */


#include <MotatePins.h>

// We don't have all of the inputs, so we don't define them.
#define INPUT1_AVAILABLE 1
#define INPUT2_AVAILABLE 1
#define INPUT3_AVAILABLE 1
#define INPUT4_AVAILABLE 1
#define INPUT5_AVAILABLE 1
#define INPUT6_AVAILABLE 1
#define INPUT7_AVAILABLE 1
#define INPUT8_AVAILABLE 1
#define INPUT9_AVAILABLE 0
#define INPUT10_AVAILABLE 0
#define INPUT11_AVAILABLE 0
#define INPUT12_AVAILABLE 0
#define INPUT13_AVAILABLE 0

#define ADC0_AVAILABLE 0
#define ADC1_AVAILABLE 0
#define ADC2_AVAILABLE 0
#define ADC3_AVAILABLE 0

#define XIO_HAS_USB 1
#define XIO_HAS_UART 0
#define XIO_HAS_SPI 0
#define XIO_HAS_I2C 0

#define TEMPERATURE_OUTPUT_ON 0

// Some pins, if the PWM capability is turned on, it will cause timer conflicts.
// So we have to explicity enable them as PWM pins.
// Generated with:
// perl -e 'for($i=1;$i<14;$i++) { print "#define OUTPUT${i}_PWM 0\n";}'
#define OUTPUT1_PWM 1
#define OUTPUT2_PWM 0
#define OUTPUT3_PWM 0
#define OUTPUT4_PWM 0
#define OUTPUT5_PWM 0
#define OUTPUT6_PWM 0
#define OUTPUT7_PWM 0
#define OUTPUT8_PWM 0
#define OUTPUT9_PWM 0
#define OUTPUT10_PWM 0
#define OUTPUT11_PWM 1
#define OUTPUT12_PWM 0
#define OUTPUT13_PWM 0

namespace Motate {

// Pin name and function
//
_MAKE_MOTATE_PIN(kSocket1_StepPinNumber, 'D', 2);            // Socket1_StepPinNumber
_MAKE_MOTATE_PIN(kSocket1_DirPinNumber, 'C', 12);              // Socket1_DirPinNumber
_MAKE_MOTATE_PIN(kSocket1_EnablePinNumber, 'C', 10);          // Socket1_EnablePinNumber

_MAKE_MOTATE_PIN(kSocket2_StepPinNumber, 'A', 15);            // Socket2_StepPinNumber
_MAKE_MOTATE_PIN(kSocket2_DirPinNumber, 'A', 10);              // Socket2_DirPinNumber
_MAKE_MOTATE_PIN(kSocket2_EnablePinNumber, 'C', 9);          // Socket2_EnablePinNumber

_MAKE_MOTATE_PIN(kSocket3_StepPinNumber, 'C', 8);            // Socket3_StepPinNumber
_MAKE_MOTATE_PIN(kSocket3_DirPinNumber, 'C', 7);              // Socket3_DirPinNumber
_MAKE_MOTATE_PIN(kSocket3_EnablePinNumber, 'B', 15);          // Socket3_EnablePinNumber

_MAKE_MOTATE_PIN(kSocket4_StepPinNumber, 'B', 14);            // Socket4_StepPinNumber
_MAKE_MOTATE_PIN(kSocket4_DirPinNumber, 'B', 13);              // Socket4_DirPinNumber
_MAKE_MOTATE_PIN(kSocket4_EnablePinNumber, 'B', 10);          // Socket4_EnablePinNumber

_MAKE_MOTATE_PIN(kLED_USBRXPinNumber, 'A', 2);

/*
_MAKE_MOTATE_PIN(kADC1_PinNumber, 'C', 5);
_MAKE_MOTATE_PIN(kADC0_PinNumber, 'A', 3);

_MAKE_MOTATE_ADC_PIN('C',  5, 15);
_MAKE_MOTATE_ADC_PIN('A',  3, 3);

_MAKE_MOTATE_PIN(kOutput14_PinNumber, 'A',  15);
_MAKE_MOTATE_PIN(kOutput16_PinNumber, 'C',  3);


typedef TimerChannel<2,1> parentTimerType;
template<>
struct AvailablePWMOutputPin<ReversePinLookup<'A', 15>::number> : RealPWMOutputPin< ReversePinLookup<'A', 15>::number, parentTimerType>
{
	static const pin_number pinNum = ReversePinLookup<'A', 15>::number;
	AvailablePWMOutputPin() : RealPWMOutputPin<pinNum, parentTimerType>(kOutputPWM)
	{
		pwmpin_init(true ? kPWMOnInverted : kPWMOn);
	};
	AvailablePWMOutputPin(const PinOptions_t options, const uint32_t freq) : RealPWMOutputPin<pinNum, parentTimerType>(kOutputPWM, options, freq)
	{
		pwmpin_init((true ^ ((options & kPWMPinInverted)?true:false)) ? kPWMOnInverted : kPWMOn);
	};
	using RealPWMOutputPin<pinNum, parentTimerType>::operator=;
	// Signal to _GetAvailablePWMOrAlike that we're here, AND a real Pin<> exists.
	static constexpr bool _isAvailable()
	{
		return !ReversePinLookup<'A', 15>::isNull();
	};
};
*/

_MAKE_MOTATE_PIN(kSpindle_PwmPinNumber, 'B',  9);


typedef TimerChannel<2,2> parentTimerType;
template<>
struct AvailablePWMOutputPin<ReversePinLookup<'B', 9>::number> : RealPWMOutputPin< ReversePinLookup<'B', 9>::number, parentTimerType>
{
	static const pin_number pinNum = ReversePinLookup<'B', 9>::number;
	AvailablePWMOutputPin() : RealPWMOutputPin<pinNum, parentTimerType>(kOutputPWM)
	{
		pwmpin_init(false ? kPWMOnInverted : kPWMOn);
	};
	AvailablePWMOutputPin(const PinOptions_t options, const uint32_t freq) : RealPWMOutputPin<pinNum, parentTimerType>(kOutputPWM, options, freq)
	{
		pwmpin_init((false ^ ((options & kPWMPinInverted)?true:false)) ? kPWMOnInverted : kPWMOn);
	};
	using RealPWMOutputPin<pinNum, parentTimerType>::operator=;
	// Signal to _GetAvailablePWMOrAlike that we're here, AND a real Pin<> exists.
	static constexpr bool _isAvailable()
	{
		return !ReversePinLookup<'B', 9>::isNull();
	};
};


}  // namespace Motate


#endif
