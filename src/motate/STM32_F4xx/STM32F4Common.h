/*
 utility/SamCommon.h - Library for the Motate system
 http://github.com/synthetos/motate/

 Copyright (c) 2013 - 2016 Robert Giseburt

 This file is part of the Motate Library.

 This file ("the software") is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License, version 2 as published by the
 Free Software Foundation. You should have received a copy of the GNU General Public
 License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.

 As a special exception, you may use this file as part of a software library without
 restriction. Specifically, if other files instantiate templates or use macros or
 inline functions from this file, or you compile this file and link it with  other
 files to produce an executable, this file does not by itself cause the resulting
 executable to be covered by the GNU General Public License. This exception does not
 however invalidate any other reasons why the executable file might be covered by the
 GNU General Public License.

 THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef SAMCOMMON_H_ONCE
#define SAMCOMMON_H_ONCE

#include "stm32f4xx.h"


/******************************************************************************
 *
 */

static uint16_t InterruptDisabledCount = 0;
typedef uint32_t irqflags_t;

static inline irqflags_t cpu_irq_save(void) {
	/* Disable interrupts */
	__disable_irq();

	/* Increase number of disable interrupt function calls */
	InterruptDisabledCount++;
	return InterruptDisabledCount;
}

static inline void cpu_irq_restore(irqflags_t flags) {
	/* Decrease number of disable interrupt function calls */
	if (InterruptDisabledCount) {
		InterruptDisabledCount--;
	}

	/* Check if we are ready to enable interrupts */
	if (!InterruptDisabledCount) {
		/* Enable interrupts */
		__enable_irq();
	}

}

static inline void cpu_irq_disable(void) {
	/* Disable interrupts */
	__disable_irq();

	/* Increase number of disable interrupt function calls */
	InterruptDisabledCount++;
}

static inline uint8_t cpu_irq_enable(void) {
	/* Decrease number of disable interrupt function calls */
	if (InterruptDisabledCount) {
		InterruptDisabledCount--;
	}

	/* Check if we are ready to enable interrupts */
	if (!InterruptDisabledCount) {
		/* Enable interrupts */
		__enable_irq();
	}
	/* Return interrupt enabled status */
	return !InterruptDisabledCount;

}


namespace Motate {



struct STM32F4Common {
    static inline void sync() { __DMB(); };

    static void enablePeripheralClock(uint32_t peripheralId) {

    };

    static void disablePeripheralClock(uint32_t peripheralId) {

    };

    static uint32_t getPeripheralClockFreq() {

        return SystemCoreClock;
    };

    struct InterruptDisabler {
        volatile uint32_t flags;
        InterruptDisabler() : flags{__get_PRIMASK()} {
            __disable_irq();
            sync();
        };
        ~InterruptDisabler() {
            sync();
            __enable_irq();
         };
    };
};

}  // namespace Motate

#endif /* end of include guard: SAMCOMMON_H_ONCE */
