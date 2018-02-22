/*
 SamPins.cpp - Library for the Motate system
 http://github.com/synthetos/motate/

 Copyright (c) 2014 - 2016 Robert Giseburt

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

#include "MotatePins.h"

using namespace Motate;

namespace Motate
{
template<>
void PortHardware<'A'>::enableClock()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
}
template<>
void PortHardware<'B'>::enableClock()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
}
template<>
void PortHardware<'C'>::enableClock()
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
}
template<>
void PortHardware<'D'>::enableClock()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
}
template<>
void PortHardware<'F'>::enableClock()
{
	__HAL_RCC_GPIOF_CLK_ENABLE();
}
}

template<> uint32_t PortHardware<'A'>::_inverted = 0;
template<> uint32_t PortHardware<'B'>::_inverted = 0;
template<> uint32_t PortHardware<'C'>::_inverted = 0;
template<> uint32_t PortHardware<'D'>::_inverted = 0;

template<> _pinChangeInterrupt * PortHardware<'A'>::_firstInterrupt = nullptr;
template<> _pinChangeInterrupt * PortHardware<'B'>::_firstInterrupt = nullptr;
template<> _pinChangeInterrupt * PortHardware<'C'>::_firstInterrupt = nullptr;
template<> _pinChangeInterrupt * PortHardware<'D'>::_firstInterrupt = nullptr;

extern "C" void PIOA_Handler(void) {

/*	uint32_t isr = PIOA->PIO_ISR;

    _pinChangeInterrupt *current = PortHardware<'A'>::_firstInterrupt;
    while (current != nullptr) {
        if ((isr & current->pc_mask) && (current->interrupt_handler)) {
            current->interrupt_handler();
        }
        current = current->next;
    }

    NVIC_ClearPendingIRQ(PIOA_IRQn);
    */
}

extern "C" void EXTI0_IRQHandler(void)
{
	uint32_t isr = EXTI->PR;

	_pinChangeInterrupt *current = PortHardware<'A'>::_firstInterrupt;
	while (current != nullptr) {
		if ((isr & current->pc_mask) && (current->interrupt_handler)) {
			current->interrupt_handler();
		}
		current = current->next;
	}
	current = PortHardware<'B'>::_firstInterrupt;
	while (current != nullptr) {
		if ((isr & current->pc_mask) && (current->interrupt_handler)) {
			current->interrupt_handler();
		}
		current = current->next;
	}

	EXTI->PR = isr;
}

extern "C" void EXTI9_5_IRQHandler(void)
{
	uint32_t isr = EXTI->PR;

	_pinChangeInterrupt *current = PortHardware<'A'>::_firstInterrupt;
	while (current != nullptr) {
		if ((isr & current->pc_mask) && (current->interrupt_handler)) {
			current->interrupt_handler();
		}
		current = current->next;
	}
	current = PortHardware<'B'>::_firstInterrupt;
	while (current != nullptr) {
		if ((isr & current->pc_mask) && (current->interrupt_handler)) {
			current->interrupt_handler();
		}
		current = current->next;
	}

	EXTI->PR = isr;
}
extern "C" void EXTI15_10_IRQHandler(void)
{
	uint32_t isr = EXTI->PR;

	_pinChangeInterrupt *current = PortHardware<'A'>::_firstInterrupt;
	while (current != nullptr) {
		if ((isr & current->pc_mask) && (current->interrupt_handler)) {
			current->interrupt_handler();
		}
		current = current->next;
	}
	current = PortHardware<'B'>::_firstInterrupt;
	while (current != nullptr) {
		if ((isr & current->pc_mask) && (current->interrupt_handler)) {
			current->interrupt_handler();
		}
		current = current->next;
	}

	EXTI->PR = isr;
}




#ifdef ADC

ADC_HandleTypeDef	ADC_Module::AdcHandle;
uint32_t			ADC_Module::uhADCxConvertedValue[16];
uint32_t 			ADC_Module::numInitialized;
uint8_t				ADC_Module::rankToChannel[16];

/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hadc: ADC handle pointer
  * @retval None
  */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

/* Definition for ADCx's DMA */
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_0
#define ADCx_DMA_STREAM                 DMA2_Stream0

/* Definition for ADCx's NVIC */
#define ADCx_DMA_IRQn                   DMA2_Stream0_IRQn
#define ADCx_DMA_IRQHandler             DMA2_Stream0_IRQHandler

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/**
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
extern "C" void ADCx_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(ADC_Module::AdcHandle.DMA_Handle);
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{

  static DMA_HandleTypeDef  hdma_adc;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ADC3 Periph clock enable */
  __HAL_RCC_ADC1_CLK_ENABLE();
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* Enable DMA2 clock */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /*##-3- Configure the DMA streams ##########################################*/
  /* Set the parameters to be configured */
  hdma_adc.Instance = ADCx_DMA_STREAM;

  hdma_adc.Init.Channel  = ADCx_DMA_CHANNEL;
  hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_adc.Init.Mode = DMA_CIRCULAR;
  hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_adc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_adc.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_adc.Init.PeriphBurst = DMA_PBURST_SINGLE;

  HAL_DMA_Init(&hdma_adc);

  /* Associate the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

}

/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{

  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */

}

extern "C" {
    void _null_adc_pin_interrupt() __attribute__ ((unused));
    void _null_adc_pin_interrupt() {};
}

namespace Motate {
    bool ADC_Module::inited_ = false;

    /*
    template<> void ADCPin< LookupADCPinByADC< 0>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    */
    /*
    template<> void ADCPin< LookupADCPinByADC< 2>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC< 3>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC<15>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
     */
    /*
    template<> void ADCPin< LookupADCPinByADC< 5>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC< 6>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC< 7>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC< 8>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC< 9>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC<10>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC<11>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC<12>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC<13>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    template<> void ADCPin< LookupADCPinByADC<14>::number >::interrupt() __attribute__ ((weak, alias("_null_adc_pin_interrupt")));
    */
}
/*
extern "C" void ADC_Handler(void) {

    uint32_t isr = ADC->ADC_ISR; // read it to clear the ISR

//    uint32_t adc_value = ADC->ADC_LCDR;
//    uint32_t adc_num  = (adc_value & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos;
//    adc_value = (adc_value & ADC_LCDR_LDATA_Msk) >> ADC_LCDR_LDATA_Pos;

#define _INTERNAL_MAKE_ADC_CHECK(num) \
if (ADCPin< LookupADCPinByADC<num>::number >::interrupt) { \
    if ((isr & LookupADCPinByADC<num>::adcMask)) { \
        LookupADCPinByADC<num>::interrupt(); \
    } \
}

    _INTERNAL_MAKE_ADC_CHECK( 0)
    _INTERNAL_MAKE_ADC_CHECK( 1)
    _INTERNAL_MAKE_ADC_CHECK( 2)
    _INTERNAL_MAKE_ADC_CHECK( 3)
    _INTERNAL_MAKE_ADC_CHECK( 4)
    _INTERNAL_MAKE_ADC_CHECK( 5)
    _INTERNAL_MAKE_ADC_CHECK( 6)
    _INTERNAL_MAKE_ADC_CHECK( 7)
    _INTERNAL_MAKE_ADC_CHECK( 8)
    _INTERNAL_MAKE_ADC_CHECK( 9)
    _INTERNAL_MAKE_ADC_CHECK(10)
    _INTERNAL_MAKE_ADC_CHECK(11)
    _INTERNAL_MAKE_ADC_CHECK(12)
    _INTERNAL_MAKE_ADC_CHECK(13)
    _INTERNAL_MAKE_ADC_CHECK(14)

//    NVIC_ClearPendingIRQ(ADC_IRQn);


}
*/

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and
  *         you can add your own implementation.
  * @retval None
  */
extern void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
#define _INTERNAL_MAKE_ADC_CHECK(num) \
if (ADCPin< LookupADCPinByADC<num>::number >::interrupt) { \
	if (ADC_Module::rankToChannel[num] != 0xFF) { \
		LookupADCPinByADC<num>::interrupt(); \
	} \
}


	_INTERNAL_MAKE_ADC_CHECK( 2)
	_INTERNAL_MAKE_ADC_CHECK( 3)
	_INTERNAL_MAKE_ADC_CHECK( 9)
	_INTERNAL_MAKE_ADC_CHECK(15)

}

#endif // ADC
