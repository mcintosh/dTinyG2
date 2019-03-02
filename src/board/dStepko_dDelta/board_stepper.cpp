/*
 * board_stepper.cpp - board-specific code for stepper.cpp
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 Alden S. Hart, Jr.
 * Copyright (c) 2016 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
 */

#include "../dStepko/board_stepper.h"

#include "stm32f4xx_hal.h"


/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
extern "C" void Error_Handler(void);

extern "C"
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();

	  /*Configure GPIO pins : TMC1_CS C11 */
	  GPIO_InitStruct.Pin = GPIO_PIN_11;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : TMC2_CS A8 */
	  GPIO_InitStruct.Pin = GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : TMC3_CS C6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : TMC3_CS B12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    //Error_Handler();
  }

}


// These are identical to board_stepper.h, except for the word "extern"
StepDirStepper<Motate::kSocket1_StepPinNumber,
               Motate::kSocket1_DirPinNumber,
               Motate::kSocket1_EnablePinNumber,
               Motate::kSocket1_Microstep_0PinNumber,
               Motate::kSocket1_Microstep_1PinNumber,
               Motate::kSocket1_Microstep_2PinNumber,
               Motate::kSocket1_VrefPinNumber>
    motor_1{};

StepDirStepper<Motate::kSocket2_StepPinNumber,
               Motate::kSocket2_DirPinNumber,
               Motate::kSocket2_EnablePinNumber,
               Motate::kSocket2_Microstep_0PinNumber,
               Motate::kSocket2_Microstep_1PinNumber,
               Motate::kSocket2_Microstep_2PinNumber,
               Motate::kSocket2_VrefPinNumber>
    motor_2{};

StepDirStepper<Motate::kSocket3_StepPinNumber,
               Motate::kSocket3_DirPinNumber,
               Motate::kSocket3_EnablePinNumber,
               Motate::kSocket3_Microstep_0PinNumber,
               Motate::kSocket3_Microstep_1PinNumber,
               Motate::kSocket3_Microstep_2PinNumber,
               Motate::kSocket3_VrefPinNumber>
    motor_3{};

StepDirStepper<Motate::kSocket4_StepPinNumber,
               Motate::kSocket4_DirPinNumber,
               Motate::kSocket4_EnablePinNumber,
               Motate::kSocket4_Microstep_0PinNumber,
               Motate::kSocket4_Microstep_1PinNumber,
               Motate::kSocket4_Microstep_2PinNumber,
               Motate::kSocket4_VrefPinNumber>
    motor_4{};

 StepDirStepper<
			Motate::kSocket5_StepPinNumber,
			Motate::kSocket5_DirPinNumber,
			Motate::kSocket5_EnablePinNumber,
			Motate::kSocket5_Microstep_0PinNumber,
			Motate::kSocket5_Microstep_1PinNumber,
			Motate::kSocket5_Microstep_2PinNumber,
			Motate::kSocket5_VrefPinNumber>
 	motor_5 {};

// StepDirStepper<
//    Motate::kSocket6_StepPinNumber,
//    Motate::kSocket6_DirPinNumber,
//    Motate::kSocket6_EnablePinNumber,
//    Motate::kSocket6_Microstep_0PinNumber,
//    Motate::kSocket6_Microstep_1PinNumber,
//    Motate::kSocket6_Microstep_2PinNumber,
//    Motate::kSocket6_VrefPinNumber> motor_6 {};

Stepper* Motors[MOTORS] = {&motor_1, &motor_2, &motor_3, &motor_4};

void board_stepper_init() {

#define BUFFER_SIZE 3
	// Init TMC2660
	uint8_t tx_data1[BUFFER_SIZE] = {0x09, 0x01, 0xB4};  // Hysteresis mode
//	uint8_t tx_data1[BUFFER_SIZE] = {0x09, 0x45, 0x57};  // Constant toff mode
	uint8_t tx_data2[BUFFER_SIZE] = {0x0D, 0x00, 0x0C};  // Current setting: $d001F (max. current)
	uint8_t tx_data3[BUFFER_SIZE] = {0x0E, 0x00, 0x10};		// low driver strength, stallGuard2 read, SDOFF=0
	uint8_t tx_data4[BUFFER_SIZE] = {0x00, 0x00, 0x05};		// ? microstep setting
	uint8_t tx_data5[BUFFER_SIZE] = {0x0A, 0x82, 0x02};		// Enable coolStep with minimum current ¼ CS
	uint8_t rx_data[BUFFER_SIZE];


	MX_SPI1_Init();


	// TMC1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	int ret1 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data1, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	int ret2 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data2, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	int ret3 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data3, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	int ret4 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data4, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

	// TMC2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	ret1 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data1, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	ret2 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data2, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	ret3 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data3, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	ret4 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data4, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	// TMC3
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	ret1 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data1, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	ret2 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data2, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	ret3 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data3, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	ret4 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data4, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

	// TMC4
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	ret1 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data1, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	ret2 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data2, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	ret3 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data3, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	ret4 = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data4, (uint8_t *)rx_data, BUFFER_SIZE, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    for (uint8_t motor = 0; motor < MOTORS; motor++) { Motors[motor]->init(); }
}
