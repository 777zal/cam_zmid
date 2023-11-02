/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart.h"
#include "ctrl.h"
#include "zmid.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

	  if(cam_flag.timer_is_running)
	  {


		  if(cam_timer.seconds < 60-1)
		  {
			  cam_timer.seconds++;
		  }
		  else {
			  cam_timer.seconds = 0;
			  cam_timer.minutes++;
			  if(cam_timer.minutes == 10){
				  cam_timer.minutes = 0;
				  cam_flag.timer_is_running = false;
			  }
		  }
	  }

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	static uint8_t buff[50];
	uint8_t data_read;
	static uint16_t count;
	static uint16_t checksum_n;
	uint32_t isrflags   = READ_REG(huart2.Instance->SR);
	uint32_t cr1its     = READ_REG(huart2.Instance->CR1);
	if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)){
		huart2.Instance->SR;
		data_read = huart2.Instance->DR;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
	}

	switch(zmid_state)
	{
	case FRAME_IDLE :
		if(data_read == 0x55){
			count = 0;
			zmid_package.sum = (uint64_t)data_read;
			HAL_UART_Transmit(&huart2, (uint8_t*)&data_read, 1, 100);
			zmid_state = FRAME_SPECIFIER;
		}
		break;
	case FRAME_SPECIFIER :
//		HAL_UART_Transmit(&huart2, (uint8_t*)&data_read, 1, 100);
		if(data_read == 0x48){
			zmid_state = FRAME_H_CAM_ID;
		}
		else if(data_read == 0x52){
			zmid_state = FRAME_R_CAM_ID;
		}
		else if(data_read == 0x45){
			zmid_state = FRAME_E_CAM_ID;
		}
		else if(data_read == 0x46){
			zmid_package.sum += data_read;
			zmid_state = FRAME_F_CAM_ID;
		}
		else zmid_state = FRAME_RANDOM_CAM_ID;
		break;
	case FRAME_RANDOM_CAM_ID :
//		HAL_UART_Transmit(&huart2, (uint8_t*)&data_read, 1, 100);
		zmid_state = FRAME_IDLE;
		break;

		/* H ACK */
	case FRAME_H_CAM_ID :
		zmid_param.camera_id = data_read;
		zmid_state = FRAME_H_END_MARK;
		break;
	case FRAME_H_END_MARK :
		if(data_read == 0x23){
//			HAL_UART_Transmit(&huart2, (uint8_t*)"A", 1, 1);
		}
		else {
//			HAL_UART_Transmit(&huart2, (uint8_t*)"X", 1, 1);
		}
		zmid_state = FRAME_IDLE;
		break;

		/* R command */
	case FRAME_R_CAM_ID :
		zmid_param.camera_id = data_read;
		zmid_state = FRAME_R_SIZE;
		count = 4-1;
		break;
	case FRAME_R_SIZE :
		buff[count] = (char)data_read;
		if(count == 0){
			zmid_param.picture_size = ((uint32_t)(buff[0]) << 24) | \
									  ((uint32_t)(buff[1]) << 16) | \
									  ((uint32_t)(buff[2]) << 8)  | \
									  ((uint32_t)(buff[3]));
			count = 2-1;
			zmid_state = FRAME_R_AMOUNT;
		}
		else count--;
		break;
	case FRAME_R_AMOUNT :
		buff[count] = (char)data_read;
		if(count == 0){
			zmid_param.package_amount = ((uint16_t)(buff[0]) << 8) | \
									  	((uint16_t)(buff[1]));
			count = 0;
			zmid_state = FRAME_R_END_MARK;
		}
		else count--;
		break;
	case FRAME_R_END_MARK :
		if(data_read == 0x23){
//			sprintf(buff, "camera no : %u\r\n", zmid_param.camera_id);
//			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1);
//			sprintf(buff, "\r\npicture size : %lu\r\n", zmid_param.picture_size);
//			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1);
//			sprintf(buff, "package amount : %u\r\n", zmid_param.package_amount);
//			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1);
		}
		else {
//			HAL_UART_Transmit(&huart2, (uint8_t*)"X", 1, 1);
		}
		zmid_state = FRAME_IDLE;
		break;

		/* E ACK */
	case FRAME_E_CAM_ID :
		zmid_param.camera_id = data_read;
		zmid_state = FRAME_E_END_MARK;
		break;
	case FRAME_E_END_MARK :
		if(data_read == 0x23){
//			HAL_UART_Transmit(&huart2, (uint8_t*)"A", 1, 1);
		}
		else {
//			HAL_UART_Transmit(&huart2, (uint8_t*)"X", 1, 1);
		}
		zmid_state = FRAME_IDLE;
		break;

		/* F command */
	case FRAME_F_CAM_ID :
		zmid_param.camera_id = data_read;
		zmid_package.sum += data_read;
		count = 2-1;
		zmid_state = FRAME_F_PKG_NO;
		break;
	case FRAME_F_PKG_NO :
		buff[count] = data_read;
		zmid_package.sum += data_read;
		if(count == 0){
			zmid_package.package_no = ((uint16_t)(buff[0]) << 8) | \
									  ((uint16_t)(buff[1]));
			count = 2-1;
			zmid_state = FRAME_F_PKG_SIZE;
		}
		else count--;
		break;
	case FRAME_F_PKG_SIZE :
		buff[count] = data_read;
		zmid_package.sum += data_read;
		if(count == 0){
			zmid_package.package_size = ((uint16_t)(buff[0]) << 8) | \
									    ((uint16_t)(buff[1]));
			count = 0;
			zmid_state = FRAME_F_PKG_DATA;
		}
		else count--;
		break;
	case FRAME_F_PKG_DATA :
		zmid_package.sum += data_read;
		if(count==zmid_package.package_size-1){
			zmid_package.checksum = (uint16_t)(zmid_package.sum);
			count = 2-1;
			zmid_state = FRAME_F_CHECKSUM;
		}
		else {
			count++;
		}
		break;
	case FRAME_F_CHECKSUM :
		buff[count] = data_read;
		if(count == 0){
			checksum_n = ((uint16_t)(buff[0]) << 8) | \
									(uint16_t)(buff[1]);
//			sprintf((char*)&buff, "\r\npackage size : %u\r\n", zmid_package.package_size);
//			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1);
//			sprintf((char*)&buff, "package number : %u\r\n", zmid_package.package_no);
//			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1);
//			sprintf((char*)&buff, "checksum count : %x\r\n", zmid_package.checksum);
//			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1);
//			sprintf((char*)&buff, "package checksum : %x\r\n", checksum_n);
//			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1);
			if((checksum_n == zmid_package.checksum) && (zmid_package.package_no == zmid_param.package_amount) && (zmid_param.camera_id == 0x01))
			{
//				sprintf((char*)&buff, "\r\ntransmit is done and valid!\r\n");
//				HAL_UART_Transmit(&huart2, buff, strlen(buff), 1);
				if(cam_flag.cam1_is_running && !cam_flag.cam2_is_running){
					cam_flag.cam1_is_running = false;
				}
				else if(!cam_flag.cam1_is_running && cam_flag.cam2_is_running) {
					cam_flag.cam2_is_running = false;
				}
			}
			count = 0;
			zmid_state = FRAME_IDLE;
		}
		else {
			count--;
		}
		break;
	default :
		break;
	}

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
