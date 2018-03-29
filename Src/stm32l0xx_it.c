/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"

/* USER CODE BEGIN 0 */
#include "main.h"

#define I2C_DEVICE_FLAG_DIR_SEND                 ((uint32_t)0x80000000)
#define I2C_DEVICE_FLAG_READ                     ((uint32_t)0x40000000)
#define I2C_DEVICE_FLAG_NACK                     ((uint32_t)0x20000000)
#define I2C_DEVICE_FLAG_WRITE                    ((uint32_t)0x10000000)
#define I2C_DEVICE_FLAG_ERROR                    ((uint32_t)0x08000000)
__packed typedef struct
{
	uint32_t I2C_DeviceFlag;  //操作过程中各种标识集合
	
	uint32_t I2C_EventStatus;
	uint32_t I2C_ErrorStatus;
} I2C_DEVICE_TypeDef;
I2C_DEVICE_TypeDef i2c_dev;

#define I2C_DEVICE_DR_NULL_R                     0xae
#define I2C_DEVICE_DR_NULL_S                     0xFB

#define I2C_STATUS_EVT_MASK                      (uint32_t)(0x000000FE)
#define I2C_STATUS_ERR_MASK                      (uint32_t)(0x00000700)

#define I2C_NOSTRETCH_EN                         1  //是否打开时钟延展
//0: Disable
//1: Enable

#define I2C_REG_TEST_ADD                         (uint8_t)0xF0

//
//I2C接收中断用到变量
//
uint8_t I2CFirstByteFlag = 0;  //接收第一个字节数据标志
uint8_t I2CAddressOffset = 0;  //接收主机地址
uint8_t i2c_regtemp = 0;
uint8_t I2C_DEVICE_REGADDRESS = 0;
uint8_t I2C_DEVICE_REGOFFSET = 0;

void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_CLEAR_IT(I2C_IT));

  /* Clear the selected flag */
  I2Cx->ICR = I2C_IT;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
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
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
*/
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */
//  if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
//    HAL_I2C_ER_IRQHandler(&hi2c1);
//  } else {
//    HAL_I2C_EV_IRQHandler(&hi2c1);
//  }
  /* USER CODE BEGIN I2C1_IRQn 1 */

	uint32_t temp = 0;
	
	/*!< Read Event Flag */	
	temp = I2C1->ISR;
	i2c_dev.I2C_EventStatus = (uint32_t)(temp & I2C_STATUS_EVT_MASK);
	i2c_dev.I2C_ErrorStatus = (uint32_t)(temp & I2C_STATUS_ERR_MASK);
	
	/* ==================================================== */
	/*                    地址适配中断                       */
	/* ==================================================== */
	if ((i2c_dev.I2C_EventStatus & I2C_ISR_ADDR) != 0)
	{
		/*!< ************************************ */
		/*!<          Enable Nostretch            */
		/*!< ************************************ */
		#if I2C_NOSTRETCH_EN
		{
			if ((I2C1->ISR & I2C_ISR_DIR) != 0)  /*!< Slave enter transmitter mode, Host read data */
			{
				I2C1->ISR |= I2C_ISR_TXE;  /*!< set TXE bit */
				i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_DIR_SEND;
			}
			else  /*!< Host write data (slave receive data from host) */
			{
				i2c_dev.I2C_DeviceFlag &= ~I2C_DEVICE_FLAG_DIR_SEND;
			}
		}
		/*!< ************************************ */
		/*!<          Disable Nostretch           */
		/*!< ************************************ */
		#else
		{
			if ((I2C1->ISR & I2C_ISR_DIR) != 0)  /*!< Slave enter transmitter mode, Host read data */
			{
				i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_DIR_SEND;
				I2C1->ISR |= I2C_ISR_TXE;  /*!< set TXE bit, I2C_TXDR empty */
				//
				//根据不同寄存器地址来填充data1
				//
				if (I2C_DEVICE_REGADDRESS == I2C_REG_TEST_ADD)
				{
					if (I2C_DEVICE_REGOFFSET < I2C_REG_TEST_SIZE)
					{
						I2C1->TXDR = I2C_REG_TEST[I2C_DEVICE_REGOFFSET];
						I2C_DEVICE_REGOFFSET++;
					}
					else
					{
						I2C1->TXDR = I2C_DEVICE_DR_NULL_S;
					}
				}
				else
				{
					I2C1->TXDR = I2C_DEVICE_DR_NULL_R;
				}
			}
			else  //Receiver mode
			{
				i2c_dev.I2C_DeviceFlag &= ~I2C_DEVICE_FLAG_DIR_SEND;
			}
		}
		#endif
		I2C_ClearITPendingBit(I2C1, I2C_FLAG_ADDR);  /*!< set ADDRCF */
	}
	
	/* ==================================================== */
	/*                   从机发送数据中断                    */
	/* ==================================================== */
	if ((i2c_dev.I2C_EventStatus & I2C_ISR_TXIS) != 0)
	{
		//
		//根据不同的寄存器地址来发送数据，主机读取数据
		//
		
		//
		//测试寄存器
		//
		if (I2C_DEVICE_REGADDRESS == I2C_REG_TEST_ADD)  //read and write
		{
			if (I2C_DEVICE_REGOFFSET < I2C_REG_TEST_SIZE)
			{
				I2C1->TXDR = I2C_REG_TEST[I2C_DEVICE_REGOFFSET];
				I2C_DEVICE_REGOFFSET++;
				if (I2C_DEVICE_REGOFFSET >= I2C_REG_TEST_SIZE)
				{
					I2C_DEVICE_REGOFFSET = 0;
				}
			}
			else
			{
				I2C1->TXDR = I2C_DEVICE_DR_NULL_S;
			}
		}
		else
		{
			I2C1->TXDR = I2C_DEVICE_DR_NULL_R;
		}
	}
	
	/* ==================================================== */
	/*                   从机接收数据中断                    */
	/* ==================================================== */
	if ((i2c_dev.I2C_EventStatus & I2C_ISR_RXNE) != 0)
	{
		if (I2CFirstByteFlag != 0)
		{
			//
			//根据不同的寄存器偏移量进行接收主机数据，主机写入数据
			//
			
			//
			//测试寄存器
			//
			if (I2C_DEVICE_REGADDRESS == I2C_REG_TEST_ADD)  //read and write
			{
				if (I2C_DEVICE_REGOFFSET < I2C_REG_TEST_SIZE)
				{
					I2C_REG_TEST[I2C_DEVICE_REGOFFSET] = I2C1->RXDR;
					I2C_DEVICE_REGOFFSET++;
					if (I2C_DEVICE_REGOFFSET >= I2C_REG_TEST_SIZE)
					{
						I2C_DEVICE_REGOFFSET = 0;
					}
				}
				else
				{
					i2c_regtemp = I2C1->RXDR;
				}
			}
			else
			{
				i2c_regtemp = I2C1->RXDR;
				i2c_regtemp = 0;
			}
		}
		else
		{
			//
			//接收第一个地址字节
			//
			I2CAddressOffset = I2C1->RXDR;
			I2C_DEVICE_REGADDRESS = I2CAddressOffset & 0xF0;
			I2C_DEVICE_REGOFFSET  = I2CAddressOffset & 0x0F;
			I2CFirstByteFlag++;
		}
	}
	
	/* ==================================================== */
	/*                      非应答中断                       */
	/* ==================================================== */
	if ((i2c_dev.I2C_EventStatus & I2C_ISR_NACKF) != 0)
	{
		I2C_ClearITPendingBit(I2C1, I2C_ISR_NACKF);
		if (I2CFirstByteFlag != 0)
		{
			I2CFirstByteFlag = 0;
		}
		i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_NACK;
	}
	
	/* ==================================================== */
	/*                     停止信号中断                      */
	/* ==================================================== */
	if ((i2c_dev.I2C_EventStatus & I2C_ISR_STOPF) != 0)
	{
		if ((i2c_dev.I2C_DeviceFlag & I2C_DEVICE_FLAG_DIR_SEND) != 0)  /*!< Slave send to host */
		{
			/*!< ************************************ */
			/*!<          Enable Nostretch            */
			/*!< ************************************ */
			#if I2C_NOSTRETCH_EN
			
			/*!< ************************************ */
			/*!<          Disable Nostretch           */
			/*!< ************************************ */
			#else
			I2C1->ISR |= I2C_ISR_TXE;
			I2C1->ISR |= I2C_ISR_TXIS;
			#endif
			i2c_dev.I2C_DeviceFlag &= ~I2C_DEVICE_FLAG_DIR_SEND;
			i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_READ;
		}
		else
		{
			i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_WRITE;  //置操作标志
		}
		if (I2CFirstByteFlag != 0)
		{
			I2CFirstByteFlag = 0;
		}
		I2C_ClearITPendingBit(I2C1, I2C_ISR_STOPF);  /*!< set STOPCF */
	}
	
	
	/* ==================================================== */
	/*                       错误中断                        */
	/* ==================================================== */
	//
	//总线错误
	//
	if ((i2c_dev.I2C_ErrorStatus & I2C_ISR_BERR) != 0)
	{
		I2C1->ICR = I2C_ICR_BERRCF;
		i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_ERROR;
	}
	//
	//总裁失败
	//
	if ((i2c_dev.I2C_ErrorStatus & I2C_ISR_ARLO) != 0)
	{
		I2C1->ICR = I2C_ICR_ARLOCF;
		i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_ERROR;
	}
	//
	//溢出
	//
	if ((i2c_dev.I2C_ErrorStatus & I2C_ISR_OVR) != 0)
	{
		I2C1->ICR = I2C_ICR_OVRCF;
		i2c_dev.I2C_DeviceFlag |= I2C_DEVICE_FLAG_ERROR;
	}

	
	
  /* USER CODE END I2C1_IRQn 1 */
}

/**
* @brief This function handles I2C2 interrupt.
*/
void I2C2_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_IRQn 0 */

  /* USER CODE END I2C2_IRQn 0 */
  if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c2);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c2);
  }
  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
