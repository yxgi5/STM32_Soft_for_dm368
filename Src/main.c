/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32l0xx_hal.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define CMD_TEMPERATURE      0x08
#define CMD_VOLTAGE          0x09
#define CMD_AVERAGE_CURRENT  0x0B
#define CMD_RSOC             0x0D    //RelativeStateOfCharge
#define REMAINING_CAPACITY   0x0F
#define FULL_CHARGE_CAPACITY 0x10
#define STATE_OF_HEALTH      0x4F


uint8_t aTxStartMessage[] = "****Please send the number blow to get related value****\r\n1:  电池温度\r\n2:  电池电压\r\n\
3:  平均电流\r\n4:  电量百分比\r\n5:  剩余容量\r\n6:  总容量\r\n7： 电池寿命\r\n";
uint8_t aRxBufferUART;

#define ADDR_bq40z50r1_Write 0x16
#define ADDR_bq40z50r1_Read 0x17

uint8_t aRxSMBusBuffer[2] = {0, 0};
uint8_t aTxSMBusBuffer[1] = {0x09};

uint8_t flag_Rx_OK;

uint8_t I2C_REG_TEST[I2C_REG_TEST_SIZE];

uint8_t flag_10ms = 0;
uint8_t flag_1s   = 0;

typedef struct 
{
	uint8_t tmp_cur;
	uint8_t count;
	uint8_t long_press;
	uint8_t short_press;
}class_key;
class_key  key;

static void key_process(void);
void lcd_init(void);
static void power_dm36m_on(void)
{
	uint32_t i;
	
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	  //BAT_CH1
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	  //BAT_CH2
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   	//1.35V for core  
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);     //1.8V for core		
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	    //3.3V for core	
	for(i=0;i<10;i++);
	lcd_init();
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   // DM368 Reset
	for(i=0;i<10000;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//	lcd_init();
}
static void power_dm36m_off(void)
{
	uint32_t i; 
	
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);	  //BAT_CH1
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	  //BAT_CH2
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);   	//1.35V for core  
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);     //1.8V for core		
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	    //3.3V for core	
	for(i=0;i<10;i++);
}
static void reset_bq40z50(void)
{
	uint32_t i; 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);   // bq40z50 wakeup
	for(i=0;i<10000;i++);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	for(i=0;i<10000;i++);
	//for(i=0;i<10000;i++);for(i=0;i<10000;i++);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//the follow code is copy from older version     //zbb 20180111

/*   	MCU: STM32L051C6T6
			STM32.PA0->SDA
			STM32.PA1->SCL
			STM32.PA5->SCLK
			STM32.PA7->MOSI			
			I2C init the OV426
			SPI init the LCD(TL035VG03)
			STM32 GPIO control the dm368 power supply.
			STM32.PB4  Control  BAT_CH1_CTL     0->OPEN  1->OFF
			STM32.PB5  Control  BAT_CH2_CTL			0->OPEN  1->OFF
			STM32.PB1  Control  DV_1V3_EN				1->OPEN  0->OFF
			STM32.PB2  Control  DV_1V8_EN				1->OPEN  0->OFF
			STM32.PB3  Control  DV_3V3_EN				1->OPEN  0->OFF
			STM32.PB0  Control  POWER_RESET_CTL 0->RESET  //zbb 20180111
*/

void delay_ns(unsigned int delay)
{
	uint32_t i;
  for(i=0;i<(4*delay);i++);		
}

void delay_ms(unsigned int delay)
{
	uint32_t i;
  for(i=0;i<(10000*delay);i++);		
}

void gpio4_set_high(void)	
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);			
}

void gpio4_set_low(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);	
}

void gpio5_set_low(void)
{	
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);	
}

void gpio5_set_high(void)	
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);				
}

void gpio7_set_high(void)	
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);			
}

void gpio7_set_low(void)
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);	
}

//PA5->  SCLK
//PA7->  MOSI

void spi_write_8bit(unsigned char data)
{
	unsigned char temp;
	
	//delay_ns(1);	
	gpio5_set_low();
	delay_ns(1);	
	temp=data;
//######send the bit 7
	if((temp&0x80)==0x80)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 6
	temp=data;
	if((temp&0x40)==0x40)
	
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);

	//######send the bit 5
	temp=data;
	if((temp&0x20)==0x20)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	
	//######send the bit 4
	temp=data;
	if((temp&0x10)==0x10)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 3
	temp=data;
	if((temp&0x08)==0x08)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 2
	temp=data;
	if((temp&0x04)==0x04)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 1
	temp=data;
	if((temp&0x02)==0x02)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 0
	temp=data;
	if((temp&0x01)==0x01)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);	
	
}



//	PA5->  SCLK
//	PA7->  MOSI
// 	GPIO Emulator the SPI  9bit 
// 	bit9=0->> command   ,   bit9=1->> data
//  Periode is nearly 150uS

void spi_write_9bit(unsigned char data,unsigned char command)
{
	unsigned char temp;
	gpio5_set_low();
	delay_ns(1);	
	temp=data;;	
	
	//######send the bit 8
	if((command&0x01)==1)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);

	//######send the bit 7
	if((temp&0x80)==0x80)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 6
	temp=data;
	if((temp&0x40)==0x40)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);

	//######send the bit 5
	temp=data;
	if((temp&0x20)==0x20)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	
	//######send the bit 4
	temp=data;
	if((temp&0x10)==0x10)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 3
	temp=data;
	if((temp&0x08)==0x08)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 2
	temp=data;
	if((temp&0x04)==0x04)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 1
	temp=data;
	if((temp&0x02)==0x02)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);
	
	//######send the bit 0
	temp=data;
	if((temp&0x01)==0x01)
	{	
	gpio7_set_high();	
	}
	else
	{
	gpio7_set_low();	
	}	
	delay_ns(2);
	gpio5_set_high();
	delay_ns(2);
	gpio5_set_low();
	delay_ns(2);	
	
}

// LCD Init
void lcd_init(void)
{
	
	gpio4_set_low();
	delay_ns(40);
	
	//Sleep in
	spi_write_9bit(0x10,0);
	delay_ms(250);
	delay_ms(250);
	
	//set usr command		
	spi_write_9bit(0xB9,0);
	spi_write_9bit(0xff,1);
	spi_write_9bit(0x83,1);
	spi_write_9bit(0x63,1);
	
	delay_ns(2);
		
	//set power		  
	spi_write_9bit(0xB1,0);
	spi_write_9bit(0x81,1);
	spi_write_9bit(0x34,1);
	spi_write_9bit(0x07,1);
	spi_write_9bit(0x31,1);
	spi_write_9bit(0x02,1);
	spi_write_9bit(0x13,1);
	spi_write_9bit(0x11,1);
	spi_write_9bit(0x11,1);
	spi_write_9bit(0x39,1);
	spi_write_9bit(0x41,1);
	spi_write_9bit(0x3f,1);
	spi_write_9bit(0x3f,1);
	
	
	//Set COLMOD
	spi_write_9bit(0x3a,0);	
	spi_write_9bit(0x60,1);
	
	//Set_RGBIF 
	spi_write_9bit(0xb3,0);
	spi_write_9bit(0x09,1);	

	//Set CYC
	spi_write_9bit(0xB4,0);	
	spi_write_9bit(0x08,1);
	spi_write_9bit(0x03,1);
	spi_write_9bit(0x7e,1);
	spi_write_9bit(0x02,1);
	spi_write_9bit(0x01,1);
	spi_write_9bit(0x12,1);	
	spi_write_9bit(0x64,1);
	spi_write_9bit(0x01,1);
	spi_write_9bit(0x60,1);
	//delay_ns(200);
	
	
	//set vcom voltage
	spi_write_9bit(0xb6,0);
	spi_write_9bit(0x1b,1);
	
	
	
	spi_write_9bit(0xcc,0);
	spi_write_9bit(0x0b,1);
	delay_ns(20);
	
	//set gamma 
	spi_write_9bit(0xe0,0);
	spi_write_9bit(0x00,1);
	spi_write_9bit(0x02,1);
	spi_write_9bit(0x48,1);
	spi_write_9bit(0x0f,1);
	spi_write_9bit(0x0f,1);
	spi_write_9bit(0x3f,1);
	spi_write_9bit(0x08,1);
	spi_write_9bit(0x90,1);
	spi_write_9bit(0x50,1);
	spi_write_9bit(0x15,1);
	spi_write_9bit(0x17,1);
	spi_write_9bit(0xd5,1);
	spi_write_9bit(0x56,1);
	spi_write_9bit(0x14,1);	
	spi_write_9bit(0x19,1);	
	spi_write_9bit(0x00,1);	
	spi_write_9bit(0x02,1);
	spi_write_9bit(0x48,1);	
	spi_write_9bit(0x0f,1);
	
	
	spi_write_9bit(0x0f,1);
	spi_write_9bit(0x3f,1);
	spi_write_9bit(0x08,1);
	spi_write_9bit(0x90,1);
	spi_write_9bit(0x50,1);
	spi_write_9bit(0x15,1);
	spi_write_9bit(0x17,1);
	spi_write_9bit(0xd5,1);
	spi_write_9bit(0x56,1);
	spi_write_9bit(0x14,1);	
	spi_write_9bit(0x19,1);	
	
	
	delay_ns(20);
	delay_ns(20);
	
	
	spi_write_9bit(0x36,0);
	//delay_ns(2);
	spi_write_9bit(0x00,1);
	delay_ns(20);
	
	//Display on
	spi_write_9bit(0x11,0);
	delay_ms(250);
	delay_ms(250);
	delay_ms(250);
	
	spi_write_9bit(0x21,0);
	spi_write_9bit(0x29,0);
	
	delay_ns(40);
	
	gpio4_set_high();	

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t tmp = 0;
	uint32_t i2c_temp = 0;
	uint8_t stm32_onoff_status = 0;
		uint8_t i = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
//  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	memset(&key, 0, sizeof(class_key));
	
	memset((char *)I2C_REG_TEST, 0x00, I2C_REG_TEST_SIZE);
	I2C1->CR2 &= ~I2C_CR2_NACK;  /*!< ACK enable */
	
		/*!< 时钟延展功能 */
	#if I2C_NOSTRETCH_EN
	I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;  /*!< Enable Nostretch */
	#else
	I2C1->CR1 |= I2C_CR1_NOSTRETCH;  /*!< Disable Nostretch */
	#endif
	
	i2c_temp = 0;
	i2c_temp = (I2C_CR1_ERRIE/*!< Error interrupts enable */  |\
	            I2C_CR1_STOPIE/*!< STOP detection Interrupt enable */ |\
	            I2C_CR1_NACKIE/*!< Not acknowledge received Interrupt enable */ |\
	            I2C_CR1_ADDRIE/*!< Address match Interrupt enable (slave only) */ |\
	            I2C_CR1_RXIE/*!< RX Interrupt enable */ |\
	            I2C_CR1_TXIE/*!< TX Interrupt enable */);
	I2C1->CR1 |= i2c_temp;
	
	
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage)-1);
	while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);//两个灯
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
	
	power_dm36m_on();//上电
	delay_ms(1000);
	power_dm36m_off();//断电
	reset_bq40z50();
	delay_ms(1000);
		

	
	//电流
	aTxSMBusBuffer[0] = CMD_AVERAGE_CURRENT;	
	HAL_I2C_Master_Transmit_IT(&hi2c2, ADDR_bq40z50r1_Write, &aTxSMBusBuffer[0], 1);
	while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
	HAL_I2C_Master_Receive_IT(&hi2c2, ADDR_bq40z50r1_Write, &aRxSMBusBuffer[0], 2);
	while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
	tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
	tmp |= aRxSMBusBuffer[0];
	if((tmp&0x8000) != 0)  //放电
		I2C_REG_TEST[2] = 0x01;
	else
		I2C_REG_TEST[2] = 0x02;
	
	//电流要么正要么负 为零暂时认为bq40z50故障
	if((aRxSMBusBuffer[0]==0)&&(aRxSMBusBuffer[1]==0))
	{
		I2C_REG_TEST[0] = 0x00; //ERR
	}
	else
	{
		I2C_REG_TEST[0] = 0x01; //OK
	}

	aRxSMBusBuffer[0] = 0;
	aRxSMBusBuffer[1] = 0;
	
	//电量
	aTxSMBusBuffer[0] = CMD_RSOC;
	HAL_I2C_Master_Transmit_IT(&hi2c2, ADDR_bq40z50r1_Write, &aTxSMBusBuffer[0], 1);
	while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
	HAL_I2C_Master_Receive_IT(&hi2c2, ADDR_bq40z50r1_Write, &aRxSMBusBuffer[0], 2);
	while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
	tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
	tmp |= aRxSMBusBuffer[0];
	I2C_REG_TEST[1] = (uint8_t)tmp;

	aRxSMBusBuffer[0] = 0;
	aRxSMBusBuffer[1] = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	HAL_TIM_Base_Start_IT(&htim2);
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		key_process();
		
		if(flag_1s)
		{
			flag_1s = 0;
			
			//电流
			aTxSMBusBuffer[0] = CMD_AVERAGE_CURRENT;	
			HAL_I2C_Master_Transmit_IT(&hi2c2, ADDR_bq40z50r1_Write, &aTxSMBusBuffer[0], 1);
			while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
			HAL_I2C_Master_Receive_IT(&hi2c2, ADDR_bq40z50r1_Write, &aRxSMBusBuffer[0], 2);
			while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
			tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
			tmp |= aRxSMBusBuffer[0];
			if((tmp&0x8000) != 0)  //放电
				I2C_REG_TEST[2] = 0x01;
			else
				I2C_REG_TEST[2] = 0x02;

//			//测试充电放电转换时间
//			printf("\n\r aRxSMBusBuffer[0]%d \n\r",aRxSMBusBuffer[0]);
//			printf("\n\r aRxSMBusBuffer[1]%d \n\r",aRxSMBusBuffer[1]);
			
			aRxSMBusBuffer[0] = 0;
			aRxSMBusBuffer[1] = 0;
			
			//电量
			aTxSMBusBuffer[0] = CMD_RSOC;
			HAL_I2C_Master_Transmit_IT(&hi2c2, ADDR_bq40z50r1_Write, &aTxSMBusBuffer[0], 1);
			while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
			HAL_I2C_Master_Receive_IT(&hi2c2, ADDR_bq40z50r1_Write, &aRxSMBusBuffer[0], 2);
			while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
			tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
			tmp |= aRxSMBusBuffer[0];
			I2C_REG_TEST[1] = (uint8_t)tmp;
			aRxSMBusBuffer[0] = 0;
			aRxSMBusBuffer[1] = 0;
//			printf("\n\r I2C_REG_TEST[0]%d \n\r",I2C_REG_TEST[0]);
//			printf("\n\r I2C_REG_TEST[1]%d \n\r",I2C_REG_TEST[1]);
		}
		
		
		//关机状态且在充电则直接上电
		if((stm32_onoff_status == 0x00)&&(I2C_REG_TEST[2] == 0x02))   
		{
			power_dm36m_on();//上电
			stm32_onoff_status = 0x01;
		}	
	
		if(key.short_press)
		{
	
			I2C_REG_TEST[3]  = 0x01;  //开机请求
			if(stm32_onoff_status == 0x00)   //关机状态
			{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);  //点亮指示灯

					power_dm36m_on();//上电
					stm32_onoff_status = 0x01;
					I2C_REG_TEST[3]  = 0x00;  //开机请求响应完毕
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  //开完机关掉指示灯
			}	
			key.short_press = 0;
		}	
	
		if(key.long_press)
		{
			key.long_press = 0;		
			I2C_REG_TEST[3]  = 0x02;  //关机请求
		}
		if(stm32_onoff_status == 0x01)//开机状态
			{
			  //关机请求下应用程序阶段可以断电或者Uboot阶段可以断电那么就断电
				if((I2C_REG_TEST[3] == 0x02)&&((I2C_REG_TEST[4] == 0x02)||(I2C_REG_TEST[4] == 0x03)))
				{
						power_dm36m_off();//断电
						stm32_onoff_status = 0x00;
						I2C_REG_TEST[3]  = 0x00;  //关机请求响应完毕
						I2C_REG_TEST[4]  = 0x00;
					}
			}				
		
		if(I2C_REG_TEST[5] == 0x01) //Uboot请求初始化LCD
		{
			I2C_REG_TEST[5] = 0;
			lcd_init();
//			printf("\n\r init LCD");
		}
		if(I2C_REG_TEST[4] == 0x03) //Uboot阶段时可以断电
		{	printf("\n\r I2C_REG_TEST[4] is %d \n\r",I2C_REG_TEST[4]);
			I2C_REG_TEST[4] = 0;
			power_dm36m_off();//断电
			stm32_onoff_status = 0x00;
		}
		
		HAL_UART_Receive_IT(&huart1, &aRxBufferUART, 1);
		if(flag_Rx_OK)
		{
				flag_Rx_OK = 0;
//				printf("\n\r flag_USART_Rx_OK\n\r");
				
				HAL_I2C_Master_Transmit_IT(&hi2c2, ADDR_bq40z50r1_Write, &aTxSMBusBuffer[0], 1);
				while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
				HAL_I2C_Master_Receive_IT(&hi2c2, ADDR_bq40z50r1_Write, &aRxSMBusBuffer[0], 2);
				while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY );
					switch(aRxBufferUART)
					{		
						case '1':
							tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
							tmp |= aRxSMBusBuffer[0];
						  printf("\n\r 电池温度是： %d℃ \n\r",tmp);
							break;
						case '2': 
							tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
							tmp |= aRxSMBusBuffer[0];
						  printf("\n\r 电池电压是： %dmV \n\r",tmp);
							break;
						case '3': 
							tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
							tmp |= aRxSMBusBuffer[0];
						  printf("\n\r 充电平均电流是： %dmA \n\r",tmp);
							break;
						case '4': 
							tmp  = 0;
							tmp |= aRxSMBusBuffer[0];
						  printf("\n\r 电量百分比是： %d%% \n\r",tmp);
							break;
						case '5': 
							tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
							tmp |= aRxSMBusBuffer[0];
						  printf("\n\r 剩余容量是： %dmAh \n\r",tmp);
							break;
						case '6': 
							tmp  = (uint16_t)(aRxSMBusBuffer[1] << 8);
							tmp |= aRxSMBusBuffer[0];
						  printf("\n\r 总容量是： %dmAh \n\r",tmp);
							break;
						case '7': 
							tmp  = 0;
							tmp |= aRxSMBusBuffer[0];
						  printf("\n\r 电池寿命是： %d%% \n\r",tmp);
							break;	
						default:
							break;
					}
					aRxSMBusBuffer[0] = 0;
					aRxSMBusBuffer[1] = 0;
					while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

//    /**Configure the main internal regulator output voltage 
//    */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//    /**Configure LSE Drive Capability 
//    */
//  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
//  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
//                              |RCC_PERIPHCLK_RTC;
//  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
//  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
//  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

	__PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //	HAL_GPIO_WritePin(EN485_GPIO_Port, EN485_Pin, GPIO_PIN_SET);
	switch(aRxBufferUART)
	{		
		case '1': 
			aTxSMBusBuffer[0] = CMD_TEMPERATURE;
			break;
		case '2': 
			aTxSMBusBuffer[0] = CMD_VOLTAGE; 
			break;
		case '3': 
			aTxSMBusBuffer[0] = CMD_AVERAGE_CURRENT;
			break;
		case '4': 
			aTxSMBusBuffer[0] = CMD_RSOC; 
			break;
		case '5': 
			aTxSMBusBuffer[0] = REMAINING_CAPACITY;
			break;
		case '6': 
			aTxSMBusBuffer[0] = FULL_CHARGE_CAPACITY;
			break;
		case '7': 
			aTxSMBusBuffer[0] = STATE_OF_HEALTH; 
			break;	
		default:
			break;
	}
	 flag_Rx_OK = 1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t cnt = 0;
	if (htim->Instance == htim2.Instance)
	{
//		GPIOA->ODR ^= GPIO_PIN_15;
		flag_10ms = 1;
		cnt++;
		if(cnt >= 100)
		{
			flag_1s = 1;
			cnt     = 0;
		}	
	}
}
void key_process(void)
{
	if(flag_10ms)
	{
		flag_10ms = 0;
		key.tmp_cur = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);	
		if(key.tmp_cur == GPIO_PIN_RESET)//按键被按下
		{
				if(key.count < 255)
					key.count++;
				else
					key.long_press = 1;
		}
		else//按键被松开
		{
			if((key.count > 4) && (key.count <= 200) )// 50ms -- 2s
			{    
				key.short_press = 1;
      } 
			else if(key.count >= 200) // 2s
			{ 
				key.long_press = 1;
      } 
			else //去除干扰
			{
        key.short_press = 0;
				key.long_press  = 0;
      }
      key.count = 0;
		}
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
