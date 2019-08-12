/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "DS18B20.h"

uint8_t Radio_OK_Received = 0;
uint8_t Radio_NOK_received = 0;
uint8_t AT_NOK_Received = 1;
#define ADC_DELAY_ENABLE_CALIB_CPU_CYCLES  (LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32)
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)   4)
__IO uint16_t aADCxConvertedData[4]; /* ADC group regular conversion data */
__IO uint16_t MQ2A_Data, MQ2B_Data, FRAMECLICK_Data, DS18B20_Data  = 0;
__IO uint8_t MQ2A_Data_Ok, MQ2B_Data_Ok, FRAMECLICK_Data_Ok, DS18B20_Data_Ok = 0;
__IO uint8_t ASCII_CODE[16] = {"0123456789ABCDEF"};
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc (int ch, FILE *stream)
{
	while(!LL_USART_IsActiveFlag_TXE(USART1));
	LL_USART_TransmitData8(USART1,ch);
	return ch ; 
}
void delay(uint32_t ms)
{
	uint32_t i,y;
	for(i=0; i<ms; i++)
	{
		for(y = 0; y<7200; y++)
		{
			__NOP();
		}
	}
}
    /* USER CODE END WHILE */
		//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_1)	;
uint8_t AtCmd_AT[4] = "AT\r\n";
uint8_t AtCmd_Poll[1] = "$";
uint32_t AT_timeout = 3;
uint8_t AT_cnt = 0;
void AT_poll()
{
	Radio_OK_Received = 0;
	while(!LL_USART_IsActiveFlag_TXE(USART3));
	LL_USART_TransmitData8(USART3,'$');
}

uint8_t AT_send(uint8_t *cmd)
{
	uint8_t Cmd_Ok = 0;
	AT_cnt = AT_timeout;
	AT_poll();
	while(!Radio_OK_Received)
	{
		delay(100);
		if(AT_cnt ==0 )
		{
			return 0;
		}else if (Radio_NOK_received)
		{
			Radio_NOK_received = 0;
			return 0 ;
		}
		AT_cnt--;

	}	
	Radio_OK_Received = 0;
	uint8_t i = 0;

	for( i = 0; i<6; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART3));
		LL_USART_TransmitData8(USART3,cmd[i]);
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1,cmd[i]);
	}
	
	return 1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
float temprature = 0;
__IO uint16_t _temprature = 0;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  
	/* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */


  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();	
	MX_ADC1_Init();	
  MX_DMA_Init();
  uint8_t a = DS18B20_Init();

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
	printf("\r\n\fHello World123\r\n");
	__IO uint8_t Msg_Frame[6]="LLLLLL";
	__IO uint8_t Msg_MQ2A[6] = "MMMMMM";
	__IO uint8_t Msg_MQ2B[6] = "NNNNNN";
	__IO uint8_t Msg_DS18B20[6] ="TTTTTT";
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	printf("hello world\r\n");
  while (1)
  {
		
		__disable_irq();
		
		temprature = DS18B20_GetTemp_SkipRom();
		_temprature = temprature*10;		
		//printf(" %d\r\n", _temprature);
		Msg_DS18B20[4] = ASCII_CODE[(_temprature)&0x0f];
		Msg_DS18B20[3] = ASCII_CODE[ _temprature>>4& 0x0f ];
		Msg_DS18B20[2] = ASCII_CODE[(_temprature>>8)&0x0f];
		Msg_DS18B20[1] = ASCII_CODE[_temprature>>12&0x0f];
		__enable_irq();
	
		AT_send(Msg_DS18B20);
		uint8_t i = 0;

		if(MQ2A_Data_Ok)
		{
			MQ2A_Data_Ok = 0;
			Msg_MQ2A[4] = ASCII_CODE[MQ2A_Data&0x0f];
			Msg_MQ2A[3] = ASCII_CODE[MQ2A_Data>>4& 0x0f];
			Msg_MQ2A[2] = ASCII_CODE[MQ2A_Data>>8&0x0f];
			Msg_MQ2A[1] = ASCII_CODE[MQ2A_Data>>12&0x0f];
			AT_send(Msg_MQ2A);
		}
		if(MQ2B_Data_Ok)
		{
			MQ2B_Data_Ok = 0;			
			Msg_MQ2B[4] = ASCII_CODE[MQ2B_Data&0x0f];
			Msg_MQ2B[3] = ASCII_CODE[MQ2B_Data>>4& 0x0f];
			Msg_MQ2B[2] = ASCII_CODE[MQ2B_Data>>8&0x0f];
			Msg_MQ2B[1] = ASCII_CODE[MQ2B_Data>>12&0x0f];
			AT_send(Msg_MQ2B);
		}
		
		if(FRAMECLICK_Data_Ok)
		{
			FRAMECLICK_Data_Ok = 0;
			Msg_Frame[4] = ASCII_CODE[FRAMECLICK_Data&0x0f];
			Msg_Frame[3] = ASCII_CODE[FRAMECLICK_Data>>4& 0x0f];
			Msg_Frame[2] = ASCII_CODE[FRAMECLICK_Data>>8&0x0f];
			Msg_Frame[1] = ASCII_CODE[FRAMECLICK_Data>>12&0x0f];
			AT_send(Msg_Frame);
		}	
		
		//delay(100);
		
		/*
		__disable_irq();
		temprature = DS18B20_GetTemp_SkipRom();
		printf("%f\r\n", temprature);
		__enable_irq();
		delay(1000);
		*/
		
    /* USER CODE END WHILE */
		/*
		AT_send(AtCmd_AT);
		delay(1000);
		
		if(Radio_OK_Received)
		{
			Radio_OK_Received = 0;
			//printf("Received AT\r\n");
		}*/
		
		
    /* USER CODE BEGIN 3 */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(72000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  __IO uint32_t wait_loop_index = 0;

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  /**ADC1 GPIO Configuration  
  PC0   ------> ADC1_IN10
  PC1   ------> ADC1_IN11
  PC2   ------> ADC1_IN12
  PC3   ------> ADC1_IN13 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ADC1 DMA Init */
  
  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);


	
  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Common config 
  */
	
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;

  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	
  /**Configure Regular Channel 
  */
	
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_10);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_10, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_11);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_12);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_13);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_13, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC1_Init 2 */
	LL_ADC_Enable(ADC1);
	/*
	wait_loop_index = (ADC_DELAY_ENABLE_CALIB_CPU_CYCLES >> 1);
	while(wait_loop_index != 0)
	{
		wait_loop_index--;
	}
	*/
	//LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_INDEPENDENT);
	//LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);
  //LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);	
	//LL_ADC_REG_SetSequencerDiscont(ADC1,LL_ADC_REG_SEQ_DISCONT_4RANKS);
	LL_ADC_REG_StartConversionSWStart(ADC1);	
	//LL_ADC_REG_StartConversionExtTrig(ADC1,LL_ADC_REG_TRIG_EXT_RISING);
	LL_ADC_StartCalibration(ADC1);
		//LL_ADC_REG_StartConversionExtTrig	(ADC1,LL_ADC_REG_TRIG_EXT_RISING);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 50000;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 10000-1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 7200-1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV4;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */
	LL_TIM_GenerateEvent_TRIG	(TIM3);
  LL_TIM_EnableCounter(TIM3);	
	LL_TIM_EnableIT_UPDATE(TIM3);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
	LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration  
  PC10   ------> USART3_TX
  PC11   ------> USART3_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  LL_GPIO_AF_RemapPartial_USART3();

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */
	LL_USART_EnableIT_RXNE(USART3);
  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	 LL_DMA_ConfigTransfer(DMA1,
                        LL_DMA_CHANNEL_1,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_MODE_CIRCULAR              |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_HALFWORD        |
                        LL_DMA_MDATAALIGN_HALFWORD        |
                        LL_DMA_PRIORITY_HIGH               );
	LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&aADCxConvertedData,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1,
                         LL_DMA_CHANNEL_1,
                       4);
	LL_DMA_EnableIT_TC(DMA1,
                        LL_DMA_CHANNEL_1);		
	LL_DMA_EnableChannel(DMA1,
                       LL_DMA_CHANNEL_1);								
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE12);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE13);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE14);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_14;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_FLOATING);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_FLOATING);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_FLOATING);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
