/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Ping-Pong implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    27-February-2017
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stdio.h"
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "delay.h"
#include "low_power.h"
#include "vcom.h"
#include "stm32l0xx_hal.h"
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
I2C_HandleTypeDef hi2c1;

__IO uint8_t LocalACK_Ok,LocalNACK = 0;
__IO uint8_t RadioResend_Ok = 0;
__IO uint8_t RadioDS18B20_Ok,RadioFRAMECLICK_Ok,RadioMQ2A_Ok,RadioMQ2B_Ok = 0;
__IO uint8_t RadioMsg_DS18B20[6];
__IO uint8_t RadioMsg_FRAMECLICK[6];
__IO uint8_t RadioMsg_Humidity[6];
__IO uint8_t RadioMsg_MQ2A[6];
__IO uint8_t RadioMsg_MQ2B[6];
__IO uint8_t RadioCmd[32];
__IO uint8_t HTS221_RxBuff[1];
__IO uint8_t HTS221_I2C_Ok = 0;
__IO uint16_t HUMIDITY_OUT = 0;
__IO uint8_t ASCII[16]= "0123456789ABCDEF";
uint8_t MyNameisPeterMsg[] = "This is the message from Shield\r\n";
__IO uint8_t HTS221_20H[2]={0x20,10000001U};				
__IO uint8_t HTS221_21H[2]={0x21,00000001U};
__IO uint8_t HTS221_28H[2]={0x28};
__IO uint8_t HTS221_29H[2]={0x29};
uint8_t LPUART1_RX_Buff[10];
static void MX_LPUART1_UART_Init(void);
void DMA1_Channel2_3_IRQHandler(void);
void MyRadioSend(uint8_t* msg);
static void MX_DMA_Init(void) ;
static void MX_LPUART1_UART_Init(void);


#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             20       // dBm

#if defined( USE_MODEM_LORA )


#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       8
// [SF7..SF12]
#define LORA_CODINGRATE                             1        // [1: 4/5,
																																//  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here
#define LED_PERIOD_MS               200

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;


									 
States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;
									 
static void MX_I2C1_Init(void);

 /* Led Timers objects*/
static  TimerEvent_t timerLed;

/* Private function prototypes -----------------------------------------------*/
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on when led timer elapses
 */
/**
 * Main application entry point.
 */
static UART_HandleTypeDef UartHandle;
int main( void )
{
  bool isMaster = true;
  uint8_t i;

  HAL_Init( );
  
  SystemClock_Config( );
  
  //DBG_Init( );

  HW_Init( );  	
	MX_DMA_Init();
	MX_LPUART1_UART_Init();
	MX_I2C1_Init();
	if(HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, 0xBE,HTS221_20H,2,I2C_FIRST_AND_LAST_FRAME)!=HAL_OK)
	{
		Error_Handler();
	}
	
	HAL_UART_Receive_DMA( &hlpuart1,  LPUART1_RX_Buff, 1);     

	
	
  /* Led Timers*/
  ////TimerInit(&timerLed, OnledEvent);   
  //TimerSetValue( &timerLed, LED_PERIOD_MS);

  //TimerStart(&timerLed );

  // Radio initialization
  //RadioEvents.TxDone = OnTxDone;
  //RadioEvents.RxDone = OnRxDone;
  //RadioEvents.TxTimeout = OnTxTimeout;
  //RadioEvents.RxTimeout = OnRxTimeout;
  //RadioEvents.RxError = OnRxError;
	
  Radio.Init( &RadioEvents );

  Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000000 );
    
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

  Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000000 );
    
  Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif
  HAL_UART_Receive_DMA( &UartHandle,  LPUART1_RX_Buff, 1);                             
  //Radio.Rx( RX_TIMEOUT_VALUE );
	//printf("HELLO\r\n");
	//HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1,0xBE	,HTS221_Cmd+2,1,I2C_FIRST_AND_LAST_FRAME);
	uint16_t HUMIDITY_INFLOAT = 0;
	while(1)
	{
		if(LocalACK_Ok)
		{
			LocalACK_Ok = 0;
			printf("@");
		}else if (LocalNACK)
		{
			LocalNACK = 0;
			printf("!");
		}
		if(RadioMQ2A_Ok)
		{
			RadioMQ2A_Ok = 0;
		//	MyRadioSend(RadioMsg_MQ2A);
		}else if(RadioMQ2B_Ok)
		{
			RadioMQ2B_Ok  =0;
		//	MyRadioSend(RadioMsg_MQ2B);
		}else if(RadioDS18B20_Ok)
		{	
		//	MyRadioSend(RadioMsg_DS18B20);
			RadioDS18B20_Ok = 0;
		}else if(RadioFRAMECLICK_Ok)
		{
		//	MyRadioSend(RadioMsg_FRAMECLICK);		
			RadioFRAMECLICK_Ok = 0;
		}else if(HTS221_I2C_Ok)
		{
			HTS221_I2C_Ok = 0; 
			HUMIDITY_INFLOAT = HUMIDITY_OUT/256/3 * 10;
			RadioMsg_Humidity[0] = 'H';
			RadioMsg_Humidity[5] = 'H';
			RadioMsg_Humidity[4] = ASCII[HUMIDITY_INFLOAT&0xf];
			RadioMsg_Humidity[3] = ASCII[(HUMIDITY_INFLOAT>>4)&0xf];
			RadioMsg_Humidity[2] = ASCII[(HUMIDITY_INFLOAT>>8)&0xf];
			RadioMsg_Humidity[1] = ASCII[(HUMIDITY_INFLOAT>>12)&0xf];
			//MyRadioSend(RadioMsg_Humidity);					
		}
		uint8_t i = 0;
		for(i = 0; i <6;i++)
		{
			RadioCmd[i]= RadioMsg_MQ2A[i];
			RadioCmd[i+6] = RadioMsg_MQ2B[i];
			RadioCmd[i+12] = RadioMsg_DS18B20[i];
			RadioCmd[i+18] = RadioMsg_FRAMECLICK[i];
			RadioCmd[i+24] = RadioMsg_Humidity[i];
			RadioCmd[30] = '\r';
			RadioCmd[31] = '\n';
			MyRadioSend(RadioCmd);
		}
	//MyRadioSend(MyNameisPeterMsg);
	//Radio.Send(MyNameisPeterMsg, 18);
	//Radio.Send( Buffer, BufferSize );
	}	
}
void MyRadioSend(uint8_t* msg)
{
	uint8_t i = 0;
	while(msg[i]!='\n')
	{
		//Radio.Send(msg+i, 1);
		i++;
	}
	Radio.Send(msg, i);
}
__IO uint8_t HTS221_Status = 0;
__IO uint8_t HUMIDITY_OUT_H, HUMIDITY_OUT_L = 0;
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1)
	{
		switch(HTS221_Status)
		{
			case 0:
				if(HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, 0xBE,HTS221_21H,2,I2C_FIRST_AND_LAST_FRAME)!=HAL_OK)
				{
					Error_Handler();
				}
				HTS221_Status++;
				break;
			case 1:
				if(HAL_I2C_Master_Transmit_IT(&hi2c1, 0xBE,HTS221_28H,1)!=HAL_OK)
				{
					Error_Handler();
				}

				HTS221_Status++;

				break;
			case 2:
				if(HAL_I2C_Master_Receive_IT(&hi2c1,0xBF,HTS221_28H+1,1)!=HAL_OK)
				{
					Error_Handler();
				}
				HTS221_Status = 3;
				break;
			case 4:
				if(HAL_I2C_Master_Receive_IT(&hi2c1,0xBF,HTS221_29H+1,1)!=HAL_OK)
				{
					Error_Handler();
				}
				HTS221_Status = 5;
				break;
		}
	}
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1)
	{	
		switch(HTS221_Status)
		{
			case 3:
				HUMIDITY_OUT_L = HTS221_28H[1];
				if(HAL_I2C_Master_Transmit_IT(&hi2c1,0xBE,HTS221_29H,1)!=HAL_OK)
				{
					Error_Handler();
				}
				HTS221_Status =4;
				break;
			case 5:
				HUMIDITY_OUT_H = HTS221_29H[1];
				HUMIDITY_OUT = HUMIDITY_OUT_H<<8|HUMIDITY_OUT_L;
				if(HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, 0xBE,HTS221_20H,2,I2C_FIRST_AND_LAST_FRAME)!=HAL_OK)
				{
					Error_Handler();
				}
				HTS221_I2C_Ok = 1;
				HTS221_Status =0;
				break;
		}
	}
}
__IO uint8_t RadioRx_Start_Ok = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart == &hlpuart1)
	{	
		if(!RadioRx_Start_Ok)
		{
			uint8_t rx_buff = 	LPUART1_RX_Buff[0];
			if(rx_buff == '$')
			{
				RadioRx_Start_Ok = 1;
				LocalACK_Ok = 1;
				HAL_UART_Receive_DMA( &hlpuart1,  LPUART1_RX_Buff, 6);   
			}else
			{
				LocalNACK = 1; 
				HAL_UART_Receive_DMA( &hlpuart1,  LPUART1_RX_Buff, 1);   
			}				
		}else
		{
			RadioRx_Start_Ok = 0 ;
			uint8_t rx_buff[6],i = 0;
			for(i = 0 ; i<6; i++)
			{
				rx_buff[i] = LPUART1_RX_Buff[i];
			}
			if(rx_buff[0]==rx_buff[5])
			{
				switch(rx_buff[0])
				{
					case 'M':
							RadioMsg_MQ2A[0] = rx_buff[0];
							RadioMsg_MQ2A[3] = rx_buff[3];
							RadioMsg_MQ2A[1] = rx_buff[1];
							RadioMsg_MQ2A[2] = rx_buff[2];
							RadioMsg_MQ2A[4] = rx_buff[4];
							RadioMsg_MQ2A[5] = rx_buff[5];

							RadioMQ2A_Ok = 1; 
							break;
					case 'N':
							RadioMsg_MQ2B[1] = rx_buff[1];
							RadioMsg_MQ2B[2] = rx_buff[2];
							RadioMsg_MQ2B[0] = rx_buff[0];
							RadioMsg_MQ2B[3] = rx_buff[3];
							RadioMsg_MQ2B[4] = rx_buff[4];
							RadioMsg_MQ2B[5] = rx_buff[5];

							RadioMQ2B_Ok = 1; 
							break;
					case 'T':
							RadioMsg_DS18B20[1] = rx_buff[1];
							RadioMsg_DS18B20[2] = rx_buff[2];
							RadioMsg_DS18B20[0] = rx_buff[0];
							RadioMsg_DS18B20[3] = rx_buff[3];
							RadioMsg_DS18B20[4] = rx_buff[4];
							RadioMsg_DS18B20[5] = rx_buff[5];
							RadioDS18B20_Ok = 1;
							break;
					case 'L':
							RadioMsg_FRAMECLICK[0] = rx_buff[0];	
							RadioMsg_FRAMECLICK[1] = rx_buff[1];
							RadioMsg_FRAMECLICK[2] = rx_buff[2];
							RadioMsg_FRAMECLICK[3] = rx_buff[3];
							RadioMsg_FRAMECLICK[4] = rx_buff[4];
							RadioMsg_FRAMECLICK[5] = rx_buff[5];
							RadioFRAMECLICK_Ok = 1;
							break;
					default:				
							break;
				}
			}
				HAL_UART_Receive_DMA( &hlpuart1,  LPUART1_RX_Buff, 1);   
		}
		//printf("%c",rx_buff);
		//while(1);
	}
}
void DMA1_Channel2_3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
}

static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}
int fputc(int ch , FILE *stream)
{
	//uint8_t x[1] = {ch};
	HAL_UART_Transmit(&hlpuart1,(uint8_t *)&ch,1,0xff);
	return ch;
}
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
	
  /**Configure Analogue filter 
  */
	
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
	
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
	
  /* USER CODE BEGIN I2C1_Init 2 */
	
  /* USER CODE END I2C1_Init 2 */
}

HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_ANALOG_FILTER(AnalogFilter));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State = HAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __HAL_I2C_DISABLE(hi2c);

    /* Reset I2Cx ANOFF bit */
    hi2c->Instance->CR1 &= ~(I2C_CR1_ANFOFF);

    /* Set analog filter bit*/
    hi2c->Instance->CR1 |= AnalogFilter;

    __HAL_I2C_ENABLE(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Configure I2C Digital noise filter.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2Cx peripheral.
  * @param  DigitalFilter Coefficient of digital noise filter between Min_Data=0x00 and Max_Data=0x0F.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter)
{
  uint32_t tmpreg;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_DIGITAL_FILTER(DigitalFilter));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State = HAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __HAL_I2C_DISABLE(hi2c);

    /* Get the old register value */
    tmpreg = hi2c->Instance->CR1;

    /* Reset I2Cx DNF bits [11:8] */
    tmpreg &= ~(I2C_CR1_DNF);

    /* Set I2Cx DNF coefficient */
    tmpreg |= DigitalFilter << 8U;

    /* Store the new register value */
    hi2c->Instance->CR1 = tmpreg;

    __HAL_I2C_ENABLE(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}