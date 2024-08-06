/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "External/RingBuffer.h"
#include "External/CanBus.h"
#include "External/Nextion.h"
#include "External/LoRa.h"

#include "stdio.h"
#include "string.h"
#include <stdbool.h>
#include <stdlib.h>

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// SD CARD
char stringValue[40];
char SdCardBuffer[30];
void process_SD_card(const char* message, int choise, int queue);

//CAN
uint8_t RxData[8];
uint8_t* pData;


// UART
uint8_t TxLora[20];
const uint16_t RxLoraSize = 4;
uint8_t RxLora[RxLoraSize];
uint32_t getClock;
bool flag = 0;
bool synchronizationFlag = 0;
uint32_t sonGonderilmeSaati;
uint32_t speed_fixed;

	// Flag'lar (Bayraklar)
	bool sw0 = 1; // Hız Göstergesi
	bool sw1 = 1; // Batarya Verileri
	bool sw2 = 1; // Veri Akış Hızı
	bool sw3 = 1; // Yer İstasyonu
	bool sw4 = 1; // SD Kart Verisi

	bool farAcikmi = 0;
	bool VeriUlastiFlag = 0;
	bool bms1_PacketNumber = 0;
	bool bms2_PacketNumber = 0;
	bool veriBiriktimi = 0;
	bool SdCardStatusFlag = 0;



// ADC
uint32_t adc1;

// GLOBAL


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	if (huart == &huart3) {
        if (RxLora[0] == 12 && RxLora[1] == 12 &&
        		RxLora[2] == 12 && RxLora[3] == 12)
        {
            flag = 1;
            getClock = HAL_GetTick();
            synchronizationFlag = 1;
        }

        for (int i = Size; i < RxLoraSize; i++)
        {
        	RxLora[i] = 0;
        }
	}

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxLora, sizeof(RxLora));
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	RingBuffer ringBuffer(800); // 800 Byte'lık bir döngüsel tampon.
	Nextion nextion(&huart1);
	LoRa lora(&huart3, 197, 19); // address: 199 | channel: 17


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxLora, sizeof(RxLora));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  CanBus can;

  for(int i = 0; i < 20; i++) {
	  TxLora[i] = 12;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      pData = can.Receive();
      for(uint8_t i = 0; i < 8; i++) {
    	  RxData[i] = pData[i];
      }
      for(int i = 0; i < 20; i++) {
    	  TxLora[i] += 1;
      }
      if(TxLora[0] == 254) {
		for(int i = 0; i < 20; i++) {
			TxLora[i] = 0;
		}
      }
      lora.Transmit(TxLora, sizeof(TxLora));
      lora.Receive(RxLora, sizeof(RxLora));

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
      //	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	  HAL_Delay(400);
    /* USER CODE END WHILE */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
