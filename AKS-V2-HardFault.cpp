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
#include "fatfs.h"
#include "spi.h"
#include "adc.h"
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

bool fsFlag = 1;
uint8_t fsQueue = 0;
void process_SD_card(const char* message, int choise, int queue);

//CAN
uint8_t RxData[8];
uint8_t* pData;
uint8_t TxCan[8];

// UART
uint8_t TxLora[27];
uint8_t gecici[27];

const uint16_t RxLoraSize = 4;
uint8_t RxLora[RxLoraSize];
volatile uint32_t getClock;
volatile bool flag = 0;
volatile bool synchronizationFlag = 0;
uint32_t sonGonderilmeSaati;
uint32_t speed_fixed;


#define RxNextionBufferSize 20
uint8_t RxNextionBuffer[RxNextionBufferSize];

	// Flag'lar (Bayraklar)
	volatile bool sw0 = 1; // Hız Göstergesi
	volatile bool sw1 = 1; // Batarya Verileri
	volatile bool sw2 = 1; // Veri Akış Hızı
	volatile bool sw3 = 1; // Yer İstasyonu
	volatile bool sw4 = 1; // SD Kart Verisi

	volatile bool farAcikmi = 0;
	bool VeriUlastiFlag = 0;
	bool bms1_PacketNumber = 0;
	bool bms2_PacketNumber = 0;
	bool veriBiriktimi = 0;
	bool SdCardStatusFlag = 0;



// ADC
uint32_t adc1;

// GLOBAL
int deger;

uint16_t interrupt_counter;
uint16_t veriAkisHiziDelay = 650;

RingBuffer ringBuffer(800); // 800 Byte'lık bir döngüsel tampon.
Nextion nextion(&huart1);


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {



    if (huart == &huart1) { // EGER NEXTION EKRANINDAN BILGI GELIRSE
        // Farı aç
        if (strncmp((const char *)RxNextionBuffer, "LediYak", strlen("LediYak")) == 0)
        {
            if (farAcikmi == 0)
            {
//                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
                farAcikmi = 1;
            }
        }

        // Farı kapat
        if (strncmp((const char *)RxNextionBuffer, "LediSondur", strlen("LediSondur")) == 0)
        {
            if (farAcikmi == 1)
            {
//                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
                farAcikmi = 0;
            }
        }

        // Veri Akış hızı Hızlı/Yavaş
        if (strncmp((const char *)RxNextionBuffer, "VeriAkisHizi", strlen("VeriAkisHizi")) == 0)
        {
            if (sw2 == 1)
            {
                sw2 = 0;
                veriAkisHiziDelay = 1300;
            }
            else
            {
                sw2 = 1;
                veriAkisHiziDelay = 650;
            }
        }

        // Batarya Verileri Açık/Kapalı
        if (strncmp((const char *)RxNextionBuffer, "BataryaVerileri", strlen("BataryaVerileri")) == 0)
        {
            if (sw1 == 1)
            {
                sw1 = 0;
            }
            else
            {
                sw1 = 1;
            }
        }

        // Loraya Giden Veri Açık/Kapalı
        if (strncmp((const char *)RxNextionBuffer, "YerIstasyonuVerisi", strlen("YerIstasyonuVerisi")) == 0)
        {
            if (sw3 == 1)
            {
                sw3 = 0;
                nextion.SetNumber("sw3", 0);
            }
            else
            {
                sw3 = 1;
                nextion.SetNumber("sw3", 1);
            }
        }

        // Hız Göstergesi Açık/Kapalı
        if (strncmp((const char *)RxNextionBuffer, "HizGostergesi", strlen("HizGostergesi")) == 0)
        {
            if (sw0 == 1)
            {
                sw0 = 0;
                nextion.SetNumber("sw0", 0);
                nextion.SetNumber("n11", 0);
                nextion.SetNumber("z0", 270);
            }
            else
            {
                sw0 = 1;
                nextion.SetNumber("sw0", 1);
            }
        }


        if (strncmp((const char *)RxNextionBuffer, "SdCardVerisi", strlen("SdCardVerisi")) == 0)
        {
            if (sw4 == 1)
            {
                sw4 = 0;
                nextion.SetNumber("sw4", 0);
            }
            else
            {
                sw4 = 1;
                nextion.SetNumber("sw4", 1);
            }
        }


        interrupt_counter += 1;

        for (int i = Size; i < RxNextionBufferSize; i++)
        {
            RxNextionBuffer[i] = 0;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxNextionBuffer, sizeof(RxNextionBuffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }









	if (huart == &huart3) {
        if (RxLora[0] == 12 && RxLora[1] == 12 &&
        		RxLora[2] == 12 && RxLora[3] == 12)
        {
            flag = 1;
            getClock = HAL_GetTick();
            synchronizationFlag = 1;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxLora, sizeof(RxLora));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}


}


bool synchronization() {
//	__disable_irq(); // Kesmeleri devre dışı bırak
	for (uint8_t i = 0; i < sizeof(TxLora) - 1; i++) {
		deger = ringBuffer.read();
		if (deger != -1) {
			gecici[i] = deger;
		}
		else return 0;
	}
	return 1;
//	__enable_irq();
}

void clearRxData() {
	for(int i = 0; i < 8; i++) {
		RxData[i] = 0;
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


	LoRa lora(&huart3, 196, 12); // address: 199 | channel: 17




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
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxNextionBuffer, sizeof(RxNextionBuffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxLora, sizeof(RxLora));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  CanBus can(0x401);

//  for(int i = 0; i < 20; i++) {
//	  TxLora[i] = 12;
//  }



  HAL_Delay(500);
  process_SD_card(stringValue, 2, fsQueue);
  uint8_t counter = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  HAL_Delay(veriAkisHiziDelay);


	  // Herhangi bir can alıcı kodu başlangıç
//	can.Receive(RxData, sizeof(RxData));
//
//	for(int i = 0; i < 27; i++) {
//	  TxLora[i] += 1;
//	}
//	if(TxLora[0] == 254) {
//		for(int i = 0; i < 27; i++) {
//			TxLora[i] = 0;
//		}
//	}


//	if (sw0)
//	{
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, 100);
//		adc1 = HAL_ADC_GetValue(&hadc1);
//		speed_fixed = ((adc1 * 80) / 4096);
//	}
//









//	  RxData[0] = 10;
//	  RxData[1] = 10;
//	  RxData[2] = 10;
//	  RxData[3] = 10;
//
//	  RxData[4] = 12;
//	  RxData[5] = 12;
//	  RxData[6] = 12;
//	  RxData[7] = 12;
	  if (sw1)
	  {
		  if (RxData[0] == 1)
		  {
			  for (int i = 0; i < 5; i++)
			  {
				  char buffer[10]; // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
				  sprintf(buffer, "j%d", i + 1);
				  nextion.SetNumber(buffer, RxData[i + 1]);
				  nextion.SetString("statuscodetext", "1. Batarya Paketi verileri alınıyor...");
			  }
		  }

		  else if (RxData[0] == 2)
		  {
			  for (int i = 0; i < 5; i++)
			  {
				  char buffer[10];                          // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
				  sprintf(buffer, "j%d", i + 6);            // "j1", "j2", ..., "j8" şeklinde metin oluştur
				  nextion.SetNumber(buffer, RxData[i + 1]); // Oluşturulan metni ve RxData dizisindeki değeri gönder
				  nextion.SetString("statuscodetext", "2. Batarya Paketi verileri alınıyor...");
			  }
		  }

		  else if (RxData[0] == 3)
		  {
			  for (int i = 0; i < 5; i++)
			  {
				  char buffer[10];                          // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
				  sprintf(buffer, "j%d", i + 11);           // "j1", "j2", ..., "j8" şeklinde metin oluştur
				  nextion.SetNumber(buffer, RxData[i + 1]); // Oluşturulan metni ve RxData dizisindeki değeri gönder
				  nextion.SetString("statuscodetext", "3. Batarya Paketi verileri alınıyor...");
			  }
		  }

		  else if (RxData[0] == 4)
		  {
			  for (int i = 0; i < 5; i++)
			  {
				  char buffer[10];                          // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
				  sprintf(buffer, "j%d", i + 16);           // "j1", "j2", ..., "j8" şeklinde metin oluştur
				  nextion.SetNumber(buffer, RxData[i + 1]); // Oluşturulan metni ve RxData dizisindeki değeri gönder
				  nextion.SetString("statuscodetext", "4. Batarya Paketi verileri alınıyor...");
			  }
		  }
	  }

	  nextion.SetVisibility("beniaksyebagla", 0);
	  nextion.SetVisibility("bataryauyari", 0);





//	  if (sw0)
//	  {
//		  if (speed_fixed <= 60)
//		  {
//			  nextion.SetNumber("n11", speed_fixed);
//			  nextion.SetNumber("z0", (3 * (speed_fixed / 2)) + 270);
//		  }
//
//		  if (speed_fixed > 60)
//		  {
//			  nextion.SetNumber("n11", speed_fixed);
//			  nextion.SetNumber("z0", (3 * (speed_fixed / 2)) - 90);
//		  }
//	  }
//
//


	  if (sw3)
	  {
		  clearRxData();
		  // BİRİNCİ BMS PAKETİNİ GEÇİR:
		  TxCan[1] = 1;
//		  can.Transmit(TxCan, 8);
		  HAL_Delay(50);
//		  pData = can.Receive();
		  HAL_Delay(50);
		  counter++;
		  if(counter == 254) counter = 0;
		  if(RxData[0] == 1) {
			  TxLora[3] = counter;// RxData[1]; // CELL 1
			  TxLora[4] = counter;// RxData[2]; // CELL 2
			  TxLora[5] = counter;// RxData[3]; // CELL 3
			  TxLora[6] = counter;// RxData[4]; // CELL 4
			  TxLora[7] = counter;// RxData[5]; // CELL 5
		  }
		  HAL_Delay(50);
		  clearRxData();


		  // İKİNCİ BMS PAKETİNİ GEÇİR:
		  TxCan[1] = 2;
//		  can.Transmit(TxCan, 8);
		  HAL_Delay(50);
//		  pData = can.Receive();
	  	  HAL_Delay(50);
		  if(RxData[0] == 2) {
			  TxLora[8]  = counter;// RxData[1]; // CELL 1
			  TxLora[9]  = counter;// RxData[2]; // CELL 2
			  TxLora[10] = counter;// RxData[3]; // CELL 3
			  TxLora[11] = counter;// RxData[4]; // CELL 4
			  TxLora[12] = counter;// RxData[5]; // CELL 1
		  }
		  HAL_Delay(50);
		  clearRxData();


	  // ÜÇÜNCÜ BMS PAKETİNİ GEÇİR:
		  TxCan[1] = 3;
//		  can.Transmit(TxCan, 8);
		  HAL_Delay(50);
//		  pData = can.Receive();
		  HAL_Delay(50);
		  if(RxData[0] == 3) {
			  TxLora[13] = counter;// RxData[1];
			  TxLora[14] = counter;// RxData[2];
			  TxLora[15] = counter;// RxData[3];
			  TxLora[16] = counter;// RxData[4];
			  TxLora[17] = counter;// RxData[5];
		  }
		  HAL_Delay(50);
		  clearRxData();


		  // DÖRDÜNCÜ BMS PAKETİNİ GEÇİR:
		  TxCan[1] = 4;
//		  can.Transmit(TxCan, 8);
		  HAL_Delay(50);
//		  pData = can.Receive();
		  HAL_Delay(50);
		  if(RxData[0] == 4) {
			  TxLora[18] = counter;// RxData[1];
			  TxLora[19] = counter;// RxData[2];
			  TxLora[20] = counter;// RxData[3];
			  TxLora[21] = counter;// RxData[4];
			  TxLora[22] = counter;// RxData[5];
		  }
		  HAL_Delay(30);




		  for (uint8_t i = 0; i < sizeof(TxLora) - 1; i++)
		{
			__disable_irq(); // Kesmeleri devre dışı bırak
			ringBuffer.write(TxLora[i]);
			__enable_irq();
		}



		// Flag kalktıktan sonra 3'lü veri seti gönderme yaptığımız yer
		if (synchronizationFlag == 1 && sw3)
		{
			if(synchronization()) {
				lora.Transmit(gecici, sizeof(gecici));
				HAL_Delay(250);
			}

		if(synchronization()) {
				lora.Transmit(gecici, sizeof(gecici));
				HAL_Delay(250);
			}

			if(synchronization()) {
				lora.Transmit(gecici, sizeof(gecici));
				HAL_Delay(250);
			}
			sonGonderilmeSaati = HAL_GetTick();

			flag = 0;
			synchronizationFlag = 0;
		}
	  }


	gecici[26] = 155;
	lora.Transmit(gecici, sizeof(gecici));
	gecici[26] = 0;

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);



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







#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  // HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// flagControl fonksiyonunu process_SD_card dışında tanımladık
void flagControl(FRESULT fres) {
  if (fres != FR_OK) {
    SdCardStatusFlag = 0;
    // printf("No SD Card found : (%i)\r\n", fres);
  } else {
    SdCardStatusFlag = 1;
  }
}

void process_SD_card(const char* message, int choise, int queue)
{
  FATFS       FatFs;                //Fatfs handle
  FIL         fil;                  //File handle
  UINT bw;
  FRESULT     fres;                 //Result after operations
  char        buf[100];

  do
  {
    //Mount the SD Card
    fres = f_mount(&FatFs, "", 1);    //1=mount now
    flagControl(fres); // flagControl fonksiyonunu burada çağırıyoruz

    // printf("SD Card Mounted Successfully!!!\r\n");

    // Read the SD Card Total size and Free Size
    FATFS *pfs;
    DWORD fre_clust;
//    uint32_t totalSpace, freeSpace;

    f_getfree("", &fre_clust, &pfs);
//    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
//    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

    // printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);

    // Open the file

    // printf("Writing data!!!\r\n");
    // write the data

    // DOSYA İSMİ SAPTANDI. YAZDIRMA İŞLEMLERİNİ YAP.
    if (choise == 1) {
      sprintf(stringValue, "log%d.txt", queue);
      fres = f_open(&fil, stringValue, FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
      flagControl(fres);
      sprintf(stringValue, "deger: %s", message);

      f_lseek(&fil, f_size(&fil));
      f_write(&fil, message, strlen(message), &bw);
      //close your file
      f_close(&fil);
    }

    // DOSYA İSMİ İÇİN SIRA AL
    if (choise == 2) {
      fres = f_open(&fil, "queue.txt", FA_READ);
      flagControl(fres);
      //read the data
      f_gets(buf, sizeof(buf), &fil);
      fsQueue = atoi(buf);
      f_close(&fil);

      fres = f_open(&fil, "queue.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
      flagControl(fres);
      f_lseek(&fil, f_size(&fil));
      sprintf(stringValue, "%d", fsQueue + 1);
      f_write(&fil, stringValue, strlen(stringValue), &bw);
      f_close(&fil);
    }

#if 0
    //Delete the file.
    fres = f_unlink("Zeugma.txt");
    if (fres != FR_OK)
    {
      // printf("Cannot able to delete the file\n");
    }
#endif
  } while (false);

  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);
  // printf("SD Card Unmounted Successfully!!!\r\n");
}






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
