/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "OnePole.h"
#include "stm32f4xx_ll_usb.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// these buffers need to be divisible by 2 and 3, because (the little one at least)
// they are split into two halves, and contain a number of samples which is a
// multiple of 3.

#define NCHANNELS 3

#define PPS_CHANNEL 0
#define DETECT_CHANNEL 1
// record channel numbers FIRST_RECORD_CHANNEL to NCHANNELS-1
#define FIRST_RECORD_CHANNEL 1

// A "block" is half of the little buffer.
#define NBLOCKS 31 // discard first block
#define LITTLEBUFFERFILLS (NBLOCKS/2)
#define LITTLESAMPLES (512*2*NCHANNELS) // multiple of 2, 3, and 512
#define ADCBLOCKSIZE (LITTLESAMPLES/2)
// BIGSAMPLES needs to be a multiple of NCHANNELS, so that copying from the little buffer
// to the big buffer doesn't write off the end of the array.
#define BIGSAMPLES (512*2*(NCHANNELS-FIRST_RECORD_CHANNEL)*LITTLEBUFFERFILLS)//(LITTLESAMPLES*LITTLEBUFFERFILLS)



// Amount of change in ADC value required to detect a PPS rising edge on the PPS detect
// channel. 2800 seems to be a good default for the u-blox SAM-M8Q GPS module.
#define PPS_RISE_THRESHOLD 2800

// Output file info
#define FORMAT_VERSION 2
#define BITS_PER_CHANNEL 12
#define SAMPLES_SINCE_PPS_DIGITS 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd;

/* USER CODE BEGIN PV */

// We have to discard the first block, because it is always full of jittery,
// corrupt data.
bool firstBlock = false;
int operatingBlock = 0;
OnePoleFState_t onePoleState = {0};

uint16_t littleBuffer[LITTLESAMPLES] __attribute__((aligned(4))) = {0};//{0xafff};

uint16_t bigBuffer[BIGSAMPLES] __attribute__((aligned(4))) = {0};//{0xffff};

unsigned int bigBufferIndex = 0;
unsigned long samplesSincePPS = 0;

int counter = 0;

// lastSample is initialized to a large value so that the (sample - lastSample) test
// will never detect a rising edge in the first block
uint16_t sample = 0;
uint16_t lastSample = 4096;


enum systemState {STATE_INACTIVE, STATE_ARMED, STATE_FRESH_BLOCK, STATE_WRITE};
volatile enum systemState stateNow;

enum innerState {INNER_STATE_RECORD, INNER_STATE_NO_RECORD};
volatile enum innerState innerStateNow;

// cardstatetypedef:
/*
typedef uint32_t HAL_SD_CardStateTypeDef;

#define HAL_SD_CARD_READY          0x00000001U
#define HAL_SD_CARD_IDENTIFICATION 0x00000002U
#define HAL_SD_CARD_STANDBY        0x00000003U
#define HAL_SD_CARD_TRANSFER       0x00000004U
#define HAL_SD_CARD_SENDING        0x00000005U
#define HAL_SD_CARD_RECEIVING      0x00000006U
#define HAL_SD_CARD_PROGRAMMING    0x00000007U
#define HAL_SD_CARD_DISCONNECTED   0x00000008U
#define HAL_SD_CARD_ERROR          0x000000FFU
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USB_print(char* str) {
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

void delayWhile(volatile int usec) {
	// from https://hackaday.com/2020/12/11/bare-metal-stm32-blinky-and-the-secret-of-delay-functions/
	volatile int count = (usec * 168) / 4; // assume 168 MHz, 4 cycles per iteration
	while (count > 0) {
		count--;
	}
}
// Note: blink is not working. I have no idea why, but it does need to be fixed.
void blink(volatile int time) {
	__disable_irq();
	while (1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		delayWhile(time);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		delayWhile(time);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	//USB_print("ConvHalfCplt\n");
	operatingBlock = 0;
	stateNow = STATE_FRESH_BLOCK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//USB_print("ConvCplt\n");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	operatingBlock = 1;
	stateNow = STATE_FRESH_BLOCK;
}


bool littleToBigFor() {
	// These two loops can write off the end of bigBuffer if BLOCKSIZE is not a multiple of NCHANNELS.
	for (int i = 0; i < ADCBLOCKSIZE; i += NCHANNELS) {
		for (int j = FIRST_RECORD_CHANNEL; j < NCHANNELS; j += 1) {
			bigBuffer[bigBufferIndex] = littleBuffer[ ADCBLOCKSIZE*operatingBlock + i+j ];
			bigBufferIndex++;

			if (bigBufferIndex >= BIGSAMPLES) {
				return true;
			}
		}
	}
	return false;
}

bool detectOnChannel() {
	bool detected = false;
	for (int i = DETECT_CHANNEL; i < ADCBLOCKSIZE; i += NCHANNELS) {
		detected = OnePoleF_update(&onePoleState, (float)(littleBuffer[operatingBlock*ADCBLOCKSIZE+i]));
		if (detected) {
			// as soon as we detect, we want to leave the function.
			return true;
		}
	}
	return false;
}

void processPPS() {
	bool risingEdgeFound = false;

	for (int i = PPS_CHANNEL; i < ADCBLOCKSIZE; i += NCHANNELS) {
		// when whichBlock is 0, this will just be littleBuffer[i]
		sample = littleBuffer[ i + (ADCBLOCKSIZE*operatingBlock) ];
		if (sample > PPS_RISE_THRESHOLD && lastSample < PPS_RISE_THRESHOLD) {
			// PPS rising edge detect
			// reset samplesSincePPS
			USB_print("rising edge found\n");
			// divide by NCHANNELS because a block contains all the channels
			samplesSincePPS = (ADCBLOCKSIZE - i) / NCHANNELS;
			risingEdgeFound = true;
		}
		lastSample = sample;
	}

	if (!risingEdgeFound) {
		//USB_print("rising edge not found, incrementing counter\n");
		// if no rising edge was found, increment samplesSincePPS by ADCBLOCKSIZE
		samplesSincePPS += (ADCBLOCKSIZE / NCHANNELS);
	}
}
// whichBlock is the same as blockToCopy above
void freshBlockCallback() {
	switch (innerStateNow) {
	case INNER_STATE_RECORD:
		//USB_print("INNER_STATE_RECORD\n");
		;
		// the copy function should not be responsible for changing the state.
		bool bigBufferFull = littleToBigFor(); // copy first block

		if (bigBufferFull) {
			USB_print("big buffer full\n");
			// stop DMA so that we can write
			if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) {
				USB_print("failed to stop DMA\n");
				Error_Handler();
			}
			USB_print("moving to STATE_WRITE\n");
			stateNow = STATE_WRITE;
		} else {
			USB_print("big buffer not full\n");
			stateNow = STATE_ARMED;
		}

		break;

	case INNER_STATE_NO_RECORD:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		if (!firstBlock) {
			firstBlock = true;
			stateNow = STATE_ARMED;
			return; // discard first block.
		}
		// Look for detection first so that the case where PPS occurs in the same block as


		// if detected, currentlyRecording = true and copy this block to bigBuffer, then don't
		// look for PPS rising edge.
		if (detectOnChannel()) {

			USB_print("detected\n");
			innerStateNow = INNER_STATE_RECORD;
			// We don't change the outer state here, because the while loop
			// in main will send us back to this function, where littleToBigFor()
			// will be called (the inner state has changed).
			return;
		}

		// if not first block and no detection, look for the PPS rising edge
		// or increment the samples since the last edge.
		processPPS(); // modifies global variables
		stateNow = STATE_ARMED;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		break;
	}
}

FIL openNextFile() {
	int fileCounter = 0;
	char fileName[20] = {0};
	FILINFO fi;
	while (1) {
		sprintf(fileName, "LAMDA%03d.bin", fileCounter);
		FRESULT res = f_stat(fileName, &fi);
		// If we couldn't open this file, it is the one to go with.
		if (res == FR_NO_FILE) {
			USB_print("found file\n");
			FIL outputFile;
			res = f_open(&outputFile, fileName, FA_WRITE|FA_CREATE_ALWAYS);
			if (res != FR_OK) {
				// printing a buffer kills any chance of printing again, so we
				// print the filename last
				USB_print("failed to open output file for writing\n");
				USB_print(fileName);
				USB_print("\n");
				blink(500);
			}
			return outputFile;
		} else {
			fileCounter++;
		}
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(3000);

  FATFS fs = {0};
  FRESULT res;
  res = f_mount(&fs, "", 0);
  if (res != FR_OK) {
	  USB_print("failed to mount fs\n");
	  Error_Handler();
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  stateNow = STATE_INACTIVE;
  innerStateNow = INNER_STATE_NO_RECORD;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch (stateNow) {
	  case STATE_INACTIVE:
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) {
			  USB_print("armed");
			  stateNow = STATE_ARMED;
			  // I believe these numbers should be #defines.
			  OnePoleF_init(&onePoleState, 0.01, 0.007, 10);
			  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&littleBuffer, LITTLESAMPLES) != HAL_OK) {
				  USB_print("failed to start ADC DMA\n");
				  Error_Handler();
			  }
		  }
		  break;

	  case STATE_ARMED:
		  // Start ADC DMA and (indirectly) the detector.
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) != GPIO_PIN_SET) {
			  USB_print("dearmed");
			  stateNow = STATE_INACTIVE;
			  if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) {
				USB_print("failed to stop DMA\n");
				Error_Handler();
			  }
		  }
		  break;


	  case STATE_FRESH_BLOCK:
		  freshBlockCallback();
		  break;

	  case STATE_WRITE:
		  //innerStateNow = INNER_STATE_WAIT;
		  USB_print("STATE_WRITE");
		  // Stop DMA, write bigBuffer to the file, go back to STATE_ARMED and wait for detect again.
		  UINT written = 0;
		  FIL logFile = openNextFile();
		  char lamdaDeadbeef[] = {'L', 'A', 'M', 'D', 'A', 0xDE, 0xAD, 0xBE, 0xEF};
		  res = f_write(&logFile, lamdaDeadbeef, sizeof(lamdaDeadbeef), NULL);
		  if (res != FR_OK) {
			  USB_print("failed to write header to file\n");
			  blink(500);
		  }

		  const char fileInfo[] = {FORMAT_VERSION, NCHANNELS, BITS_PER_CHANNEL};
		  res = f_write(&logFile, fileInfo, sizeof(fileInfo), NULL);
		  if (res != FR_OK) {
			  USB_print("failed to write ADC data header to file\n");
			  blink(500);
		  }
		  // write samplesSincePPS
		  res = f_write(&logFile, (char*)(&samplesSincePPS), sizeof(samplesSincePPS), &written);
		  if (res != FR_OK) {
			  USB_print("failed to write samples since PPS to file\n");
			  blink(500);
		  }

		  res = f_write(&logFile, bigBuffer, sizeof(bigBuffer), &written);
		  if (res != FR_OK) {
			  USB_print("failed to write to file\n");
			  blink(500);
		  }
		  if (written != sizeof(bigBuffer)) {
			  USB_print("written doesn't match\n");
			  blink(1000);
		  }


		  res = f_sync(&logFile);
		  if (res != FR_OK) {
			  USB_print("failed to sync file\n");
			  blink(1000);
		  }

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		  res = f_close(&logFile);
		  if (res != FR_OK) {
			  USB_print("failed to close file\n");
			  blink(1500);
		  }
		  bigBufferIndex = 0;
		  samplesSincePPS = 0;
		  stateNow = STATE_INACTIVE;
		  innerStateNow = INNER_STATE_NO_RECORD;

		  break;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 16;
  /* USER CODE BEGIN SDIO_Init 2 */
  uint8_t sd_state = HAL_SD_Init(&hsd);
    /* Configure SD Bus width (4 bits mode selected) */
    if (sd_state == MSD_OK)
    {
      /* Enable wide operation */
      if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
      {
        sd_state = MSD_ERROR;
      }
    }
  /* USER CODE END SDIO_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Arming_pin__Feather_D5_Pin */
  GPIO_InitStruct.Pin = Arming_pin__Feather_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Arming_pin__Feather_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
