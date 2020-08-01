/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
//#define __FPU_PRESENT 1U
#include "arm_math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define adc_dma_size 4096
#define adc_samp_size adc_dma_size*2
#define tim_samp_size 1024
#define mag_avg_size 100
#define uart_rx_size 4
#define uart_tx_size 64 //might try and make each message configurable

//state definitions
#define WAIT 0
#define CAL 1
#define IMP 2
#define MAG 3
#define PHA 4
#define SEND 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int state = WAIT;
int state_prev = WAIT;
float32_t freq = 0;

//UART vars
uint8_t rxbuffer[uart_rx_size];
char message[uart_tx_size];
char delimiter[] = "\r\n";
int rx_flag = 0;

//mag vars
uint32_t adcdata[adc_dma_size];
float32_t adcfloat[adc_samp_size];
float32_t rms_val_arr[mag_avg_size];
float32_t rms_val_in = 0;
float32_t rms_val_dut = 0;
int rms_flag = 0;
float32_t threshold = 4096; //should lock to high resistance, actual val tbd in cal
float32_t r_val = 1000000;

float32_t mag = 0;

//phase vars
uint32_t cyc_diff_arr[tim_samp_size];
float32_t cyc_diff_float[tim_samp_size];
float32_t cyc_avg = 0;
int tim_flag = 0;

float32_t phase = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(uint32_t channel);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(uint32_t channel);
static void MX_ADC3_Init(uint32_t channel);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	//HAL_UART_Receive_IT(&huart2, (uint8_t*) rxbuffer , sizeof(rxbuffer));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	rms_flag = 1;

	//disable adc for single capture
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);
	HAL_ADC_Stop(&hadc3);

	/* old code
	//uint32_t localarray[128];
	//HAL_Delay(1000);
	//localarray = adcdata;
	 */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	tim_flag = 1;
	HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
	/* old code
	cyc_diff_arr = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);
	input_capture_2 = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);
	*/
}

void load_off(void){
	//TODO: change pin values when bodge complete
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //10
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //100
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //1000
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); //10 000
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //100 000
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //1 000 000
}

//resistor selecting
void load_sel(uint32_t val){
	load_off();
	HAL_Delay(20);

	switch(val){
	case 10: {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		break;
	}
	case 100: {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	}
	case 1000: {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		break;
	}
	case 10000: {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		break;
	}
	case 100000: {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		break;
	}
	default: { //selects largest impedance
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		break;
	}
	}
}

void run_adc_mag(uint32_t channel){
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);
	HAL_ADC_Stop(&hadc3);
	HAL_Delay(20);

	//channel0 is input - ADC_CHANNEL_0
	//channel1 in dut - ADC_CHANNEL_1
	MX_ADC1_Init(channel);
	MX_ADC2_Init(channel);
	MX_ADC3_Init(channel);

	for (int i=0; i<mag_avg_size; i++){
		//start ADC measurement
		HAL_ADC_Start(&hadc2);
		HAL_ADC_Start(&hadc3);
		if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) adcdata, adc_dma_size) != HAL_OK){
		  HAL_Delay(1000); // no bueno
		}

		while (rms_flag == 0){
			__NOP();
		}

		if (rms_flag == 1){
			int k = 0;
			uint16_t upper;
			uint16_t lower;
			for (int j=0; j<adc_dma_size; j++){
				lower = (uint16_t)adcdata[j];
				upper = (uint16_t)(adcdata[j] >> 16);
				adcfloat[k] = (float32_t)upper;
				adcfloat[k+1] = (float32_t)lower;
				k+=2;
			}
			arm_rms_f32( &adcfloat[0], adc_samp_size, &rms_val_arr[i]);
			HAL_Delay(20);
			rms_flag = 0;
		}
	}

	//compute mean
	if (channel == ADC_CHANNEL_0){
		arm_mean_f32(&rms_val_arr[0], mag_avg_size, &rms_val_in);
	} else if (channel == ADC_CHANNEL_1){
		arm_mean_f32(&rms_val_arr[0], mag_avg_size, &rms_val_dut);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  //MX_ADC1_Init(channel);
  MX_SPI1_Init();
  MX_TIM2_Init();
  //MX_ADC2_Init(channel);
  //MX_ADC3_Init(channel);
  /* USER CODE BEGIN 2 */
  load_sel(1000000);
  char message[] = "Receiving... \r\n";
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));
  if (HAL_UART_Receive(&huart2, (uint8_t*) rxbuffer , sizeof(rxbuffer), HAL_MAX_DELAY) != HAL_OK){
	  //assuming this means timeout
	  rx_flag = 0;
  } else {
	  rx_flag = 1;
  }

  /* old reference code
   * //THIS ACTUALLY WORKS
  if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) cyc_diff_arr, 10) != HAL_OK){
	  HAL_Delay(100);
  }

   // set PA0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  // reset PA0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) adcdata, adc_samp_size) != HAL_OK){
	  HAL_Delay(1000); // no bueno
  }

  oh baby a triple
  PB3 - jtag pin
  PA2 - uart
  PA3 - uart
  */
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(state) {
	  case WAIT: {//listen to uart

		  if (state_prev != WAIT){ //stuff that runs once - not sure if this will work
			  //turn the stuff off
			  HAL_ADC_Stop(&hadc1);
			  HAL_ADC_Stop(&hadc2);
			  HAL_ADC_Stop(&hadc3);

			  HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
			  HAL_Delay(20);

			  load_sel(1000000);

		  }
		  state_prev = state;

		  if (rx_flag == 0){
			  char message[] = "Receiving... \r\n";
			  HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));
			  if (HAL_UART_Receive(&huart2, (uint8_t*) rxbuffer , sizeof(rxbuffer), HAL_MAX_DELAY) != HAL_OK){
				  //assuming this means timeout
				  rx_flag = 0;
			  } else {
				  rx_flag = 1;
			  }

		  } else {
			  //process the rxbuffer
			  //choose next state
			  //char * temp =
			  if (rxbuffer[0] == 'c'){
				  state = CAL;
				  freq = 0.0;
			  } else if (rxbuffer[1] == 'k'){
				  state = IMP;
				  freq = 1000.0;
			  } else if	(rxbuffer[1] == '4'){
				  state = IMP;
				  freq = 10000.0;
			  } else if	(rxbuffer[1] == '5'){
				  state = IMP;
				  freq = 100000.0;
			  } else if	(rxbuffer[1] == 'm'){
				  state = IMP;
				  freq = 1000000.0;
			  } else {
				  state = WAIT;
				  freq = 0;
			  }
			  rx_flag = 0;
		  }
		  HAL_Delay(20);
		  break;
	  }
	  case CAL: {
		  //TODO not sure at the minute
		  char message[] = "Calibrating... \r\n";
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));

		  state = WAIT;
		  break;
	  }
	  case IMP: {
		  state_prev = state; //not in this state for more than one loop
		  char message[] = "Matching impedance... \r\n";
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));

		  //TODO work out bits to send for freq val (stored as a float in freq)
		  uint16_t data[5] = {0x2100, 0x69F1, 0x4000, 0xC000, 0x2000};
		  //turn on input signal via SPI
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //why this way round???
		  HAL_SPI_Transmit(&hspi1, (uint8_t*) data , sizeof(data)/sizeof(uint16_t), HAL_MAX_DELAY);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

		  for (int i=5; i>0; i--){
			  run_adc_mag(ADC_CHANNEL_1);
			  if (rms_val_in > threshold){
				  load_sel(pow(10,i));
				  r_val = (float32_t)pow(10,i);
			  } else {
				  break;
			  }
		  }
		  state = MAG;
		  break;
	  }
	  case MAG: {
		  state_prev = state;
		  //magnitude measurment
		  run_adc_mag(ADC_CHANNEL_1);
		  HAL_Delay(20);
		  run_adc_mag(ADC_CHANNEL_0);

		  //do the funky maths
		  mag = (rms_val_dut / rms_val_in) * r_val;

		  state = PHA;
		  break;
	  }
	  case PHA: {
		  state_prev = state;
		  //clear buffer
		  for (int i=0; i<tim_samp_size; i++){
			  cyc_diff_arr[i] = 0;
		  }
		  //phase measurement
		  if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) cyc_diff_arr, tim_samp_size) != HAL_OK){
			  HAL_Delay(100);
		  }

		  while (tim_flag == 0){
			  __NOP();
		  }

		  if (tim_flag == 1){
			  for (int i=0; i<tim_samp_size; i++){
				  cyc_diff_float[i] = (float32_t)cyc_diff_arr[i];
			  }
			  arm_mean_f32(&cyc_diff_float[0], tim_samp_size, &cyc_avg);

			  //avg number of cycles difference between zero crossing
			  //to go to time, divide by clock freq 180Mhz
			  float32_t time_diff;
			  time_diff = (cyc_avg / 180000000.0) ;

			  //not sure how to get the sign
			  phase = (time_diff * freq) * 360.0 ;
			  //if (phase > 180){
				  //phase = 180 - phase;
			  //}
			  tim_flag = 0;

		  }
		  state = SEND;
		  break;
	  }
	  case SEND: {
		  //send the magnitude and phase measurement over uart
		  char message[32];
		  sprintf(message, "%3f", mag);


		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));
		  HAL_Delay(20);
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)delimiter, strlen(delimiter));
		  HAL_Delay(20);
		  sprintf(message, "%3f", phase);
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));
		  HAL_Delay(20);
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)delimiter, strlen(delimiter));
		  HAL_Delay(20);
		  state = WAIT;
		  break;
	  }
	  }
	  HAL_Delay(20);

	  /*old reference code

	   //sending to SPI, blocking
	   uint16_t data[5] = {0x2100, 0x69F1, 0x4000, 0xC000, 0x2000};
	   HAL_SPI_Transmit(&hspi1, (uint8_t*) data , sizeof(data)/sizeof(uint16_t), HAL_MAX_DELAY);

	   //send over UART, non blocking
	   char message[] = "Send [B]ASS \r\n";
	   HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));

	   //send over uart, blocking example
	   HAL_UART_Transmit(UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size,uint32_t Timeout)

	   //send over uart, blocking example
	   HAL_UART_Receive(UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size,uint32_t Timeout)

	   if (rms_flag == 1){

		  //convert to float
		  //int j=0;
		  for (int i=0; i<adc_samp_size; i++){
			  adcfloat[i] = (float32_t)adcdata[i];
			  //if (i%16 == 0){
				//  adcfloat_u[j]  = (float32_t)adcdata[j];
				  //j++;
			  //}
		  }

		  //float32_t * pSrc,
		  //uint32_t blockSize,
		  //float32_t * pResult
		  arm_rms_f32( &adcfloat[0], adc_samp_size, &rms_val);
		  HAL_Delay(10);
		  //arm_rms_f32( &adcfloat_u[0], adc_samp_size/16, &rms_val_u);

		  HAL_Delay(1000);
		  rms_flag = 0;
	  } else if (rms_flag == 0) {
		  //restart ADC measurement
		  //HAL_ADC_Start(&hadc2);
		  //HAL_ADC_Start(&hadc3);
		  //if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) adcdata, adc_samp_size) != HAL_OK){
			//  HAL_Delay(1000); // no bueno
		  //}
	  }
	  HAL_Delay(10);*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(uint32_t channel)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_TRIPLEMODE_INTERL;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(uint32_t channel)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(uint32_t channel)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
