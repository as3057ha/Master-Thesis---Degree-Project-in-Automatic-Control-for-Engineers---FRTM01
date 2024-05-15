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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Output pins
#define PUL_PIN GPIO_PIN_7
#define PUL_PORT GPIOF

#define DIR_PIN GPIO_PIN_8
#define DIR_PORT GPIOF

#define ACT_PIN GPIO_PIN_9
#define ACT_PORT GPIOF

// Input pins
#define PROXOUT_PIN GPIO_PIN_1
#define PROXOUT_PORT GPIOF

#define PROXIN_PIN GPIO_PIN_2
#define PROXIN_PORT GPIOF

// Interrupt pins
#define INTA_PIN GPIO_PIN_4
#define INTA_PORT GPIOF

#define INTB_PIN GPIO_PIN_5
#define INTB_PORT GPIOF

// Communication pins
#define COM1_PIN GPIO_PIN_0
#define COM1_PORT GPIOC

#define COM2_PIN GPIO_PIN_1
#define COM2_PORT GPIOC





/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Constant declaration
const double PPR_m = 800;
const double PPR_enc = 800;
const double R_feed = 45 *1e-3;
const double R_enc = 45 * 1e-3;
const double N_f = 60;
const double N_m = 32;
const double C_pps = 2*M_PI*R_feed / (N_f*PPR_m);
const long SAMPLE_TIME = 50;

// Variable to store feed speed
float feedSpeed = 0.0; // Assuming it's declared elsewhere in your code

// Pulse related variables.
long currentTime;
long nextSample;
long nextPulse;
float pulseWidth;

//PI controller
double Kp = 1;
double Ki = 1;
double prevError;
double integral;

// Global variables for encoder count and direction
volatile int encoder_count = 0;
volatile int encoder_direction = 0;

// Variables for time measurement
volatile uint32_t last_interrupt_time = 0;
volatile uint32_t time_elapsed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

// Function prototype for interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// Calculation of pulse delay.
double calculate_pulse_delay();

double calculate_actual_feedSpeed();

// Function prototype for microsecond delay
void DWT_Delay_us(volatile uint32_t microseconds);

// Pulse function
void pulse();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  /* USER CODE BEGIN 2 */

  // Setup pins
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  GPIO_PinState actuate = HAL_GPIO_ReadPin(COM1_PORT, COM1_PIN);
	  GPIO_PinState feed = HAL_GPIO_ReadPin(COM2_PORT, COM2_PIN);
	  if(feed){

	currentTime = HAL_GetTick();
	// Checks if it is time to sample
	if(currentTime >= nextSample){
		HAL_GPIO_WritePin(ACT_PORT, ACT_PIN, GPIO_PIN_RESET);

	  double error = feedSpeed - calculate_actual_feedSpeed(feedSpeed);
	  integral += error;
	  double output = Kp*error + Ki*integral;
	  pulseWidth = calculate_pulse_delay(output);
	}
	// Checks if it is time pulse
	if(currentTime >= nextPulse){
	  pulse();
	}
	  }
	  else if(actuate){
		  HAL_GPIO_WritePin(ACT_PORT, ACT_PIN, GPIO_PIN_SET);
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Interrupt Service Routine for INTA
/* USER CODE BEGIN 4 */

// Interrupt Service Routine for INTA
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == INTA_PIN) // Check if interrupt is from INTA
    {

        // Determine direction of rotation by checking state of INTB
        if (HAL_GPIO_ReadPin(INTB_PORT, INTB_PIN) == GPIO_PIN_SET)
        {
            encoder_direction = 1; // Clockwise rotation
        }
        else
        {
            encoder_direction = -1; // Counter-clockwise rotation
        }
        encoder_count += encoder_direction;

        // You can add additional code here to handle the calculated RPM value
    }
}

// Calculate the necessary pulse delay
double calculate_pulse_delay(double adjustedFeedSpeed){
	nextSample = currentTime + SAMPLE_TIME;
	pulseWidth = (1 / adjustedFeedSpeed) * C_pps;
	return pulseWidth;
}

// Pulse pul pin
void pulse(){
	nextPulse = currentTime + pulseWidth;
	HAL_GPIO_WritePin(PUL_PORT, PUL_PIN, GPIO_PIN_SET); // Set pulse pin high
	//DWT_Delay_us(5); // Delay for 5 microseconds
	HAL_GPIO_WritePin(PUL_PORT, PUL_PIN, GPIO_PIN_RESET); // Set pulse pin low
}

// Function for microsecond delay using DWT cycle counter
void DWT_Delay_us(volatile uint32_t microseconds) {
    uint32_t clk_cycle_start = DWT->CYCCNT;
    // Delay loop
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds * (SystemCoreClock / 1000000));
}

double calculate_actual_feedSpeed(){
    double vFeedActual = 0.0;
    if (encoder_count != 0) {
        vFeedActual = 2 * M_PI * R_enc * PPR_enc * SAMPLE_TIME * 1e-3 / (60 * encoder_count);
    }
    encoder_count = 0;
    return vFeedActual;
}



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
