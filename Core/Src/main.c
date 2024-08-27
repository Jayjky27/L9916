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

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define L9916 I2C address and OTP register addresses
#define L9916_I2C_ADDRESS_WRITE 	0xD8
#define L9916_I2C_ADDRESS_READ  	0xD9
#define OTP_WRITE_VALUE_REGISTER    0x09
#define OTP_WRITE_COMMAND_REGISTER 	0x0A
#define OTP_STATUS_REGISTER     	0x0B
#define OTP_READ_REGISTER       	0x0C
#define REGISTER_1					0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef L9916_Write_OTP(uint8_t row, uint8_t column, uint8_t data);
HAL_StatusTypeDef L9916_Read_OTP(uint8_t *data);
void Entering_program_mode(void);
void OTP_programming(void);

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
  MX_USART2_UART_Init();
  MX_GPIO_Init();
  Entering_program_mode();
  __asm__("BKPT");

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* I2C1 GPIO Configuration
	 PB6  ------> I2C1_SCL
	 PB7  ------> I2C1_SDA
	*/

	/* Configure GPIO pins for PH and IGN */
	GPIO_InitStruct.Pin = GPIO_PIN_6; // PH pin (clock signal)
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7; // IGN pin (data sequence)
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef L9916_Write_OTP(uint8_t row, uint8_t column, uint8_t data)
{
    uint8_t write_command[2];
    uint8_t status;
    HAL_StatusTypeDef ret;

    // Write data to OTP data register
    write_command[0] = OTP_WRITE_VALUE_REGISTER;
    write_command[1] = data;
    ret = HAL_I2C_Master_Transmit(&hi2c1, L9916_I2C_ADDRESS_WRITE, write_command, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    // Specify the OTP row and column to write
    write_command[0] = OTP_WRITE_COMMAND_REGISTER;
    write_command[1] = (column << 4) | row;
    for (int i = 0; i < 3; i++) // Perform the write operation three times as required
    {
        ret = HAL_I2C_Master_Transmit(&hi2c1, L9916_I2C_ADDRESS_WRITE, write_command, 2, HAL_MAX_DELAY);
        if (ret != HAL_OK) return ret;

        // Wait until the write operation is completed
        do
        {
            ret = HAL_I2C_Mem_Read(&hi2c1, L9916_I2C_ADDRESS_READ, OTP_STATUS_REGISTER, 1, &status, 1, HAL_MAX_DELAY);
            if (ret != HAL_OK) return ret;
        } while (status & 0x04); // Check D2 bit
    }
    return HAL_OK;
}

HAL_StatusTypeDef L9916_Read_OTP(uint8_t *data)
{
    uint8_t write_command[2];
    HAL_StatusTypeDef ret;

    // Write data to OTP data register
    write_command[0] = REGISTER_1;
    ret = HAL_I2C_Master_Transmit(&hi2c1, L9916_I2C_ADDRESS_WRITE, write_command, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    ret = HAL_I2C_Master_Receive(&hi2c1, L9916_I2C_ADDRESS_READ, data, 1, HAL_MAX_DELAY);
    // Specify the OTP row and column to write

    return HAL_OK;
}

void Error_Handler(void)
{
  while(1)
  {
    // Stay in this loop in case of error
  }
}

void Entering_program_mode(void)
{
    // Assuming IGN pin is connected to GPIO_PIN_0 on GPIOA
    // Assuming PH pin is connected to GPIO_PIN_1 on GPIOB
    // Assuming DFM pin is connected to GPIO_PIN_2 on GPIOB

    // Set all input terminals to 0V
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // IGN
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // PH
    //HAL_Delay(1);  // Ensure all inputs are at 0V

    for (int i = 0; i < 16; i++)
    {
        if ((0xAFF3 >> (15 - i)) & 0x1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // Set IGN pin
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Reset IGN pin
        }
        //HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // PH
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // PH
        HAL_Delay(1);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // PH
    /*GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_6; // PH pin (clock signal)
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/

    MX_I2C1_Init();

    // Apply I2C protocol to confirm Device Program Mode within 500 ms
    uint8_t confirm_command [2];
    confirm_command[0] = REGISTER_1;
    confirm_command[1] = 0xA0;
    if (HAL_I2C_Master_Transmit(&hi2c1, L9916_I2C_ADDRESS_WRITE, confirm_command, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }
}

void OTP_Write(uint8_t value, uint8_t command)
{
    uint8_t data[2];

    // Zápis hodnoty do OTP_WRITE_VALUE_REGISTER
    data[0] = OTP_WRITE_VALUE_REGISTER;
    data[1] = value;
    if (HAL_I2C_Master_Transmit(&hi2c1, L9916_I2C_ADDRESS_WRITE, data, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }

    // Zápis příkazu do OTP_WRITE_COMMAND_REGISTER
    data[0] = OTP_WRITE_COMMAND_REGISTER;
    data[1] = command;
    if (HAL_I2C_Master_Transmit(&hi2c1, L9916_I2C_ADDRESS_WRITE, data, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }
}

void OTP_WaitForCompletion(void)
{
    uint8_t buffer;
    uint8_t statusCommand = OTP_STATUS_REGISTER;

    HAL_I2C_Master_Transmit(&hi2c1, L9916_I2C_ADDRESS_WRITE, &statusCommand, 1, HAL_MAX_DELAY);

    do {
        HAL_I2C_Master_Receive(&hi2c1, L9916_I2C_ADDRESS_READ, &buffer, 1, HAL_MAX_DELAY);
    } while (!(buffer & 0x02));
}

void OTP_programming(void)
{
    OTP_Write(0xA0, 0x20); // OTP Row 0
    OTP_WaitForCompletion();

    OTP_Write(0x40, 0x21); // OTP Row 1
    OTP_WaitForCompletion();

    OTP_Write(0xC0, 0x22); // OTP Row 2
    OTP_WaitForCompletion();

    OTP_Write(0x18, 0x23); // OTP Row 3
    OTP_WaitForCompletion();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

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
