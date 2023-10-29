/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
// Standard libraries
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
// CPU
#include "mos6502.h"
// ROM
#include "basic_0xE000.h"
#include "wozMonitor_0xFF00.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Memory map
#define RAM_SIZE 0x8000
#define BASIC_START 0xE000
#define PIA_START 0xD010
#define WOZMON_START 0xFF00
// Display
#define TERM_WIDTH 40 // Font 8x10
#define TERM_HEIGHT 24
#define TERM_SIZE TERM_WIDTH * TERM_HEIGHT
// UART
#define UART_BAUD 115200
#define UART_LINE_ENDING "\r\n"
// CPU
#define CPU_FREQ 1000000 // 1 MHz
#define INSTRUCTION_CHUNK 10000
// Keyboard
#define KEYBOARD_READ_INTERVAL 10 // ms
#define SPACE_KEY 0x20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t read6502(uint16_t address);
void write6502(uint16_t address, uint8_t value);
void writelineTerminal(char *buffer);
void writeTerminal(char *buffer);
void readTerminal(char *buffer);
void initApple1(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Virtual Hardware
struct pia6821
{
  uint8_t keyboard_register; // 0xD010
  uint8_t keyboard_control;  // 0xD011
  uint8_t display_register;  // 0xD012
} pia = {0};

uint8_t ram[RAM_SIZE];
uint8_t displayBuffer[TERM_SIZE];
uint8_t keyboardBuffer[1];

/**
 * Read from memory (MOS 6502)
 */
uint8_t read6502(uint16_t address) { // Memory mapping for Apple I
  // RAM
  if (address < RAM_SIZE) {
    return ram[address];
  }

  // BASIC ROM
  if (address >= BASIC_START && address < BASIC_START + sizeof(basic)) {
    return basic[address - BASIC_START];
  }

  // PIA
  if (address >= PIA_START && address < 0xD013) {
    // Set keyboard register to 0x00 to indicate no key pressed
    if (address == PIA_START) {
      pia.keyboard_control = 0x00;
      return pia.keyboard_register;
    }
    // Display register
    else if (address == PIA_START + 2) {
      uint8_t display_register = pia.display_register;
      pia.display_register = 0x00;
      return display_register;
    }
  }

  // WOZMON ROM
  if (address >= WOZMON_START && address < 0xFFFF) {
    return monitor[address - WOZMON_START];
  }

  // Unmapped
  return 0xFF;
}

/**
 * Write to memory (MOS 6502)
 */
void write6502(uint16_t address, uint8_t value) {
  // RAM
  if (address < RAM_SIZE) {
    ram[address] = value;
    return;
  }

  // PIA
  if (address >= PIA_START && address < 0xD013) {
    if (address == PIA_START) {
      pia.keyboard_register = value;
      // Set keyboard register to 0xFF to indicate key pressed
      pia.keyboard_control = 0xFF;
    }
    else if (address == PIA_START + 1) {
      pia.keyboard_control = value;
    }
    else if (address == PIA_START + 2) {
      pia.display_register = value ^ 0x80;
    }
  }

}

/**
 * Write string with line ending to UART & LCD (STM32)
 */
void writelineTerminal(char *buffer) {
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY); HAL_UART_Transmit(&huart1, (uint8_t *)UART_LINE_ENDING, strlen(UART_LINE_ENDING), HAL_MAX_DELAY);
}

/**
 * Write string to UART & LCD without line ending (STM32)
 */
void writeTerminal(char *buffer) {
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
 * Read string from keyboard (UART & PS/2) (STM32)
 */
void readTerminal(char *buffer) {
  HAL_UART_Receive(&huart1, (uint8_t *)buffer, 1, KEYBOARD_READ_INTERVAL); // Read from UART
  // TODO: Read from PS/2

  /* Key Translation for Apple I */
  // Convert lowercase to uppercase
  if (buffer[0] >= 'a' && buffer[0] <= 'z') {
    buffer[0] -= 0x20;
  }
  // Convert LF to CR
  else if (buffer[0] == '\n') {
    buffer[0] = '\r';
  }
  // Convert backspace to rubout
  else if (buffer[0] == 0x7F) {
    buffer[0] = 0x7F;
  }
  // Ctrl + C to reset
  else if (buffer[0] == 0x03) {
    writelineTerminal("Resetting in 5 seconds...");
    HAL_Delay(5000);
    keyboardBuffer[0] = SPACE_KEY;
    initApple1();
  }
}

/**
 * Initialize Apple I
 */
void initApple1(void) {
  // Wait for space key to be pressed in UART
  do {
    HAL_Delay(1000);
    writelineTerminal("Press <space> to boot Apple I");
    readTerminal((char *)keyboardBuffer);
  } while (keyboardBuffer[0] != SPACE_KEY);

  // Initialize CPU
  writeTerminal("Initializing CPU...");
  reset6502(); writelineTerminal(" Complete");

  // Initialize RAM
  writeTerminal("Initializing RAM...");
  for (uint16_t i = 0; i < RAM_SIZE; i++) {
    ram[i] = 0x00;
  }
  writelineTerminal(" Complete");

  // Initialize PIA
  writeTerminal("Initializing PIA...");
  pia.keyboard_register = 0x00;
  pia.keyboard_control = 0x00;
  pia.display_register = 0x00;
  writelineTerminal(" Complete");
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  initApple1(); // Initialize Apple I
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read keyboard from UART and write to PIA
    readTerminal((char *)keyboardBuffer);
    write6502(PIA_START, keyboardBuffer[0]);
    // Execute instruction
    exec6502(INSTRUCTION_CHUNK);
    // Update display
    if (read6502(PIA_START + 2) != 0x00) {
      writeTerminal((char *)&pia.display_register);
    }
    // Debug
    // char debugMsg[100];
    // sprintf(debugMsg, "PIA: %02X %02X %02X", pia.keyboard_register, pia.keyboard_control, pia.display_register);
    // writelineTerminal(debugMsg);
    // Detect ChatGPT mode
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
     ex: printf("Wrong parameters value: file %s on line %d\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
