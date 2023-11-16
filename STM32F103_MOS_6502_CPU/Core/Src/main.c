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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Standard libraries
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
// MOS 6502
#include "mos6502.h"
// LCD
#include "lcd.h"
// PS2
#include "ps2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Memory map
#define BASIC_START 0xE000
#define PIA_KEYBOARD_REG 0xD010
#define PIA_KEYBOARD_CTRL 0xD011
#define PIA_DISPLAY_REG 0xD012
#define PIA_DISPLAY_CTRL 0xD013
#define WOZMON_START 0xFF00
// Display
#define TERM_WIDTH 40 // Font 8x10
#define TERM_HEIGHT 24
#define TERM_SIZE TERM_WIDTH * TERM_HEIGHT
#define FONT_WIDTH 8
#define FONT_HEIGHT 10
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
// #define TO_APPLE1CHAR(asciichar) (asciichar + 0x80);
// #define TO_ASCII(ascii) ascii & 0x7F;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SD_HandleTypeDef hsd;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FSMC_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */
// MOS 6502
uint8_t read6502(uint16_t address);
void write6502(uint16_t address, uint8_t value);
// UART & LCD
void handleInput(char *buffer);
void handleOutput(uint8_t value);
// Apple I Initialization
void initApple1(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// LCD Hardware
uint8_t LCD_CURSOR_X = 0; // 0..39
uint8_t LCD_CURSOR_Y = 0; // 0..23

// Virtual Hardware
struct pia6821
{
  uint8_t keyboard_register; // 0xD010 (Input)
  uint8_t keyboard_control;  // 0xD011 (KB Ready)
  uint8_t display_register;  // 0xD012 (Output)
  uint8_t display_control;   // 0xD013 (ATTR)
} pia = {0};

// RAM
uint8_t RAM[RAM_SIZE];
uint8_t keyboardBuffer[1] = {0x00};

// SD Card FS
FATFS SDFatFs;

/**
 * Read from memory (MOS 6502)
 */
uint8_t read6502(uint16_t address) { // Memory mapping for Apple I
  uint16_t BASIC_addr, monitor_addr;

  // RAM
  if (address < RAM_SIZE) return RAM[address];

  // PIA (ACIA 6821)
  if (address == PIA_KEYBOARD_REG) {

    // Debug
    // char debugMsg[100];
    // sprintf(debugMsg, "read6502()::PIA_KEYBOARD_REG at 0x%04X", address);
    // writelineTerminal(debugMsg);

    return pia.keyboard_register;
  }
  if (address == PIA_KEYBOARD_CTRL) {

    // Debug
    // char debugMsg[100];
    // sprintf(debugMsg, "read6502()::PIA_KEYBOARD_CTRL at 0x%04X", address);
    // writelineTerminal(debugMsg);

    // Read from keyboard
    handleInput((char *)keyboardBuffer);
    pia.keyboard_register = keyboardBuffer[0] | 0x80;

    if (keyboardBuffer[0] != 0x00) {
      keyboardBuffer[0] = 0x00;
      return 0x80;
    }
    else
      return 0x00;
  }

  // BASIC ROM
  if (address >= BASIC_START && address <= 0xEFFF) {

    // Debug
    // char debugMsg[100];
    // sprintf(debugMsg, "read6502()::BASIC_ROM at 0x%04X", address);
    // writelineTerminal(debugMsg);

    BASIC_addr = address - 0xE000;
    #if ASSEMBLER
      return BASIC[BASIC_addr];
    #else
      return A1AE0[BASIC_addr];
    #endif
  }

  // WOZMON ROM
  if (address >= 0xF000) {

    // Debug
    // char debugMsg[100];
    // sprintf(debugMsg, "read6502()::WOZMON_ROM at 0x%04X", address);
    // writelineTerminal(debugMsg);

    monitor_addr = (address - 0xF000) & 0xFF; // get => 0..255 for woz rom
    if (monitor_addr < 0x100) return monitor[monitor_addr];
  }

  // Unmapped
  return 0x00;
}

/**
 * Write to memory (MOS 6502)
 */
void write6502(uint16_t address, uint8_t value) {
  // RAM
  if (address < RAM_SIZE) {
    RAM[address] = value;
  }

  // PIA (ACIA 6821)
  if (address == PIA_DISPLAY_REG) {
    pia.display_register = value;
    value &= 0x7F;
    handleOutput(value);
  }
}

/**
 * Write string with line ending to UART & LCD (STM32)
 */
void writelineTerminal(char *buffer) {
  // UART
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY); HAL_UART_Transmit(&huart1, (uint8_t *)UART_LINE_ENDING, strlen(UART_LINE_ENDING), HAL_MAX_DELAY);

  // LCD
  LCD_DrawString(LCD_CURSOR_X * FONT_WIDTH, LCD_CURSOR_Y * FONT_HEIGHT, buffer);
  LCD_CursorNewline();
}

/**
 * Write string to UART & LCD without line ending (STM32)
 */
void writeTerminal(char *buffer) {
  // UART
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

  // LCD
  uint8_t tempX = LCD_CURSOR_X;
  uint8_t tempY = LCD_CURSOR_Y;
  for (uint8_t i = 0; i < strlen(buffer); i++) {
    LCD_CursorForward();
  }
  LCD_DrawString(tempX * FONT_WIDTH, tempY * FONT_HEIGHT, buffer);
}

/**
 * Write char to UART & LCD without line ending (STM32)
 */
void writeTerminalChar(char *buffer) {
  // UART
  if (LCD_CURSOR_X + 1 == TERM_WIDTH) {
    HAL_UART_Transmit(&huart1, (uint8_t *)UART_LINE_ENDING, strlen(UART_LINE_ENDING), HAL_MAX_DELAY);
  } else {
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, 1, HAL_MAX_DELAY);
  }

  // LCD
  uint8_t tempX = LCD_CURSOR_X;
  uint8_t tempY = LCD_CURSOR_Y;
  char tempChar[2] = {0x00};
  tempChar[0] = buffer[0];
  tempChar[1] = '\0';

  LCD_CursorForward();
  LCD_DrawString(tempX * FONT_WIDTH, tempY * FONT_HEIGHT, tempChar);
}

/**
 * Read string from keyboard (UART & PS/2) (STM32)
 */
void handleInput(char *buffer) {
  HAL_UART_Receive(&huart1, (uint8_t *)buffer, 1, KEYBOARD_READ_INTERVAL); // Read from UART
  // TODO: Read from PS/2
  if(isKbrdReady()){
	  buffer[0] = getAscii();
  }
  buffer[0] &= 0x7F; // Mask out MSB
  /* Key Translation for Apple I */
  // Convert lowercase to uppercase
  if (buffer[0] >= 'a' && buffer[0] <= 'z') {
    buffer[0] -= 0x20;
  }
  // Convert LF to CR
  else if (buffer[0] == '\n') {
    buffer[0] = '\r';
  }
  else if (buffer[0] == '\b') {
    buffer[0] = 0x5F;
  }
  // Convert backspace to rubout
  else if (buffer[0] == 0x7F) {
    buffer[0] = '_';
  }
  // Ctrl + C to reset
  else if (buffer[0] == 0x03) {
    writelineTerminal("[Ctrl + C] Resetting in 5 seconds...");
    HAL_Delay(5000);
    keyboardBuffer[0] = SPACE_KEY;
    initApple1();
  }
  // Ctrl + S to clear screen
  else if (buffer[0] == 0x13) {
    // UART
    const char *clearScreen = "\x1B\x5B\x32\x4A"; // ANSI escape sequence to clear screen
    HAL_UART_Transmit(&huart1, (uint8_t *)clearScreen, strlen(clearScreen), HAL_MAX_DELAY);

    // LCD
    LCD_Clear(0, 0, 320, 240, BACKGROUND);
    LCD_CURSOR_X = 0;
    LCD_CURSOR_Y = 0;
  }
  // Ctrl + L to load tapes
  else if (buffer[0] == 0x0C) {
    writelineTerminal("[Ctrl + L] Loading tapes...");
    writelineTerminal("Enter Filename: apple30th.0280.0FFF.bin");
    // Input filename
    // char filename[64] = {0x00};
    // while (keyboardBuffer[0] != '\r' || keyboardBuffer[0] != '\n') {
    //   HAL_UART_Receive(&huart1, (uint8_t *)keyboardBuffer, 1, KEYBOARD_READ_INTERVAL); // Read from UART
    //   // TODO: Read from PS/2
    //   if (keyboardBuffer[0] == '\r' || keyboardBuffer[0] == '\n') break;
    //   writeTerminalChar((char *)keyboardBuffer);
    //   strcat(filename, (char *)keyboardBuffer);
    //   keyboardBuffer[0] = 0x00;
    // }
    // writelineTerminal("");
    // Load tape
    char filename[64] = "apple30th.0280.0FFF.bin";
    tapeLoading(filename);
  }
}

/**
 * Write character to UART & LCD (MOS 6502)
 */
void handleOutput(uint8_t value) {
  if (value == 13) {
    // UART Newline
    HAL_UART_Transmit(&huart1, (uint8_t *)UART_LINE_ENDING, strlen(UART_LINE_ENDING), HAL_MAX_DELAY);
    // LCD Newline
    LCD_Clear(LCD_CURSOR_X * FONT_WIDTH, LCD_CURSOR_Y * FONT_HEIGHT, FONT_WIDTH, FONT_HEIGHT, BLACK);
    if (++LCD_CURSOR_Y > 23) {
		  LCD_CURSOR_Y = 0;
		  // Clear the screen
		  LCD_Clear(0, 0, 320, 240, BACKGROUND);
	  }
	  LCD_CURSOR_X = 0;
  }
  else {
    writeTerminalChar((char *)&value);
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
    handleInput((char *)keyboardBuffer);
  } while (keyboardBuffer[0] != SPACE_KEY);
  keyboardBuffer[0] = 0x00;

  // Initialize CPU
  writeTerminal("Initializing CPU...");
  reset6502(); writelineTerminal(" Complete");

  // Initialize RAM
  writeTerminal("Initializing RAM...");
  for (uint16_t i = 0; i < RAM_SIZE; i++) {
    RAM[i] = 0x00;
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
  MX_FSMC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT(); // Initialize LCD
  initApple1(); // Initialize Apple I
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Execute instruction
    exec6502(INSTRUCTION_CHUNK);
    // Debug
    // char debugMsg[100];
    // sprintf(debugMsg, "PIA: 0x%02X 0x%02X 0x%02X 0x%02X", pia.keyboard_register, pia.keyboard_control, pia.display_register, pia.display_control);
    // writelineTerminal(debugMsg);
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
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_DETECT_Pin */
  GPIO_InitStruct.Pin = SDIO_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
