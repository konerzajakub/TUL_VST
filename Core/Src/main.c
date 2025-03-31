/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef enum {
  STATE_INITIAL,      // Stav 1 - červená bliká 250ms, modrá 500ms
  STATE_BUTTON_TOGGLE,// Stav 2 - červená toggle, modrá bliká 100ms
  STATE_TRANSITIONAL, // Stav pro detekci dlouhého stisku
  STATE_CUSTOM_PERIOD, // Stav 4 - červená bliká s periodou podle délky stisku, modrá 100ms svítí/400ms nesvítí
  STATE_BLINK_COMMAND  // Nový stav pro příkaz BLIK
} StavAplikace;

typedef enum {
  LED_MODE_OFF,       // LED je trvale vypnutá
  LED_MODE_ON,        // LED je trvale zapnutá
  LED_MODE_BLINK,     // LED bliká s definovanou periodou
  LED_MODE_BLINK_ASYM // LED bliká s asymetrickou periodou (jen modrá ve stavu 4)
} LEDMode;

typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin;
  LEDMode mode;
  uint32_t blinkPeriod;
  uint32_t lastToggleTime;
  bool state;
  uint32_t onTime;     // Doba zapnutí (pro asymetrické blikání)
  uint32_t offTime;    // Doba vypnutí (pro asymetrické blikání)
} Led;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN_FILTER          (50)      // Filtr zákmitů tlačítka v ms
#define LONG_PRESS_DURATION (1000)    // Minimální doba pro dlouhý stisk v ms
#define UART_RX_BUFFER_SIZE (128)     // Velikost bufferu pro příjem UART
#define MAX_COMMAND_LENGTH  (20)      // Maximální délka příkazu
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;

/* USER CODE BEGIN PV */
volatile bool transmissionComplete = false;
char testString[100 + 1]; // jeden znak navic pro ukoncovaci znak
char outputBuffer[200]; // buffer pro vystupni znaky

// UART příjem
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
uint8_t uartRxChar;
char commandBuffer[MAX_COMMAND_LENGTH + 1];
uint8_t commandLength = 0;
volatile bool commandReady = false;

// Stav aplikace
StavAplikace currentState = STATE_INITIAL;
uint32_t buttonPressTime = 0;
uint32_t buttonReleaseTime = 0;
bool buttonPressed = false;
uint32_t lastButtonDebounceTime = 0;
bool lastButtonState = false;
uint32_t customBlinkPeriod = 500; // Výchozí perioda pro vlastní blikání

// Definice LED
Led redLed = {
  .port = LD3_GPIO_Port,
  .pin = LD3_Pin,
  .mode = LED_MODE_BLINK,
  .blinkPeriod = 250,
  .lastToggleTime = 0,
  .state = false
};

Led blueLed = {
  .port = LD2_GPIO_Port,
  .pin = LD2_Pin,
  .mode = LED_MODE_BLINK,
  .blinkPeriod = 500,
  .lastToggleTime = 0,
  .state = false,
  .onTime = 100,  // Pro asymetrické blikání
  .offTime = 400  // Pro asymetrické blikání
};

Led greenLed = {
  .port = GPIOC,
  .pin = GPIO_PIN_7,
  .mode = LED_MODE_OFF,
  .blinkPeriod = 500,
  .lastToggleTime = 0,
  .state = false
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
void generaceStringuStoZnaku(void);
void spustTest(int mode);
void updateLed(Led* led, uint32_t currentTime);
void handleButton(uint32_t currentTime);
void sendUartMessage(const char* message);
void processCommand(void);
void startUartRxInterrupt(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callback function for UART transmission complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == LPUART1) {
    transmissionComplete = true;
  }
}

// Callback function for UART reception complete
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == LPUART1) {
    // Echo received character
    HAL_UART_Transmit_IT(&hlpuart1, &uartRxChar, 1);

    // Process received character
    if (uartRxChar == '\r' || uartRxChar == '\n') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        commandReady = true;
        commandLength = 0;
      }
    } else if (commandLength < MAX_COMMAND_LENGTH) {
      commandBuffer[commandLength++] = uartRxChar;
    }

    // Start listening for next character
    HAL_UART_Receive_IT(&hlpuart1, &uartRxChar, 1);
  }
}

// Funkce pro generaci stringu o 100 znaku
void generaceStringuStoZnaku(void)
{
  const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*()";
  const int charsetSize = strlen(charset);

  for (int i = 0; i < 100; i++) {
    testString[i] = charset[i % charsetSize];
  }
  testString[100] = '\0'; // Null terminate

}

// Spusteni prenosoveho testu v specifickem modu
// mody: 0 = Polling, 1 = Interrupt, 2 = DMA
void spustTest(int mode)
{
  uint32_t startTime, endTime, duration;
  uint32_t blinkCount = 0;
  const char* modeStr;

  // Set mode string
  switch (mode) {
    case 0: modeStr = "Polling"; break;
    case 1: modeStr = "Interrupt"; break;
    case 2: modeStr = "DMA"; break;
    default: modeStr = "jak ses sem dostal?";
  }

  // reset flag
  transmissionComplete = false;

  // pocatek prenosu
  startTime = HAL_GetTick();

  switch (mode) {
    case 0: // Polling
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)testString, 100, 1000);
      transmissionComplete = true; // nastaveno manualne, protoze se nezavola callback
      break;
    case 1: // Interrupt
      HAL_UART_Transmit_IT(&hlpuart1, (uint8_t*)testString, 100);
      break;
    case 2: // DMA
      HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)testString, 100);
      break;
  }

  // blikani led fast a spocteni bliku
  while (!transmissionComplete) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    blinkCount++;
  }

  // zaznam konce
  endTime = HAL_GetTick();
  duration = endTime - startTime;

  // krasne formatovani vysledku:)
  sprintf(outputBuffer,
          "\r\nMode: %s\r\nStart time: %lu ms\r\nDuration: %lu ms\r\nBlink count: %lu\r\n",
          modeStr, startTime, duration, blinkCount);

  // Odeslani vysledku
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)outputBuffer, strlen(outputBuffer), 1000);

  // delay mezi testiky
  HAL_Delay(500);
}

// Aktualizace stavu LED
void updateLed(Led* led, uint32_t currentTime)
{
  if (led->mode == LED_MODE_OFF) {
    // LED je trvale vypnutá
    if (led->state) {
      HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
      led->state = false;
    }
  }
  else if (led->mode == LED_MODE_ON) {
    // LED je trvale zapnutá
    if (!led->state) {
      HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
      led->state = true;
    }
  }
  else if (led->mode == LED_MODE_BLINK) {
    // LED bliká se symetrickou periodou
    if (currentTime - led->lastToggleTime >= led->blinkPeriod / 2) {
      led->state = !led->state;
      HAL_GPIO_WritePin(led->port, led->pin, led->state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      led->lastToggleTime = currentTime;
    }
  }
  else if (led->mode == LED_MODE_BLINK_ASYM) {
    // LED bliká s asymetrickou periodou
    uint32_t interval = led->state ? led->onTime : led->offTime;
    if (currentTime - led->lastToggleTime >= interval) {
      led->state = !led->state;
      HAL_GPIO_WritePin(led->port, led->pin, led->state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      led->lastToggleTime = currentTime;
    }
  }
}

// Zpracování tlačítka
void handleButton(uint32_t currentTime)
{
  // Čtení stavu tlačítka
  bool currentButtonState = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET);

  // Filtr zákmitů tlačítka
  if (currentButtonState != lastButtonState) {
    lastButtonDebounceTime = currentTime;
  }

  // Pokud je stav tlačítka stabilní po dobu filtru
  if (currentTime - lastButtonDebounceTime > BTN_FILTER) {
    // Pokud se stav změnil od posledního stabilního stavu
    if (currentButtonState != buttonPressed) {
      buttonPressed = currentButtonState;

      // Stisk tlačítka
      if (buttonPressed) {
        buttonPressTime = currentTime;

        // Ve stavu 2 přepnout stav červené LED po stisku
        if (currentState == STATE_BUTTON_TOGGLE) {
          redLed.state = !redLed.state;
          HAL_GPIO_WritePin(redLed.port, redLed.pin, redLed.state ? GPIO_PIN_SET : GPIO_PIN_RESET);

          // Informace o změně stavu červené LED
          sprintf(outputBuffer, "\r\nRed LED %s\r\n", redLed.state ? "ON" : "OFF");
          sendUartMessage(outputBuffer);
        }

        // Přechod do pomocného stavu pro detekci dlouhého stisku
        if (currentState == STATE_BUTTON_TOGGLE) {
          currentState = STATE_TRANSITIONAL;

          // Informace o přechodu do přechodného stavu
          sendUartMessage("\r\nVstup do prechodoveho stavu\r\n");
        }
      }
      // Uvolnění tlačítka
      else {
        buttonReleaseTime = currentTime;
        uint32_t pressDuration = buttonReleaseTime - buttonPressTime;

        // Dlouhý stisk - přechod do stavu 4
        if (currentState == STATE_TRANSITIONAL && pressDuration >= LONG_PRESS_DURATION) {
          currentState = STATE_CUSTOM_PERIOD;
          customBlinkPeriod = pressDuration;
          redLed.mode = LED_MODE_BLINK;
          redLed.blinkPeriod = customBlinkPeriod;
          blueLed.mode = LED_MODE_BLINK_ASYM;

          // Informace o přechodu do stavu 4
          sprintf(outputBuffer, "\r\n Stav vlastni delky periody. Delka periody: %lu ms\r\n", customBlinkPeriod);
          sendUartMessage(outputBuffer);
        }
        // Krátký stisk - návrat do stavu 2
        else if (currentState == STATE_TRANSITIONAL) {
          currentState = STATE_BUTTON_TOGGLE;

          // Informace o návratu do stavu 2
          sendUartMessage("\r\nVraceno do stavu klikani tlacitka\r\n");
        }
        // Ve stavu 4 - nastavení nové periody blikání červené LED
        else if (currentState == STATE_CUSTOM_PERIOD) {
          customBlinkPeriod = pressDuration;
          redLed.blinkPeriod = customBlinkPeriod;

          // Informace o nové periodě blikání
          sprintf(outputBuffer, "\r\nNova perioda blikani: %lu ms\r\n", customBlinkPeriod);
          sendUartMessage(outputBuffer);
        }
        // Ve stavu 1 - přechod do stavu 2
        else if (currentState == STATE_INITIAL) {
          currentState = STATE_BUTTON_TOGGLE;
          redLed.mode = LED_MODE_OFF;
          blueLed.mode = LED_MODE_BLINK;
          blueLed.blinkPeriod = 200; // 100ms perioda (50ms on, 50ms off)

          // Informace o přechodu do stavu 2
          sendUartMessage("\r\nVstup do stavu klikani tlacitka\r\n");
        }
      }
    }
  }

  // Uložení posledního stavu tlačítka
  lastButtonState = currentButtonState;
}

// Odeslání zprávy přes UART
void sendUartMessage(const char* message)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)message, strlen(message), 1000);
}

// Nastavení UART přerušení pro příjem
void startUartRxInterrupt(void)
{
  HAL_UART_Receive_IT(&hlpuart1, &uartRxChar, 1);
}

// Zpracování přijatého příkazu
void processCommand(void)
{
  // Konvertovat příkaz na velká písmena pro snadnější porovnání
  char upperCommand[MAX_COMMAND_LENGTH + 1];
  strcpy(upperCommand, commandBuffer);
  for (int i = 0; upperCommand[i]; i++) {
    if (upperCommand[i] >= 'a' && upperCommand[i] <= 'z') {
      upperCommand[i] = upperCommand[i] - 'a' + 'A';
    }
  }

  // Příkaz pro rozsvícení červené LED
  if (strcmp(upperCommand, "RON") == 0) {
    redLed.mode = LED_MODE_ON;
    sprintf(outputBuffer, "\r\nCommand: %s - Cervena LED ON\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
  }
  // Příkaz pro rozsvícení modré LED
  else if (strcmp(upperCommand, "BON") == 0) {
    blueLed.mode = LED_MODE_ON;
    sprintf(outputBuffer, "\r\nCommand: %s - Modra LED ON\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
  }
  // Příkaz pro rozsvícení zelené LED
  else if (strcmp(upperCommand, "GON") == 0) {
    greenLed.mode = LED_MODE_ON;
    sprintf(outputBuffer, "\r\nCommand: %s - Zelena LED ON\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
  }
  // Příkaz pro zhasnutí červené LED
  else if (strcmp(upperCommand, "ROFF") == 0) {
    redLed.mode = LED_MODE_OFF;
    sprintf(outputBuffer, "\r\nCommand: %s - Cervena LED OFF\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
  }
  // Příkaz pro zhasnutí modré LED
  else if (strcmp(upperCommand, "BOFF") == 0) {
    blueLed.mode = LED_MODE_OFF;
    sprintf(outputBuffer, "\r\nCommand: %s - Modra LED OFF\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
  }
  // Příkaz pro zhasnutí zelené LED
  else if (strcmp(upperCommand, "GOFF") == 0) {
    greenLed.mode = LED_MODE_OFF;
    sprintf(outputBuffer, "\r\nCommand: %s - Zelena LED OFF\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
  }
  // Příkaz pro reset do stavu 1
  else if (strcmp(upperCommand, "RESET") == 0) {
    currentState = STATE_INITIAL;
    redLed.mode = LED_MODE_BLINK;
    redLed.blinkPeriod = 250;
    blueLed.mode = LED_MODE_BLINK;
    blueLed.blinkPeriod = 500;
    greenLed.mode = LED_MODE_OFF;

    sprintf(outputBuffer, "\r\nPrikaz: %s - reset to inicialniho stavu\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
  }
  // Příkaz pro blikání s definovanou periodou
  else if (strncmp(upperCommand, "BLIK ", 5) == 0) {
    int period = atoi(&upperCommand[5]);
    if (period > 0) {
      redLed.mode = LED_MODE_BLINK;
      redLed.blinkPeriod = period;
      blueLed.mode = LED_MODE_BLINK;
      blueLed.blinkPeriod = period;
      greenLed.mode = LED_MODE_BLINK;
      greenLed.blinkPeriod = period;
      currentState = STATE_BLINK_COMMAND;  // Nastavit nový stav

      sprintf(outputBuffer, "\r\nCommand: %s - Vsechny LED blikaji s periodou: %d ms\r\n", commandBuffer, period);
      sendUartMessage(outputBuffer);
    } else {
      sprintf(outputBuffer, "\r\nError: Spatna perioda v prikazu '%s'\r\n", commandBuffer);
      sendUartMessage(outputBuffer);
    }
  }
  // Neznámý příkaz
  else {
    sprintf(outputBuffer, "\r\nError: Neznamy prikaz: '%s'\r\n", commandBuffer);
    sendUartMessage(outputBuffer);
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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Nastavení pinů pro zelenou LED (PC7)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Nastartování příjmu UART přes přerušení
  startUartRxInterrupt();

  // Zaslání start zprávy
  sendUartMessage("\r\n--- LED ovladaci system ---\r\n");
  sendUartMessage("Dostupne prikazy:\r\n");
  sendUartMessage("RON, BON, GON - Zapnuti  Cervene / Modre / Zelene LED\r\n");
  sendUartMessage("ROFF, BOFF, GOFF - Vypnuti  Cervene / Modre / Zelene LED\r\n");
  sendUartMessage("RESET - Reset na pocatecni stav\r\n");
  sendUartMessage("BLIK n - Vsechny LED blikaji s periodou \"n\" \r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t currentTime = HAL_GetTick();

    // Zpracování příkazů
    if (commandReady) {
      processCommand();
      commandReady = false;
    }

    // Zpracování tlačítka
    handleButton(currentTime);

    // Aktualizace stavu LED diod

    // Červená LED - chování podle aktuálního stavu
    if (currentState != STATE_BLINK_COMMAND) {
      if (currentState == STATE_INITIAL) {
        // Ve stavu 1 bliká s periodou 250ms
        if (redLed.mode != LED_MODE_ON && redLed.mode != LED_MODE_OFF) {
          redLed.mode = LED_MODE_BLINK;
          redLed.blinkPeriod = 250;
        }
      } else if (currentState == STATE_BUTTON_TOGGLE) {
        // Ve stavu 2 je ovládána tlačítkem
        if (redLed.mode != LED_MODE_ON && redLed.mode != LED_MODE_OFF) {
          redLed.mode = LED_MODE_OFF; // Základní stav, přepínán tlačítkem
        }
      } else if (currentState == STATE_CUSTOM_PERIOD) {
        // Ve stavu 4 bliká s periodou podle délky stisku
        if (redLed.mode != LED_MODE_ON && redLed.mode != LED_MODE_OFF) {
          redLed.mode = LED_MODE_BLINK;
          redLed.blinkPeriod = customBlinkPeriod;
        }
      }
    }

    // Modrá LED - chování podle aktuálního stavu
    if (currentState != STATE_BLINK_COMMAND) {
      if (currentState == STATE_INITIAL) {
        // Ve stavu 1 bliká s periodou 500ms
        if (blueLed.mode != LED_MODE_ON && blueLed.mode != LED_MODE_OFF) {
          blueLed.mode = LED_MODE_BLINK;
          blueLed.blinkPeriod = 500;
        }
      } else if (currentState == STATE_BUTTON_TOGGLE || currentState == STATE_TRANSITIONAL) {
        // Ve stavu 2 a přechodném stavu bliká s periodou 100ms
        if (blueLed.mode != LED_MODE_ON && blueLed.mode != LED_MODE_OFF) {
          blueLed.mode = LED_MODE_BLINK;
          blueLed.blinkPeriod = 200; // 100ms perioda (50ms on, 50ms off)
        }
      } else if (currentState == STATE_CUSTOM_PERIOD) {
        // Ve stavu 4 bliká asynchronně 100ms svítí/400ms nesvítí
        if (blueLed.mode != LED_MODE_ON && blueLed.mode != LED_MODE_OFF) {
          blueLed.mode = LED_MODE_BLINK_ASYM;
          blueLed.onTime = 100;
          blueLed.offTime = 400;
        }
      }
    }

    // Aktualizace stavu jednotlivých LED
    updateLed(&redLed, currentTime);
    updateLed(&blueLed, currentTime);
    updateLed(&greenLed, currentTime);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
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
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
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
	__disable_irq();
	while (1) {
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
