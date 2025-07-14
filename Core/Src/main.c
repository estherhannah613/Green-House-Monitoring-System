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
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f405xx.h"
#include "lcd.h"

#include <stdint.h>
#include <stdio.h>

#include <math.h>

#define ledCount 10
#define THRESHOLD 819  // 20% of 4096

uint8_t fireState = 0;
uint8_t smokeState;
//uint16_t smokeAnalogValue;
char fire, smoke, gas;

volatile uint16_t smokeAnalogValue = 0;
volatile float voltage = 0.0;
volatile int temperature = 0.0;
volatile float t1 = 0.0;

//===============================KEYPAD===============================
const char keymap[4][4] = { { '1', '2', '3', 'A' }, { '4', '5', '6', 'B' }, {
		'7', '8', '9', 'C' }, { '*', '0', '#', 'D' } };

// PC pins used for rows and columns
const uint8_t row_pins[4] = { 0, 1, 3, 8 };  // PC0, PC1, PC3, PC8
const uint8_t col_pins[4] = { 4, 5, 6, 7 };  // PC4, PC5, PC6, PC7

void delayms(uint32_t dly) {
	for (uint32_t i = 0; i < dly; i++)
		for (volatile uint32_t j = 0; j < 16000; j++)
			;
}

void gpio_keypad(void) {
	RCC->AHB1ENR |= (1 << 2);  // Enable Port C clock

	for (int i = 0; i < 4; i++) {
		GPIOC->PUPDR &= ~(3 << (col_pins[i] * 2));   // Clear pull
		GPIOC->PUPDR |= (1 << (col_pins[i] * 2));   // Pull-up
	}
}

char scan_keypad(void) {
	for (int row = 0; row < 4; row++) {
		// Set all rows HIGH
		for (int r = 0; r < 4; r++)
			GPIOC->ODR |= (1 << row_pins[r]);
		// Pull current row LOW
		GPIOC->ODR &= ~(1 << row_pins[row]);

		for (volatile int d = 0; d < 1000; d++); // short delay

		for (int col = 0; col < 4; col++) {
			if ((GPIOC->IDR & (1 << col_pins[col])) == 0) {
				return keymap[row][col];  // Key matched
			}
		}
	}
	return 0;  // No key pressed
}

//===================================================================

//=================== LED BAR GRAPH===================================
const struct {
	GPIO_TypeDef *port;
	uint8_t pin;
} ledPins[ledCount] = { { GPIOB, 4 }, { GPIOB, 5 }, { GPIOB, 6 }, { GPIOB, 7 },
		{ GPIOB, 8 }, { GPIOB, 9 }, { GPIOB, 10 }, { GPIOB, 11 }, { GPIOC, 13 },
		{ GPIOC, 12 } };

void gpio_graph(void) {
	RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2); // Enable GPIOA, GPIOB, GPIOC

	for (int i = 0; i < ledCount; i++) {
		GPIO_TypeDef *port = ledPins[i].port;
		uint8_t pin = ledPins[i].pin;

		port->MODER &= ~(3 << (pin * 2));
		port->MODER |= (1 << (pin * 2));
		port->OTYPER &= ~(1 << pin);	//Sets the pin to push-pull
		port->PUPDR &= ~(3 << (pin * 2));	//Disables pull-up and pull-down resistors.
	}
}

void adc_graph(void) {
	RCC->AHB1ENR |= (1 << 2);             // Enable GPIOC clock
	GPIOC->MODER |= (3 << (2 * 2));       // PC2 analog mode

	RCC->APB2ENR |= (1 << 8);             // Enable ADC1 clock
	ADC1->SQR3 = 12;                      // Channel 12 (PC2)
	ADC1->CR1 = (1 << 8);                 // Scan mode
	ADC1->CR2 |= (1 << 1) | (1 << 0);     // Continuous mode + ADC ON
}

uint16_t adc_read_graph(void) {
	ADC1->SQR3 = 12;

	ADC1->CR2 |= (1 << 30);               // Start conversion
	while (!(ADC1->SR & (1 << 1)))
		;       // Wait for EOC
	return ADC1->DR;
}
//===========================================================================

void delayMs(uint32_t ms) {
	for (uint32_t i = 0; i < ms * 16000; i++)
		;
}

//===============LDR - THERMISTOR - LEDS===================================
void gpio_init(void) {
	RCC->AHB1ENR |= (1 << 0);  // Enable GPIOA clock

	// PA5 (Smoke A0) - analog input
	GPIOA->MODER |= (3 << (5 * 2));  // Analog mode
	GPIOA->PUPDR &= ~(3 << (5 * 2));  // No pull


	// PA7 as buzzer output
	GPIOA->MODER &= ~(3 << (7 * 2));  // Clear MODER7
	GPIOA->MODER |= (1 << (7 * 2));  // Set MODER7 = 01 (output)
	GPIOA->OTYPER &= ~(1 << 7);       // Push-pull
	GPIOA->OSPEEDR |= (3 << (7 * 2)); // High speed
	GPIOA->PUPDR &= ~(3 << (7 * 2));  // No pull-up/down

}


void adc(void) {
	RCC->APB2ENR |= (1 << 9);     // Enable ADC2 clock (bit 9)
	ADC2->CR2 = 0;
	ADC2->SMPR2 |= (7 << 15);     // Max sample time (channel 5)
	ADC2->CR2 |= (1 << 0);        // Turn on ADC2
}

uint16_t read_adc(void) {
	ADC2->SQR3 = 5;               // Channel 5 = PA5

	ADC2->CR2 |= (1 << 30);       // Start conversion
	while (!(ADC2->SR & (1 << 1)))
		; // Wait for EOC
	return ADC2->DR;
}

void toggle_led(uint8_t pin) {
	GPIOA->ODR ^= (1 << pin);   // Toggle ON/OFF
	delayMs(100);               // Short delay
}

void display_temp(void) {

	smokeAnalogValue = read_adc();
	voltage = (smokeAnalogValue / 4095.0) * 3.3;
	t1 = -50.0 * voltage;
	temperature = t1 + 107.5;
	char temp_str[17];
	sprintf(temp_str, "Temp: %d C", temperature + 45);
	lprint(0xC0, temp_str);
}

//==================================================================================

//====================================FAN AND PUMP==================================

void fan_pump(void) {
	RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB clock

	// Set PB0 and PB1 as output - FAN
	GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2))); // Clear MODER0 & MODER1
	GPIOB->MODER |= ((1 << (0 * 2)) | (1 << (1 * 2)));   // Set as output

	GPIOB->OTYPER &= ~((1 << 0) | (1 << 1));             // Push-pull
	GPIOB->PUPDR &= ~((3 << (0 * 2)) | (3 << (1 * 2)));   // No pull-up/down

	// Set PB2 and PB3 as output -PUMP
	GPIOB->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2))); // Clear MODER2 & MODER3
	GPIOB->MODER |= ((1 << (2 * 2)) | (1 << (3 * 2)));   // Set as output

	GPIOB->OTYPER &= ~((1 << 2) | (1 << 3));             // Push-pull
	GPIOB->PUPDR &= ~((3 << (2 * 2)) | (3 << (3 * 2)));   // No pull-up/down
}

void fan_on(void) {
	GPIOB->ODR |= (1 << 0);   // AIN1 = High
	GPIOB->ODR &= ~(1 << 1);  // AIN2 = Low → Forward direction
}

void fan_off(void) {
	GPIOB->ODR &= ~((1 << 0) | (1 << 1));  // AIN1 = 0, AIN2 = 0 → Stop
}

void pump_on(void) {
	GPIOB->ODR |= (1 << 2);   // BIN1 = High
	GPIOB->ODR &= ~(1 << 3);  // BIN2 = Low → Forward direction
}

void pump_off(void) {
	GPIOB->ODR &= ~((1 << 2) | (1 << 3));  // BIN1 = 0, BIN2 = 0 → Stop
}
//=================================================================================

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void TaskCHECK(void *pv) {
	char buffer[15];
	uint16_t GassensorReading;
	while (1) {
		fireState = 0;
		smokeState = 1;
		// FIRE SENSOR - PA6
		fireState = (GPIOA->IDR & (1 << 6));
		// SMOKE SENSOR - PA4 D0
		smokeState = (GPIOA->IDR & (1 << 4));

		//GAS SENSOR - POT
		GassensorReading = adc_read_graph();

		if (fireState && !smokeState) {
			LcdFxn(0, 0x01);
			lprint(0x80, "FIRE and SMOKE!!");
			//turn on buzzer
			GPIOA->ODR |= (1 << 7);
			//turn on pump
			pump_on();
			//turn on fan
			fan_on();
			//turn off graph
			for (int i = 0; i < ledCount; i++)
				ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);

			display_temp();
			fire = 'y';
			smoke = 'y';
			gas = 'n';
			//leds
			GPIOA->ODR &= ~(1 << 8);
			for (int i = 0; i < 15; i++) {
				toggle_led(2);
				toggle_led(3);
			}

		}

		else if (fireState) {
			LcdFxn(0, 0x01);
			lprint(0x80, "FIRE detected!!!");
			//turn on buzzer
			GPIOA->ODR |= (1 << 7);
			//turn on pump
			pump_on();
			//turn off graph
			for (int i = 0; i < ledCount; i++)
				ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);
			//turn off fan
			fan_off();
			display_temp();
			fire = 'y';
			smoke = 'n';
			gas = 'n';
			//leds
			GPIOA->ODR &= ~((1 << 3) | (1 << 8));
			for (int i = 0; i < 15; i++) {
				toggle_led(2);
			}

		}

		else if (!smokeState) {
			LcdFxn(0, 0x01);
			lprint(0x80, "SMOKE detected!!!");
			//turn on buzzer
			GPIOA->ODR |= (1 << 7);
			//turn on fan
			fan_on();
			//turn off pump
			pump_off();
			//turn off graph
			for (int i = 0; i < ledCount; i++)
				ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);
			display_temp();
			fire = 'n';
			smoke = 'y';
			gas = 'n';
			//leds
			GPIOA->ODR &= ~((1 << 2) | (1 << 8));
			for (int i = 0; i < 15; i++) {
				toggle_led(3);
			}

		} else if (GassensorReading > THRESHOLD) {
			uint8_t ledLevel = (GassensorReading * ledCount) / 4096;
			uint8_t gasPercent = (GassensorReading * 100) / 4096;
			if (gasPercent == 0)
				gasPercent = 1;

			for (int i = 0; i < ledCount; i++) {
				GPIO_TypeDef *port = ledPins[i].port;
				uint8_t pin = ledPins[i].pin;

				if (i < ledLevel)
					port->ODR |= (1 << pin);
				else
					port->ODR &= ~(1 << pin);
			}
			//turn off leds
			GPIOA->ODR &= ~((1 << 2) | (1 << 8) | (1 << 3));
			//turn off fan
			fan_off();
			//turn off pump
			pump_off();
			//turn off buzzer
			GPIOA->ODR &= ~(1 << 7);

			LcdFxn(0, 0x01);
			//lprint(0x80, "GAS DETECTED");
			sprintf(buffer, " Gas level: %3d %%", gasPercent);
			lprint(0x80, buffer);
			display_temp();
			fire = 'n';
			smoke = 'n';
			gas = 'y';
		}

		else {
			LcdFxn(0, 0x01);
			GPIOA->ODR &= ~(1 << 7);
			//leds
			GPIOA->ODR |= (1 << 8);
			GPIOA->ODR &= ~((1 << 3) | (1 << 2));
			//turn off graph
			for (int i = 0; i < ledCount; i++)
				ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);
			//turn off fan
			fan_off();
			//turn off pump
			pump_off();
			lprint(0x80, "SAFE");
			display_temp();
			fire = 'n';
			smoke = 'n';
			gas = 'n';
		}

		delayMs(300);
	}
}
void TaskPassword(void *pv) {

	char passkey[5] = { 0 };
	const char password[] = "127C";

	LcdFxn(0, 0x01); // Clear screen
	lprint(0x80, "Welcome");

	while (1) {
		char key = scan_keypad();
		if (key == '*') {
			LcdFxn(0, 0x01);
			lprint(0x80, "Enter Passkey:");
			memset(passkey, 0, sizeof(passkey));
			int i = 0;

			while (1) {
				key = scan_keypad();
				if (key) {
					if (key == '#') {
						passkey[i] = '\0';
						LcdFxn(0, 0x01);

						if (strcmp(passkey, password) == 0) {
							lprint(0x80, "Please come in");
						} else {
							lprint(0x80, "Wrong");
						}

						vTaskDelay(pdMS_TO_TICKS(1000));
						LcdFxn(0, 0x01);
						lprint(0x80, "Welcome");

						if (strcmp(passkey, password) == 0) {
							goto access_granted;
						} else {
							break;
						}
					}

					if (i < 4) {
						passkey[i++] = key;
						lprint(0xC0 + i - 1, "*");
					}

					while (scan_keypad())
						; // Wait for release
					vTaskDelay(pdMS_TO_TICKS(100)); // Debounce
				}

				vTaskDelay(pdMS_TO_TICKS(10));
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50)); // Polling delay
	}

	access_granted: lprint(0x80, "Welcome");

	xTaskCreate(TaskCHECK, "check", 128, NULL, 2, NULL);
}

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

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	gpio_init();
	adc();
	LcdInit();
	DelayLcd();

	char buffer_graph[20];

	adc_graph();
	gpio_graph();

	fan_pump();
	pump_off();
	fan_off();

	gpio_keypad();
	/* USER CODE BEGIN 2 */

	xTaskCreate(TaskPassword, "password", 128, NULL, 1, NULL);

	vTaskStartScheduler();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM3) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
