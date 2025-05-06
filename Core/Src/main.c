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
#include "stdio.h" //used for printing to lcd
#include "string.h" //for working with strings
#include "i2c_lcd.h" //my lcd control library
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2C_LCD_HandleTypeDef lcd1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=====STATE DEFINITIONS=====
typedef enum {
	STATE_A,
	STATE_B,
	STATE_C
} SystemState_t;

SystemState_t currentState = STATE_A;

//=====PIN DEFINITIONS===== (for clarity)
#define ADC_PORT GPIOA
#define ADC_PIN GPIO_PIN_0				//PA0
#define EXT_BUTTON_1_PORT GPIOA
#define EXT_BUTTON_1_PIN GPIO_PIN_1 	//PA1
#define ONB_BUTTON_PORT GPIOC
#define ONB_BUTTON_PIN GPIO_PIN_13		//PC13
#define LED_1_PORT GPIOA				//on-board LED
#define LED_1_PIN GPIO_PIN_5			//PA5
#define LED_2_PORT GPIOB
#define LED_2_PIN GPIO_PIN_0			//PB0
#define LED_3_PORT GPIOB
#define LED_3_PIN GPIO_PIN_1			//PB1
#define SERVO_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO_TIM_HANDLE htim1
#define SERVO_PORT GPIOA
#define SERVO_PIN GPIO_PIN_8			//PA8
#define UART_TX_PORT GPIOA
#define UART_TX_PIN GPIO_PIN_2			//PA2
#define UART_RX_PORT GPIOA
#define UART_RX_PIN GPIO_PIN_3			//PA3
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN GPIO_PIN_7			//PB7
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_6			//PB6

//=====UART DEFINITIONS=====
char uart_message[100]; 				//storage of messages
uint8_t transmit_uart = 1; 				//flag for UART transmission
uint32_t last_uart_transmit_time = 0; 	//last transmitted UART message
#define UART_TRANSMIT_INTERVAL 500

//=====ADC DEFINITIONS=====
uint32_t adc_value = 0; 				//storage of adc value

//=====LED DEFINITIONS=====
uint8_t led_select = 0; 				//switching between external LED 2 or 3
uint32_t led1_blink_period = 500; 		//flashing period for on-board led

//=====DEBOUNCE DEFINITIONS=====
uint32_t last_button1_press_time = 0;	//timer for button 1 presses
uint32_t last_button2_press_time = 0;	//timer for button 2 presses
uint8_t button1_pressed = 0;
uint8_t button2_pressed = 0;
#define DEBOUNCE_DELAY 100				//delay for debouncing

//=====STUDENT ID=====
char student_id[9] = "24784821";		//defined student id

//=====BUTTON DEBOUNCE FUNCTION=====
uint8_t debounceButton(GPIO_TypeDef *port, uint16_t pin, uint32_t *last_press_time){ //will take port and pin information when i call the function in the loop
	uint8_t current_state = HAL_GPIO_ReadPin(port, pin); //store button state
	uint32_t current_time = HAL_GetTick();	//store current time in ms
d
	if (current_state == GPIO_PIN_SET){								//if the button is pressed
		if (current_time - *last_press_time >= DEBOUNCE_DELAY) {	//and the debounce timer has passed
			HAL_Delay(DEBOUNCE_DELAY);								//confirmation delay (may not need)
			if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET){		//check if still pressed
				*last_press_time = current_time;					//update last pressed time
				return 1; 											//valid debounced press
			}
		}
	}
	return 0; 	//button not pressed
}

//=====MAP FUNCTION=====		// Calculates input range, calculates output range, then uses linear interpolation formula - taken from arduino source code
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//=====LCD PRINT INT FUNCTION=====
void lcd_print_int(int num){
	char holdme[16]; //storing string version of integer
	sprintf(holdme, "%d", num); //conversion of integer to string
	lcd_puts(&lcd1, holdme);			//send to LCD
}

//=====SET SERVO PULSE FUNCTION=====
void setServoPulse(uint16_t pulse) {
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	 HAL_TIM_PWM_ConfigChannel(&SERVO_TIM_HANDLE, &sConfigOC, SERVO_TIM_CHANNEL);
	 HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL);
}


//=====STATE A ACTIONS=====
void stateA_actions(void) {

	//LCD CONTROL
	lcd_clear(&lcd1);							//clear lcd screen
	lcd_gotoxy(&lcd1, 0, 0);					//position cursor in top left
	lcd_puts(&lcd1, "SID:");					//print 'SID'
	lcd_puts(&lcd1, student_id);				//print my student ID number
	lcd_gotoxy(&lcd1, 0, 1);					//position cursor to second line
	lcd_puts(&lcd1, "MECHATRONICS 1");			//print string to second line

	//UART CONTROL
	static uint32_t last_uart_transmit = 0; //timer starts from 0
	if (transmit_uart && (HAL_GetTick() - last_uart_transmit >= UART_TRANSMIT_INTERVAL)){ //if there is a transmission flag and the time interval for transmissions has passed, then
		sprintf(uart_message, "Autumn2025 MX1 SID: %s, ADC Reading: %lu", student_id, adc_value); //print message to monitor, inserting student_id and adc_value into the string
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_message, strlen(uart_message), HAL_MAX_DELAY); //using STM32HAL functions to transmit UART. Converts uart_message into an integer that can be transmitted over UART.
		last_uart_transmit = HAL_GetTick(); //update last transmit time
	}

	//UART KEYBOARD TOGGLE
	uint8_t received_char;												//variable to store character received
	if (HAL_UART_Receive(&huart2, &received_char, 1, 0) == HAL_OK) {	//if statement checks if the UART successfully received a character
		if (received_char == 's') {										//if it is successful, and the character matches, then toggle the UART transmissions
			transmit_uart = !transmit_uart;
		}
	}

}

//=====STATE B ACTIONS=====
void stateB_actions(void) {
	//LCD CONTROL FOR ADC VALUES
	lcd_clear(&lcd1);					//clear lcd
	lcd_gotoxy(&lcd1, 0, 0);			//position cursor top left
	lcd_puts(&lcd1, "ADC:");				//print ADC
	lcd_print_int(adc_value);		//print the ADC value as an integer
	lcd_puts(&lcd1, " STATE B");
	lcd_gotoxy(&lcd1, 0, 1);			//next line
	lcd_puts(&lcd1, "MECHATRONICS 1");


	//UART CONTROL
	transmit_uart = 0; 				//disable UART communication


	//ONBOARD LED CONTROL WITH POT
	static uint32_t last_led1_blink = 0; //period since last blink
	uint32_t led1_blink_period;			 //storage for variable blink timing
	static uint8_t led1_state = 0;				 //state of LED (off or on)

	led1_blink_period = map(adc_value, 0, 4095, 200, 1000); //linear interpolation of ADC values to led blink period, 5Hz -> 1Hz

	if (HAL_GetTick() - last_led1_blink >= led1_blink_period) { //checks if enough time has passed between blinks
		led1_state = !led1_state; 								//toggle LED
		HAL_GPIO_WritePin(LED_1_PORT, LED_1_PIN, led1_state ? GPIO_PIN_SET : GPIO_PIN_RESET); //Control of actual LED output. Takes port and pin info from definitions above
		last_led1_blink = HAL_GetTick(); //update blink time
	}

	//EXTERNAL LED CONTROL
	static uint32_t last_led2_3_blink = 0; //variable to control timing for both LEDs
	static uint8_t led2_state = 0;				   //state of LED2 (on or off)
	static uint8_t led3_state = 0;				   //state of LED3 (on or off)

	if (button1_pressed) { 			//checks for valid debounced button press
		led_select = !led_select;	//toggle between LEDs when button 1 pressed
	}

	if (HAL_GetTick() - last_led2_3_blink >= 500) { //1Hz blinking 500on 500off
        last_led2_3_blink = HAL_GetTick(); 			//update blink time
        if (led_select == 0) {						//checks which LED is selected and toggles state
        	led2_state = !led2_state;
        	HAL_GPIO_WritePin(LED_2_PORT, LED_2_PIN, led2_state ? GPIO_PIN_SET : GPIO_PIN_RESET); //actual output of LED2, will toggle either off or on
        } else {
        	led3_state = !led3_state;
        	HAL_GPIO_WritePin(LED_3_PORT, LED_3_PIN, led3_state ? GPIO_PIN_SET : GPIO_PIN_RESET); //same thing for LED3
        }
	}

	//SERVO CONTROL FROM POT
	uint16_t servo_pulse_width = map(adc_value, 0, 4095, 1000, 2000); //map adc values to servo PWM
	setServoPulse(servo_pulse_width); //write to the servo


}
//=====STATE C ACTIONS=====
void stateC_actions(void) {

	//check everything disabled
	transmit_uart = 0;


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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start(&hadc1);					//begin adc conversion

  lcd1.hi2c = &hi2c1;
  lcd1.address = 0x4E; //lcd address

  lcd_init(&lcd1); //initialise LCD

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //=====Debounce Control=====
	  uint8_t button1_pressed = debounceButton(EXT_BUTTON_1_PORT, EXT_BUTTON_1_PIN, &last_button1_press_time);	//Calls debounce function and sends button and port info
	  uint8_t button2_pressed = debounceButton(ONB_BUTTON_PORT, ONB_BUTTON_PIN, &last_button2_press_time);

	  //=====ADC CONTROL=====
	  if (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_REG_EOC) {		//checks if conversion is complete
		  adc_value = HAL_ADC_GetValue(&hadc1);						//stores the adc value
		  HAL_ADC_Start(&hadc1);									//restarts conversion for continuous readings
	  }

	  //=====UPDATING STATE=====
	  if (button2_pressed) {										//Button 2 State change control
		  if (currentState == STATE_A || currentState == STATE_C){	//if state is A or C change to B
			  currentState = STATE_B;
		  }	else if (currentState == STATE_B) {						//if state is B change to A
			  currentState = STATE_A;
		  }
	  }

	  if (currentState == STATE_A && button1_pressed) {				//Button 1 State change control
		  currentState = STATE_C;									//if state is A change to C
	  }

	  //=====FUNCTION CONTROL DEPENDING ON STATE=====
	  switch (currentState) {
	  	  case STATE_A:
	  		  stateA_actions();
	  		  break;
	  	  case STATE_B:
	  		  stateB_actions();				//EACH STATE A-C has different functions called depending on state
	  		  break;
	  	  case STATE_C:
	  		  stateC_actions();
	  		  break;

	  }

	  HAL_Delay(10); //stability

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_ONB_GPIO_Port, LED_1_ONB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ONB_BUTTON_Pin */
  GPIO_InitStruct.Pin = ONB_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ONB_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_BUTTON_1_Pin */
  GPIO_InitStruct.Pin = EXT_BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(EXT_BUTTON_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_ONB_Pin */
  GPIO_InitStruct.Pin = LED_1_ONB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_ONB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
