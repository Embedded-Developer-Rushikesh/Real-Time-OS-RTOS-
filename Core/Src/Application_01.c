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
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include"string.h"
#include "queue.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//command structure
typedef struct APP_CMD
{
	uint8_t COMMAND_NUM;
	uint8_t COMMAND_ARGS[10];
}APP_CMD_t;
 uint8_t command_buffer[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t uart_data[10]={0} ;

//This is the menu
char menu[]={"\
\r\nLED_ON             ----> 1 \
\r\nLED_OFF            ----> 2 \
\r\nLED_TOGGLE         ----> 3 \
\r\nLED_TOGGLE_OFF     ----> 4 \
\r\nLED_READ_STATUS    ----> 5 \
\r\nEXIT_APP           ----> 0 \
\r\nType your option here : "};
TaskHandle_t user_handle;
TaskHandle_t uart_handle;
TaskHandle_t menu_handle;
TaskHandle_t cmd_process_handle;
//Queue handle
QueueHandle_t command_queue=NULL;
QueueHandle_t uart_write_queue = NULL;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LED_ON_COMMAND 			1
#define LED_OFF_COMMAND 		2
#define LED_TOGGLE_COMMAND 		3
#define LED_TOGGLE_STOP_COMMAND 4
#define  LED_MENU_STATUS_COMMAND 5
static void user_handler(void* parameters);
static void uart_handler(void* parameters);
static void menu_display(void* parameters);
static void cmd_processing_handler(void* parameters);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	BaseType_t status;
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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	//lets create command queue
  //lets create command queue
  	command_queue = xQueueCreate(10,sizeof(APP_CMD_t*));

	//lets create the write queue
	uart_write_queue = xQueueCreate(10,sizeof(char*));
	if(uart_write_queue!=NULL && command_queue!=NULL){
		HAL_UART_Transmit(&huart2, &menu[0], strlen(menu), 1000);
      status= xTaskCreate(menu_display,NULL,500,NULL,1,&menu_handle);
      configASSERT(status == pdPASS);
  	  status = xTaskCreate(user_handler, NULL, 500, NULL, 2, &user_handle);
  	  configASSERT(status == pdPASS);
  	  status = xTaskCreate(uart_handler, NULL, 500, NULL, 2, &uart_handle);
  	  configASSERT(status == pdPASS);
  	//lets create task-3
  	status=	xTaskCreate(cmd_processing_handler,"TASK3-CMD-PROCESS",500,NULL,2,&cmd_process_handle);
  	configASSERT(status == pdPASS);
  	  vTaskStartScheduler();
	}

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_Led_Pin */
  GPIO_InitStruct.Pin = Green_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_Led_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void menu_display(void* parameters)
{

char *pData = menu;
BaseType_t status;

	while(1)
	{

//		//lets wait here until someone notifies.
//		if(xQueueReceive(uart_write_queue,(void*)&pData,portMAX_DELAY)==pdTRUE);
//
//		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(10));
//		if(status==pdTRUE){
		HAL_UART_Transmit(&huart2, &pData[0], strlen(pData), 1000);
	//	}

	}
	//vTaskDelay( 10 / portTICK_PERIOD_MS);
}
uint8_t getCommandCode(uint8_t *buffer)
{

	return buffer[0]-48;
}
static void user_handler(void* parameters){
	uint8_t command_code=0;

	APP_CMD_t *new_cmd;


	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		//1. send command to queue
		new_cmd = (APP_CMD_t*) pvPortMalloc(sizeof(APP_CMD_t));

		taskENTER_CRITICAL();
		command_code = getCommandCode(command_buffer);
		new_cmd->COMMAND_NUM = command_code;
		taskEXIT_CRITICAL();

		//send the command to the command queue
		if(xQueueSend(command_queue,&new_cmd,portMAX_DELAY)==pdTRUE);

	}
	vTaskDelay( 10 / portTICK_PERIOD_MS);
}
static void uart_handler(void* parameters)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	while(1){
	HAL_UART_Receive(&huart2, (char*)&command_buffer[0], sizeof(command_buffer), 1000);
	if(command_buffer[1] == '\n')
	{
		//lets notify the command handling task
		xTaskNotifyFromISR(user_handle,0,eNoAction,&xHigherPriorityTaskWoken);

		xTaskNotifyFromISR(cmd_process_handle,0,eNoAction,&xHigherPriorityTaskWoken);
	}

	}
	vTaskDelay( 10 / portTICK_PERIOD_MS);
}
void make_led_on()
{
	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin,GPIO_PIN_SET);
}
void make_led_off()
{
	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin,GPIO_PIN_RESET);
}
void led_toggle_start()
{
	HAL_GPIO_TogglePin(Green_Led_GPIO_Port, Green_Led_Pin);
}
void led_toggle_stop()
{
	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin,GPIO_PIN_RESET);
}
void print_error_message()
{
	char *pData="Invalid Code \n";
	HAL_UART_Transmit(&huart2, &pData[0], strlen(pData), 1000);
}
void read_led_status()
{
//	APP_CMD_t *new_cmd;
	char* data='\n';
	char *pData ="Led Status is ";
//	sprintf(task_msg , "\r\nLED status is : %d\r\n", GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5));
	HAL_UART_Transmit(&huart2, &pData[0], strlen(pData), 10);
	HAL_UART_Transmit(&huart2, (char*)&command_buffer[0], sizeof(command_buffer), 10);
	HAL_UART_Transmit(&huart2, &data[0], strlen(data), 10);
}
static void cmd_processing_handler(void* parameters)
{
	APP_CMD_t *new_cmd;
	BaseType_t status;

	while(1)
	{
		status=xQueueReceive(command_queue,(void*)&new_cmd,portMAX_DELAY);
if(status==pdTRUE){
		if(new_cmd->COMMAND_NUM == LED_ON_COMMAND)
		{
			make_led_on();
			read_led_status();
		}
		else if(new_cmd->COMMAND_NUM == LED_OFF_COMMAND)
		{
			make_led_off();
			read_led_status();
		}
		else if(new_cmd->COMMAND_NUM == LED_TOGGLE_COMMAND)
		{
			led_toggle_start();
			read_led_status();
		}
		else if(new_cmd->COMMAND_NUM == LED_TOGGLE_STOP_COMMAND)
		{
			led_toggle_stop();
			read_led_status();
			} else
		   {
			print_error_message();
		  }
}

		//lets free the allocated memory for the new command
		vPortFree(new_cmd);

	}
	vTaskDelay( 10 / portTICK_PERIOD_MS);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
