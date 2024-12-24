#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "task.h"
#include "freertos.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_gpio.h"

//Pins definition
#define GREEN_LED_PIN GPIO_PIN_12
#define ORANGE_LED_PIN GPIO_PIN_13
#define RED_LED_PIN GPIO_PIN_14
#define BLUE_LED_PIN GPIO_PIN_15
#define BUTTON_PIN GPIO_PIN_0

#define TASK_STACK_SIZE 128

//tasks handles
TaskHandle_t greenTaskHandle;
TaskHandle_t redTaskHandle;
TaskHandle_t blueTaskHandle;
TaskHandle_t urgentGreenHandle;

//tasks durations
const TickType_t GreenDuration = pdMS_TO_TICKS(5000);
const TickType_t RedDuration = pdMS_TO_TICKS(5000);
const TickType_t OrangeDuration = pdMS_TO_TICKS(2000);

//Mutex for emergency flag protection
SemaphoreHandle_t xEmergencyFlagMutex;

//emergency flag
volatile uint8_t emergencyFlag = 0;

void SystemClock_Config(void);
void MX_GPIO_Init(void);

//LED control helper functions
static void LED_Set(uint16_t LED_Pin, GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOD, LED_Pin, state);
}

static void LED_Toggle(uint16_t LED_Pin) {
    HAL_GPIO_TogglePin(GPIOD, LED_Pin);
}

//turn off all LEDs
static void LED_AllOff(void) {
    LED_Set(GREEN_LED_PIN, GPIO_PIN_RESET);
    LED_Set(RED_LED_PIN, GPIO_PIN_RESET);
    LED_Set(ORANGE_LED_PIN, GPIO_PIN_RESET);
    LED_Set(BLUE_LED_PIN, GPIO_PIN_RESET);
}

//green LED task
void GreenTask(void *pvParameters) {
    for(;;) {
        vTaskSuspend(NULL);

        LED_Set(GREEN_LED_PIN, GPIO_PIN_SET);
        vTaskDelay(GreenDuration);
        LED_Set(GREEN_LED_PIN, GPIO_PIN_RESET);

        LED_Set(ORANGE_LED_PIN, GPIO_PIN_SET);
        vTaskDelay(OrangeDuration);
        LED_Set(ORANGE_LED_PIN, GPIO_PIN_RESET);

        vTaskResume(redTaskHandle);
    }
}

//red LED task
void RedTask(void *pvParameters) {
    for(;;) {
        if(xSemaphoreTake(xEmergencyFlagMutex, portMAX_DELAY) == pdTRUE) {
            if (emergencyFlag == 0) {
                xSemaphoreGive(xEmergencyFlagMutex);

                LED_Set(RED_LED_PIN, GPIO_PIN_SET);
                vTaskDelay(RedDuration);
                LED_Set(RED_LED_PIN, GPIO_PIN_RESET);

                LED_Set(ORANGE_LED_PIN, GPIO_PIN_SET);
                vTaskDelay(OrangeDuration);
                LED_Set(ORANGE_LED_PIN, GPIO_PIN_RESET);

                vTaskResume(greenTaskHandle);
                vTaskSuspend(NULL);
            } else {
                xSemaphoreGive(xEmergencyFlagMutex);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}

//interrupt handler
void EXTI0_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    HAL_GPIO_EXTI_IRQHandler(B1_Pin);

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    if (HAL_GPIO_ReadPin(GPIOD, GREEN_LED_PIN) == GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(GPIOD, ORANGE_LED_PIN) == GPIO_PIN_SET ||
        emergencyFlag == 1) {
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    LED_AllOff();

    if(emergencyFlag == 0) {
        emergencyFlag = 1;
    }

    //start the emergency mode
    BaseType_t check1 = xTaskResumeFromISR(urgentGreenHandle);
    BaseType_t check2 = xTaskResumeFromISR(blueTaskHandle);

    if (check1 == pdTRUE || check2 == pdTRUE) {
        xHigherPriorityTaskWoken = pdTRUE;
    }

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//blue LED task
void BlueTask(void *pvParameters) {
    for(;;) {
        vTaskSuspend(NULL);
        while (emergencyFlag == 1) {
            LED_Toggle(BLUE_LED_PIN);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        LED_Set(BLUE_LED_PIN, GPIO_PIN_RESET);
    }
}

//urgent green LED task
void urgentGreen(void *p) {
    for(;;) {
        vTaskSuspend(NULL);

        vTaskDelete(redTaskHandle);
        vTaskDelete(greenTaskHandle);

        LED_Set(GREEN_LED_PIN, GPIO_PIN_SET);
        vTaskDelay(GreenDuration);
        LED_Set(GREEN_LED_PIN, GPIO_PIN_RESET);

        if(xSemaphoreTake(xEmergencyFlagMutex, portMAX_DELAY) == pdTRUE) {
            emergencyFlag = 0;
            xSemaphoreGive(xEmergencyFlagMutex);
        }

        //recreate Red and Green tasks with higher priorities & resume them
        xTaskCreate(RedTask, "RedTask", TASK_STACK_SIZE, NULL, 3, &redTaskHandle);
        vTaskResume(redTaskHandle);

        xTaskCreate(GreenTask, "GreenTask", TASK_STACK_SIZE, NULL, 3, &greenTaskHandle);
        vTaskResume(greenTaskHandle);
    }
}



//main
int main(void) {
    BaseType_t xReturned;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    //create mutex
    xEmergencyFlagMutex = xSemaphoreCreateMutex();
    if (xEmergencyFlagMutex == NULL) {
        while(1);
    }

    //create tasks with error checking
    xReturned = xTaskCreate(GreenTask, "GreenTask", TASK_STACK_SIZE, NULL, 1, &greenTaskHandle);
    if (xReturned != pdPASS) {
        while(1);
    }

    xReturned = xTaskCreate(RedTask, "RedTask", TASK_STACK_SIZE, NULL, 2, &redTaskHandle);
    if (xReturned != pdPASS) {
        while(1);
    }

    xReturned = xTaskCreate(BlueTask, "BlueTask", TASK_STACK_SIZE, NULL, 3, &blueTaskHandle);
    if (xReturned != pdPASS) {
        while(1);
    }

    xReturned = xTaskCreate(urgentGreen, "urgentGreen", TASK_STACK_SIZE, NULL, 3, &urgentGreenHandle);
    if (xReturned != pdPASS) {
        while(1);
    }

    vTaskStartScheduler();

    //should never reach here
    while (1) {
    }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
