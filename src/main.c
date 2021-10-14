#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usbd_audio.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

// Amount to shave off the bottom and top from the potentiometer to hit "absolute 0% and 100%"
// The value of 20 rougly corresponds to 0.05% for a 12-bit ADC.
#define POTENTIOMETER_SHAVE 20

#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))

USBD_HandleTypeDef USBD_Device;
AUDIO_STATUS_TypeDef audio_status;
__IO uint16_t adcPotVals[2] = { 0, 0 };

void SystemClock_Config(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_USART2_UART_Init();
  printMsg("\r\nmixboi booting up.\r\n");

  bsp_init();

  // ADC subsystems
  MX_DMA_Init();
  MX_ADC1_Init();
  printMsg("\r\nADC initialized\r\n");

  // Init Device Library
  USBD_Init(&USBD_Device, &AUDIO_Desc, 0);
  // Add Supported Class
  USBD_RegisterClass(&USBD_Device, USBD_AUDIO_CLASS);
  // Add Interface callbacks for AUDIO Class
  USBD_AUDIO_RegisterInterface(&USBD_Device, &USBD_AUDIO_fops);
  // Start Device Process
  USBD_Start(&USBD_Device);
  printMsg("\r\nUSBD Started\r\n");

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adcPotVals, 2) != HAL_OK) {
    printMsg("\r\nHAL ADC DMA init failed.");
  }
  printMsg("\r\nHAL ADC DMA started.\r\n");

  uint32_t counter = 0;
  
  while (1) {
    switch (audio_status.frequency) {
      case 44100:
          BSP_LED_Off(LED_RED);
          BSP_LED_Off(LED_GREEN);
          BSP_LED_On(LED_BLUE);
          break;
      case 48000:
          BSP_LED_Off(LED_RED);
          BSP_LED_On(LED_GREEN);
          BSP_LED_Off(LED_BLUE);
          break;
      case 96000:
          BSP_LED_On(LED_RED);
          BSP_LED_Off(LED_GREEN);
          BSP_LED_Off(LED_BLUE);
          break;
      default:
          BSP_LED_Off(LED_RED);
          BSP_LED_Off(LED_GREEN);
          BSP_LED_Off(LED_BLUE);
          break;
    }

    if (counter % 300 == 69) {
      printMsg("chan 1: %d, chan 2: %d (logs: %f, %f)\r\n", adcPotVals[0], adcPotVals[1], logChannelLevels[0], logChannelLevels[1]);
    }

    // if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    //   printMsg("HAL_ADC_Start failed.");
    // }

    // HAL_ADC_Stop(&hadc1);
    // printMsg("ADC1 %d. ADC2 %d.\r\n", adcPotVals[0], adcPotVals[1]);
    HAL_Delay(1);
    counter++;
  }
}

// STM32F411CEU6 versus STM32F401CCU6 "Black Pill" 

#ifdef STM32F411xE

// 96MHz system clock (max 100MHz)
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 271;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

#ifdef STM32F401xC
// 84MHz system clock (max = 84MHz)
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 271;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif



void printMsg(char* format, ...) {
	char sz[100];
	va_list args;
	va_start(args, format);
	vsprintf(sz, format, args);
	HAL_UART_Transmit(&huart2, (uint8_t *)sz, strlen(sz), HAL_MAX_DELAY);
	va_end(args);
	}

void Error_Handler(void){
	uint32_t counter;
  printMsg("ERROR HAPPENED.");
	while(1){
		BSP_OnboardLED_Toggle();
		counter = 0xFFFF;
		while (counter--) ;
	}

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
    printMsg("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */

// Called when the ADC has a successful DMA write.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  for (int i = 0; i < 2; i++) {
    int32_t shaved = (adcPotVals[i] - POTENTIOMETER_SHAVE);
    float floatVal = CLAMP(((float)shaved / (4095.0 - (float)(POTENTIOMETER_SHAVE * 2))), 0.0, 1.0);
    logChannelLevels[i] += (floatVal - logChannelLevels[i]) / 30.0;
  }
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}
