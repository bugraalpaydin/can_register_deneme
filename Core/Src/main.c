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
#include "can_init.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t TxMailbox[3];

int gonder;

uint8_t TxData[8] = {0};
uint8_t RxData[8] = {0};

void readMessage(void){
  RxData[1] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
  RxData[2] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  RxData[3] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  RxData[4] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 24);
  RxData[5] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
  RxData[6] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  RxData[7] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  RxData[8] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 24);

  CAN1->RF0R |= CAN_RF0R_RFOM0;
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
  /* USER CODE BEGIN 2 */
  clock_enable();
  gpio_enable();
  can_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CAN1->sTxMailBox[0].TIR = 0; //reset TIR register, reset value = 0xXXXXXXXX
    CAN1->sTxMailBox[0].TIR |= (1<<2); //extended identifier
    /*
      bits between 3 to 20 (18 bits) are responsible for LSBs of Extended identifier
      bits between 21 to 31 (11 bits) are resbonsible for MSBs of Extended identifier when bit 2 is 1 otherwise these bits are responsible for standart identifier (when bit 2 is 0)
      obviously these bits are for Mailboxes (for transmitting identifiers)
    */
    CAN1->sTxMailBox[0].TIR |= (0x3FFFF<<3); 
    // Setup type information
    CAN1->sTxMailBox[0].TIR &= ~(1<<1); // data frame 
    // Setup data bytes
    CAN1->sTxMailBox[0].TDLR = (((unsigned int)TxData[3] << 24) | 
                                ((unsigned int)TxData[2] << 16) |
                                ((unsigned int)TxData[1] <<  8) | 
    ((unsigned int)TxData[0]));
    CAN1->sTxMailBox[0].TDHR = (((unsigned int)TxData[7] << 24) | 
                                ((unsigned int)TxData[6] << 16) |
                                ((unsigned int)TxData[5] <<  8) |
                                ((unsigned int)TxData[4]));
    //SETUP LENGTH             
    CAN1->sTxMailBox[0].TDTR |= (1<<3);
    CAN1->sTxMailBox[0].TDTR |= (1<<2);
    CAN1->sTxMailBox[0].TDTR |= (1<<1);
    CAN1->sTxMailBox[0].TDTR |= (1<<0);
    CAN1->IER |= (1<<0);                   // Enable TME interrup
    CAN1->sTxMailBox[0].TIR |= (1<<0);     // Transmit message
    HAL_Delay(100);
    readMessage();
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
