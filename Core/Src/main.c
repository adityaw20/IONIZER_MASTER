/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/**
 * @file    main.c
 * @brief   Ionizer System — Master MCU Main Application
 * @target  STM32F4xx (HAL)
 * @author  Ionizer Firmware Team
 * @version 1.0.0
 *
 * Execution Flow (main loop):
 *   Step 1  — Read GSM Timestamp
 *   Step 2  — Read Inlet WFS + Poll Slave 1 HR[0] + Poll Slave 2 HR[0,1]
 *   Step 3  — TDS Monitor (80–100 ppm)  → Ionizer OFF on fault
 *   Step 4  — Booster Tank Level        → Ionizer OFF + RED LED + Buzzer
 *   Step 5  — Tank A Level H/L Limit    → Ionizer OFF
 *   Step 6  — Ionizer Set Alternation (Set 1 ↔ Set 2 every 30 min)
 *   Step 7  — CT1 / CT2 Current Monitor
 *   Step 8  — Receive Solenoid CMD from server → FC06 → Slave 2
 *   Step 9  — Build JSON payload
 *   Step 10 — HTTP POST to Cloud + Mobile App
 *   Step 11 — Update HMI Display
 *   Step 12 — Kick Watchdog
 *
 * Peripheral Assignments:
 *   UART1  → RS485 / Modbus RTU Master (PA9=DE/RE, PA10=RX, PB6=TX)
 *   UART2  → GSM Module (PA2=TX, PA3=RX)
 *   ADC1   → CH0=CT1 (PA0), CH1=CT2 (PA1)
 *   TIM2   → Free-running tick reference
 *   I2C1   → HMI LCD (PB8=SCL, PB9=SDA)
 *   IWDG   → Independent Watchdog (2 s timeout)
 *   GPIOB  → Relay1 (PB0), Relay2 (PB1)
 *   GPIOC  → RED LED (PC0), Buzzer (PC1),
 *             Tank A HIGH (PC2), Tank A LOW (PC3),
 *             Booster Level (PC4)
 *   EXTI   → Inlet WFS pulse (e.g. PB5)
 */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver.h"
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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t total_flow=0;
uint16_t slave1_WF_LTR=0,slave1_sol_sts=0,slave2_WF_LTR=0,slave2_TDS,slave2_TWLS=0;
uint8_t sol_open_cnt=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


/* ─────────────────────────────────────────────────────────────────────────
   Global System Data Structure
───────────────────────────────────────────────────────────────────────── */
static IonizerSystem_t g_sys;

/* ─────────────────────────────────────────────────────────────────────────
   Private Function Prototypes
───────────────────────────────────────────────────────────────────────── */

static void App_Step1_ReadTimestamp(IonizerSystem_t *sys);
static void App_Step2_PollSensors(IonizerSystem_t *sys);
static void App_Step3_MonitorTDS(IonizerSystem_t *sys);
static void App_Step4_MonitorBooster(IonizerSystem_t *sys);
static void App_Step5_MonitorTankA(IonizerSystem_t *sys);
static void App_Step6_IonizerAlternation(IonizerSystem_t *sys);
static void App_Step7_MonitorCT(IonizerSystem_t *sys);
static void App_Step8_SolenoidCMD(IonizerSystem_t *sys);
static void App_Step9_BuildJSON(IonizerSystem_t *sys, char *json_buf, uint16_t buf_size);
static void App_Step10_CloudSync(IonizerSystem_t *sys, const char *json_buf);
static void App_Step11_UpdateHMI(IonizerSystem_t *sys);
static void App_Step12_KickWDT(IonizerSystem_t *sys);
static void App_HandleFaults(IonizerSystem_t *sys);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
	return len;
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
  DRV_Init(&g_sys);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
//  HAL_TIM_Base_Start(&htim1);
//  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  DWT_Init();
//  /* ── Driver Layer Init ── */
//     if (DRV_Init(&g_sys) != DRV_OK) {
//         /* Critical init failure — halt with LED blink */
//         while (1) {
//             DRV_RedLED_Set(true);
//             HAL_Delay(200);
//             DRV_RedLED_Set(false);
//             HAL_Delay(200);
//         }
//     }
//
//     /* ── Startup Indication ── */
//     DRV_Buzzer_Beep(2, 100, 100);
//     DRV_HMI_Clear();
//
//     /* ── GSM Self-check ── */
//     if (DRV_GSM_Check(g_sys.huart_gsm) != DRV_OK) {
//         g_sys.fault_gsm = true;
//         DRV_HMI_ShowFault("GSM FAIL");
//         HAL_Delay(1000);
//     }
//
//     /* ── Start with Ionizer Set 1 ── */
     g_sys.active_set = IONIZER_SET1;
     g_sys.ionizer_on = true;
     DRV_Ionizer_ON(IONIZER_SET1);

     /* ── JSON buffer (stack) ── */
     char json_buf[JSON_BUF_SIZE];
     printf("abcd\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




//	  DRV_WFS_GetFlowLPM();

	  /* ── Step 1: Read GSM Timestamp ── */
//	         App_Step1_ReadTimestamp(&g_sys);
//
//	         /* ── Step 2: Read Inlet WFS + Poll Slave MCUs ── */
	         App_Step2_PollSensors(&g_sys);

//	         /* ── Step 3: TDS Monitor ── */
	        // readTDS(g_sys);
	         DRV_TDS_Read(&g_sys);
//	         App_Step3_MonitorTDS(&g_sys);

//	         /* ── Step 4: Booster Tank Level ── */
	        // JSN_SR04_Trigger();
	       //  HAL_Delay(100);
	         read_distance(&g_sys);
//	         App_Step4_MonitorBooster(&g_sys);
//
//	         /* ── Step 5: Tank A Level ── */
//	         App_Step5_MonitorTankA(&g_sys);
//
//	         /* ── Step 6: Ionizer Set Alternation ── */
             App_Step6_IonizerAlternation(&g_sys);
//
//	         /* ── Step 7: CT Sensor Monitor ── */
	         App_Step7_MonitorCT(&g_sys);
//
	         /* ── Step 8: Solenoid CMD from Server ── */
	         App_Step8_SolenoidCMD(&g_sys);
//
//	         /* ── Step 9: Build JSON Payload ── */
	         App_Step9_BuildJSON(&g_sys, json_buf, sizeof(json_buf));
//
//	         /* ── Step 10: Cloud + Mobile App Sync ── */
	         App_Step10_CloudSync(&g_sys, json_buf);
//
//	         /* ── Step 11: HMI Display Update ── */
//	         App_Step11_UpdateHMI(&g_sys);
//
//	         /* ── Step 12: Kick Watchdog ── */
//	         App_Step12_KickWDT(&g_sys);

	         /* Loop delay — adjust for desired polling rate */
	         HAL_Delay(100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ─────────────────────────────────────────────────────────────────────────
   STEP 1 — READ GSM TIMESTAMP
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read current date/time from GSM module and store in system struct.
 *         On failure the timestamp field retains its previous value.
 */
static void App_Step1_ReadTimestamp(IonizerSystem_t *sys)
{
    char ts_buf[32] = {0};
    DRV_Status st = DRV_GSM_GetTimestamp(sys->huart_gsm, ts_buf, sizeof(ts_buf));
    if (st == DRV_OK) {
        strncpy(sys->timestamp, ts_buf, sizeof(sys->timestamp) - 1);
        sys->fault_gsm = false;
    } else {
        /* Keep previous timestamp; mark fault */
        sys->fault_gsm = true;
        DBG_PRINT("GSM Timestamp read failed");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 2 — READ INLET WFS + POLL SLAVE MCUs
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read inlet flow sensor and poll both slave MCUs via Modbus FC03.
 *         Slave 1: HR[0] = Flow Rate
 *         Slave 2: HR[0] = Flow Rate, HR[1] = TDS value
 */
static void App_Step2_PollSensors(IonizerSystem_t *sys)
{
    /* ── Inlet WFS (direct GPIO pulse count) ── */
    sys->inlet_flow_lpm = DRV_WFS_GetFlowLPM();
    total_flow= DRV_WFS_GetFlowLPM();

    /* ── Poll Slave 1 — read 1 register (HR[0] = Flow) ── */
    uint16_t s1_data[4] = {0};
    DRV_Status st1 = DRV_Modbus_ReadHR(
        sys->huart_rs485, SLAVE1_ADDR,
        S1_HR_FLOW_RATE, 4, s1_data);

    if (st1 == DRV_OK) {
     //    Register value stored as x10 (e.g. 125 = 12.5 L/min)
        sys->slave1_flow_lpm  =   (float)s1_data[0] / 10.0f;
        slave1_WF_LTR         =    s1_data[0];

        sys->solenoid_status  =    s1_data[1];
        slave1_sol_sts        =    s1_data[1];
        sys->fault_modbus_s1  = false;
    } else {
        sys->fault_modbus_s1 = true;
        DBG_PRINT("Slave 1 poll failed (err=%d)", (int)st1);
    }

    HAL_Delay(100);
    // ── Poll Slave 2 — read 2 registers (HR[0]=Flow, HR[1]=TDS) ──
    uint16_t s2_data[5] = {0};
    DRV_Status st2 = DRV_Modbus_ReadHR(
        sys->huart_rs485, SLAVE2_ADDR,
        S2_HR_FLOW_RATE, 5, s2_data);

    if (st2 == DRV_OK) {
        sys->slave2_flow_lpm  = (float)s2_data[0] / 10.0f;
        slave2_WF_LTR         =    s2_data[0];

        sys->tds_ppm          = s2_data[1];
        slave2_TDS            =s2_data[1];

        sys->TWLS             = s2_data[4];
        slave2_TWLS            = s2_data[4];
        sys->fault_modbus_s2  = false;
    } else {
        sys->fault_modbus_s2 = true;
        DBG_PRINT("Slave 2 poll failed (err=%d)", (int)st2);
    }

   // HAL_Delay(1000);
    /*DBG_PRINT("Inlet=%.1f S1=%.1f S2=%.1f TDS=%uppm",
              (double)sys->inlet_flow_lpm,
              (double)sys->slave1_flow_lpm,
              (double)sys->slave2_flow_lpm,
              sys->tds_ppm);*/
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 3 — TDS MONITOR  (Threshold: 80–100 ppm)
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Check TDS value. If outside 80–100 ppm, turn ionizer OFF.
 */
static void App_Step3_MonitorTDS(IonizerSystem_t *sys)
{
    if (sys->tds_ppm < TDS_MIN_PPM || sys->tds_ppm > TDS_MAX_PPM) {
        sys->fault_tds_out_of_range = true;
        if (sys->ionizer_on) {
            sys->ionizer_on = false;
            DRV_Ionizer_OFF_All();
            DRV_HMI_ShowFault("TDS OUT RANGE!");
            DRV_Buzzer_Beep(3, 200, 100);
            DBG_PRINT("TDS OUT OF RANGE (%u ppm) — Ionizer OFF", sys->tds_ppm);
        }
    } else {
        if (sys->fault_tds_out_of_range) {
            /* TDS returned to range — allow ionizer to restart */
            sys->fault_tds_out_of_range = false;
            DBG_PRINT("TDS back in range (%u ppm)", sys->tds_ppm);
        }
    }

    /* Re-evaluate overall ionizer enable state */
    App_HandleFaults(sys);
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 4 — BOOSTER TANK LEVEL MONITOR
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Monitor booster tank level sensor.
 *         If below minimum: Ionizer OFF + RED LED + Buzzer.
 */
static void App_Step4_MonitorBooster(IonizerSystem_t *sys)
{
    sys->booster_level_low = DRV_Booster_LevelLow();

    if (sys->booster_level_low) {
        sys->fault_booster_low = true;
        if (sys->ionizer_on) {
            sys->ionizer_on = false;
            DRV_Ionizer_OFF_All();
            DBG_PRINT("BOOSTER LOW — Ionizer OFF");
        }
        DRV_RedLED_Set(true);
        DRV_Buzzer_Set(true);   /* Continuous alert */
        DRV_HMI_ShowFault("BOOSTER LOW!");
    } else {
        if (sys->fault_booster_low) {
            sys->fault_booster_low = false;
            DRV_RedLED_Set(false);
            DRV_Buzzer_Set(false);
            DBG_PRINT("Booster level restored");
        }
    }

    App_HandleFaults(sys);
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 5 — TANK A WATER LEVEL MONITOR  (HIGH / LOW limit)
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Monitor Tank A level sensors.
 *         If HIGH or LOW limit is reached, turn Ionizer OFF.
 */
static void App_Step5_MonitorTankA(IonizerSystem_t *sys)
{
    sys->tank_a_high_limit = DRV_TankA_HighLimit();
    sys->tank_a_low_limit  = DRV_TankA_LowLimit();

    if (sys->tank_a_high_limit || sys->tank_a_low_limit) {
        sys->fault_tank_a_limit = true;
        if (sys->ionizer_on) {
            sys->ionizer_on = false;
            DRV_Ionizer_OFF_All();
            const char *msg = sys->tank_a_high_limit ? "TANK A HIGH!" : "TANK A LOW!";
            DRV_HMI_ShowFault(msg);
            DBG_PRINT("Tank A limit — %s — Ionizer OFF", msg);
        }
    } else {
        if (sys->fault_tank_a_limit) {
            sys->fault_tank_a_limit = false;
            DBG_PRINT("Tank A level normal");
        }
    }

    App_HandleFaults(sys);
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 6 — IONIZER SET ALTERNATION
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Alternate between Ionizer Set 1 and Set 2 after Timer T expires.
 *         Only operates when all fault flags are clear.
 */
static void App_Step6_IonizerAlternation(IonizerSystem_t *sys)
{
    if (!sys->ionizer_on) return;
    DRV_Ionizer_Alternate(sys);
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 7 — CT SENSOR MONITOR
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read CT1 and CT2 sensors via ADC.
 *         Flag fault if current is outside normal operating range.
 */
static void App_Step7_MonitorCT(IonizerSystem_t *sys)
{
    /* Read CT1 */
    if (DRV_ADC_ReadCT1(sys->hadc, &sys->ct1_raw) == DRV_OK) {
        sys->fault_ct1 = DRV_CT_IsFault(sys->ct1_raw);
        if (sys->fault_ct1) {
            DRV_HMI_ShowFault("CT1 FAULT!");
            DRV_Buzzer_Beep(2, 150, 100);
            DBG_PRINT("CT1 FAULT raw=%u", sys->ct1_raw);
        }
    }

    /* Read CT2 */
    if (DRV_ADC_ReadCT2(sys->hadc, &sys->ct2_raw) == DRV_OK) {
        sys->fault_ct2 = DRV_CT_IsFault(sys->ct2_raw);
        if (sys->fault_ct2) {
            DRV_HMI_ShowFault("CT2 FAULT!");
            DRV_Buzzer_Beep(2, 150, 100);
            DBG_PRINT("CT2 FAULT raw=%u", sys->ct2_raw);
        }
    }

    /* Read CT3 */
      if (DRV_ADC_ReadCT3(sys->hadc, &sys->ct3_raw) == DRV_OK) {
          sys->fault_ct3 = DRV_CT_IsFault(sys->ct3_raw);
          if (sys->fault_ct3) {
              DRV_HMI_ShowFault("CT3 FAULT!");
              DRV_Buzzer_Beep(2, 150, 100);
              DBG_PRINT("CT3 FAULT raw=%u", sys->ct3_raw);
          }
      }

      /* Read CT4 */
      if (DRV_ADC_ReadCT4(sys->hadc, &sys->ct4_raw) == DRV_OK) {
          sys->fault_ct4 = DRV_CT_IsFault(sys->ct4_raw);
          if (sys->fault_ct4) {
              DRV_HMI_ShowFault("CT4 FAULT!");
              DRV_Buzzer_Beep(2, 150, 100);
              DBG_PRINT("CT4 FAULT raw=%u", sys->ct4_raw);
          }
      }

    DBG_PRINT("CT1=%u CT2=%u CT3=%u CT4=%u", sys->ct1_raw, sys->ct2_raw);
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 8 — SOLENOID CMD FROM SERVER → FC06 → SLAVE 2
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Check if server has sent a solenoid command via GSM.
 *         If "OPEN" → FC06 write SOLENOID_OPEN to Slave 2 HR[2].
 *         If "CLOSE"→ FC06 write SOLENOID_CLOSE to Slave 2 HR[2].
 */
static void App_Step8_SolenoidCMD(IonizerSystem_t *sys)
{
    char cmd[32] = {0};
    uint16_t write_val;


 /*   DRV_Status st = DRV_GSM_ReceiveCMD(sys->huart_gsm, cmd, sizeof(cmd));

    if (st != DRV_OK) return;   No pending command


    if (strncmp(cmd, "OPEN", 4) == 0) {
        write_val = SOLENOID_OPEN;
        sys->solenoid_status = SOLENOID_OPEN;
        DBG_PRINT("Server CMD: OPEN Solenoid");
    } else if (strncmp(cmd, "CLOSE", 5) == 0) {
        write_val = SOLENOID_CLOSE;
        sys->solenoid_status = SOLENOID_CLOSE;
        DBG_PRINT("Server CMD: CLOSE Solenoid");
    } else {
        DBG_PRINT("Unknown server CMD: %s", cmd);
        return;
    }
*/

   // write_val=SOLENOID_CLOSE;    // for testing purpose
    if(slave1_WF_LTR >= 1)
    {
    write_val=SOLENOID_CLOSE;    // for testing purpose
    }
    else if(slave1_WF_LTR < 1 && sol_open_cnt == 0)
    {
    	write_val=SOLENOID_OPEN;   // for testing purpose
    	sol_open_cnt++;
    }

    /* Send FC06 to Slave  */
    DRV_Status mb_st = DRV_Modbus_WriteSR(
        sys->huart_rs485,
        SLAVE1_ADDR,
        S1_HR_SOLENOID,
        write_val);

    if (mb_st != DRV_OK) {
        DBG_PRINT("FC06 to Slave 1 FAILED (err=%d)", (int)mb_st);
        sys->fault_modbus_s2 = true;
    } else {
        DBG_PRINT("FC06 Solenoid CMD sent OK to Slave 1");
    }
    UART_Restart();
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 9 — BUILD JSON PAYLOAD
───────────────────────────────────────────────────────────────────────── */

static void App_Step9_BuildJSON(IonizerSystem_t *sys, char *json_buf, uint16_t buf_size)
{
    DRV_JSON_Build(sys, json_buf, buf_size);
    DBG_PRINT("JSON: %s", json_buf);
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 10 — CLOUD SYNC + MOBILE APP
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Send JSON payload to cloud server via GSM HTTP POST.
 *         Retries up to MODBUS_RETRIES times on failure.
 */
static void App_Step10_CloudSync(IonizerSystem_t *sys, const char *json_buf)
{
    DRV_Status st = DRV_ERROR;
    for (uint8_t attempt = 0; attempt < MODBUS_RETRIES; attempt++) {
        st = DRV_GSM_SendHTTP(sys->huart_gsm, SERVER_URL, json_buf);
        if (st == DRV_OK) {
            sys->fault_gsm = false;
            DBG_PRINT("Cloud sync OK (attempt %d)", attempt + 1);
            break;
        }
        HAL_Delay(500);
    }
    if (st != DRV_OK) {
        sys->fault_gsm = true;
        DBG_PRINT("Cloud sync FAILED after %d attempts", MODBUS_RETRIES);
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 11 — UPDATE HMI DISPLAY
───────────────────────────────────────────────────────────────────────── */

static void App_Step11_UpdateHMI(IonizerSystem_t *sys)
{
    DRV_HMI_Update(sys);
}

/* ─────────────────────────────────────────────────────────────────────────
   STEP 12 — KICK WATCHDOG
───────────────────────────────────────────────────────────────────────── */

static void App_Step12_KickWDT(IonizerSystem_t *sys)
{
  //  DRV_WDT_Kick(sys->hiwdg);
}



/* ─────────────────────────────────────────────────────────────────────────
   FAULT HANDLER — Evaluate all faults, enable Ionizer if all clear
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Central fault evaluator.
 *         If all safety faults are cleared, re-enable the ionizer.
 *         If any fault is active, ensure ionizer stays OFF.
 */
static void App_HandleFaults(IonizerSystem_t *sys)
{
    bool any_critical_fault =
        sys->fault_tds_out_of_range ||
        sys->fault_booster_low      ||
        sys->fault_tank_a_limit;

    if (any_critical_fault) {
        if (sys->ionizer_on) {
            sys->ionizer_on = false;
            DRV_Ionizer_OFF_All();
        }
    } else {
        /* All critical faults cleared — re-enable ionizer */
        if (!sys->ionizer_on) {
            sys->ionizer_on = true;
            /* Resume from the last active set */
            if (sys->active_set == IONIZER_SET_NONE)
                sys->active_set = IONIZER_SET1;
            DRV_Ionizer_ON(sys->active_set);
            DRV_RedLED_Set(false);
            DRV_Buzzer_Set(false);
            DBG_PRINT("All faults cleared — Ionizer restarted Set %d",
                      (int)sys->active_set);
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   EXTI ISR — Inlet Water Flow Sensor Pulse
   Called by stm32f1xx_it.c HAL_GPIO_EXTI_Callback
───────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Override HAL EXTI callback for inlet WFS pulse counting.
 *         Wire the inlet WFS output to the configured EXTI GPIO pin.
 * @note   GPIO_PIN_5 = PB5 in this example; change to your pin.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15) {   /* Inlet WFS pin — adjust as needed */
        DRV_WFS_PulseCallback();
    }
}


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
