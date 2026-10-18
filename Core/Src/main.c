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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_libWrapper.h"

#include "uartDMA.h"
#include "bms_can.h"
#include "bms_gpio.h"
#include "bms_ivt.h"
#include "bms_safety.h"
#include "bms_precharge.h"
#include "bms_inverter.h"
#include "bms_settings.h"
#include "bms_charger.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    volatile uint32_t runtime_sec;
    volatile uint32_t counter_commsError;
    volatile uint32_t counter_commsErrorCumulative;
} ProgramStats;

typedef BmsImdStatus ImdStatus;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

ProgramStats    programStats = {0};
ImdStatus       imdStatus = {0};
static volatile struct {uint32_t period,high;bool pending;} imdCapture;

volatile bool initRequired = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void BMS_WriteFanDuty(float duty);
void BMS_CommsRecovery(void);
static void BMS_PollChargerButton(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    BMS_Startup();

    while (1)
    {
        BMS_MainLoopIteration();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Everything after the CubeMX peripheral init, up to the main loop
void BMS_Startup(void)
{
    // Board I/O from bms_config.h. Fault output asserted and TSAC ready relay off until proven safe.
    BMS_GPIO_Init();

    // Runtime settings saved from the GUI (inverter limits, charger setpoints), before anything uses them
    BMS_Settings_Init();

    // Configure FDCAN (CAN1 car bus, CAN2 inside the TSAC, CAN3 charger)
    BMS_CAN_Config();

    // Start Timer16 (fast CAN frames, safety tick and CAN burst)
    if(HAL_TIM_Base_Start_IT(&htim16)!=HAL_OK ||
       HAL_TIM_IC_Start_IT(&htim15,TIM_CHANNEL_1)!=HAL_OK ||
       HAL_TIM_IC_Start(&htim15,TIM_CHANNEL_2)!=HAL_OK ||
       HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2)!=HAL_OK)Error_Handler();
    BMS_WriteFanDuty(0.7);

    printfDma("\n\n +++++             PROGRAM START             +++++ \n\n");
    printfDma("Board profile: %s, slaves=%d, energetic capabilities B/C/H/I=%d/%d/%d/%d\n",BMS_PROFILE_NAME,TOTAL_IC,BMS_CAN_BALANCE,BMS_CAN_ACTUATE_CONTACTORS,BMS_CAN_ENABLE_CHARGER,BMS_CAN_DRIVE_INVERTER);
    if (CFG_LOCAL_OVERRIDES)
    {
        printfDma(" *** BENCH BUILD: bms_config_local.h overrides bms_config.h. NOT FOR THE CAR. *** \n\n");
    }
    printfDma("Inputs: SDC %s, charger button %s\n", BMS_GPIO_IsSdcClosed() ? "closed" : "open",
              BMS_GPIO_IsChargerButtonPressed() ? "pressed" : "released");
    bms_softReset();
    HAL_Delay(500);         // Initialisation delay, also covers the IVT-S start-up time

    if(!bms_wakeupChain())BMS_RequestSafeOutputs();
    bms_softReset();

    // From here the main loop must run at least every CFG_WATCHDOG_TIMEOUT_MS
    BMS_Watchdog_Start();
}


// One pass of the main loop: measure, react, then wait out the rest of CFG_MEAS_PERIOD_MS
// Logs each change of the fault flags or the fault output with the tick it happened on
static void logSafetyEvents(void)
{
    static const char* const names[] = { "COMMS", "VOLTAGE", "TEMP", "CURRENT", "OPEN_WIRE", "IVT", "STARTUP" };
    static bool     lastOutput = true;
    static uint32_t lastLost   = 0;
    SafetyEvent e;
    uint32_t lost;

    while (BMS_Safety_PopEvent(&e, &lost))
    {
        char changes[96];
        int  n = 0;
        for (uint32_t b = 0; b < sizeof(names) / sizeof(names[0]) && n < (int)sizeof(changes) - 16; b++)
        {
            uint32_t bit = 1UL << b;
            if ((e.flags ^ e.prevFlags) & bit)
            {
                n += snprintf(&changes[n], sizeof(changes) - n, " %c%s", (e.flags & bit) ? '+' : '-', names[b]);
            }
        }
        changes[n] = '\0';
        const char* output = e.faultOutput == lastOutput ? "" : e.faultOutput ? "output ON" : "output off";
        if (e.flags != e.prevFlags)
        {
            printfDma("Fault %lu ms: 0x%02lX -> 0x%02lX%s%s%s\n", (unsigned long)e.tick, (unsigned long)e.prevFlags,
                      (unsigned long)e.flags, changes, output[0] ? ", " : "", output);
        }
        else
        {
            printfDma("Fault %lu ms: %s\n", (unsigned long)e.tick, output);
        }
        lastOutput = e.faultOutput;
        if (lost != lastLost)
        {
            printfDma("Fault log: %lu changes not logged\n", (unsigned long)(lost - lastLost));
            lastLost = lost;
        }
    }
}


void BMS_MainLoopIteration(void)
{
    BMS_StatusTypeDef bmsStatus;
    BMS_ProcessImdCapture();
    uint32_t timeStart = HAL_GetTick();

#ifdef BMS_TEST_WATCHDOG
    // Bring-up test build only: hang after 10 s. The watchdog must reset the MCU with the fault output held.
    if (timeStart > 10000)
    {
        printfDma("BMS_TEST_WATCHDOG: hanging the main loop\n");
        while (1)
        {
        }
    }
#endif

    BMS_Watchdog_Kick();
    bool commsFailed = false;

    // Init BMS
    if (initRequired == true)
    {
        bmsStatus = bms_init();
        if (bmsStatus == BMS_OK)
        {
            initRequired = false;
        }
        else
        {
            BMS_CommsRecovery();
            commsFailed = true;
        }
    }

    // BMS Program Loop. Limit violations go to the safety module, only comms errors come back here.
    if (initRequired == false)
    {
        bmsStatus = BMS_ProgramLoop();
        if (bmsStatus == BMS_OK)
        {
            programStats.counter_commsError = 0;
        }
        else
        {
            BMS_CommsRecovery();
            commsFailed = true;
        }
    }

    // Never charge while the AMS is faulted
    if (BMS_IsCharging() && BMS_Safety_IsFaultOutputActive())
    {
        printfDma("Fault Detected, Disabling Charging \n");
        BMS_EnableCharging(false);
    }

    BMS_IVT_Service();
    BMS_Settings_Service();
    logSafetyEvents();

    // Calculate single loop runtime
    uint32_t timeDiff = HAL_GetTick() - timeStart;
    if (CFG_DEBUG_PRINT_LOOP)
    {
        printfDma("Runtime: %lu ms, LoopTime: %lu ms, Faults: 0x%02lX \n", (unsigned long)HAL_GetTick(),
                  (unsigned long)timeDiff, (unsigned long)BMS_Safety_GetActiveFlags());
    }
    if (timeDiff > CFG_MEAS_PERIOD_MS)
    {
        printfDma("WARNING! Loop overrun: %lu ms \n", (unsigned long)timeDiff);
    }
    if (BMS_GetLoopTimeMs() > CFG_MEAS_CYCLE_MAX_MS)
    {
        printfDma("WARNING! Measurement cycle %lu ms, longer than CFG_MEAS_CYCLE_MAX_MS \n",
                  (unsigned long)BMS_GetLoopTimeMs());
    }

    // Wait for the rest of the measurement period, answering settings requests and looking after the IVT meanwhile.
    // A failed cycle is retried after CFG_COMMS_RETRY_DELAY_MS while the measurements aren't stale yet, so a brief
    // isoSPI glitch or slave power dip is ridden through; after that, at the normal period.
    bool retrySoon = commsFailed && BMS_Safety_GetMeasurementAgeMs() < CFG_MEAS_STALE_TIMEOUT_MS;
    uint32_t waitFrom = retrySoon ? HAL_GetTick() : timeStart;
    uint32_t waitFor  = retrySoon ? CFG_COMMS_RETRY_DELAY_MS : CFG_MEAS_PERIOD_MS;
    while (HAL_GetTick() - waitFrom < waitFor)
    {
        BMS_Watchdog_Kick();
        BMS_Settings_Service();
        BMS_IVT_Service();
        logSafetyEvents();
    }
}


/// Timer interrupt callback
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // runtime (seconds) timer
    static uint32_t secondsDiv = 0;

    // CAN Transmit Periodic Timing Variable
    static uint32_t canTiming = 0;

    // Timer tick dividers. The inverter limits go out half a fast period after the fast frames.
    static uint32_t fastTiming = 0;
    static uint32_t inverterTiming = CFG_CAN_FAST_PERIOD_MS / 2 / CFG_TIMER_TICK_MS;
    static uint32_t safetyTiming = 0;

    if (htim == &htim16) // Every CFG_TIMER_TICK_MS
    {
        BMS_IVT_Tick();                             // U1 averaged over the cell conversion
        BMS_Inverter_Tick();                        // 500 ms moving average of the discharge power

        // Frames queued this tick: only 2 TX buffers are kept free, so each tick sends at most one group
        bool sentThisTick = false;
        static bool responseSentThisPeriod = false;

        if (++fastTiming >= CFG_CAN_FAST_PERIOD_MS / CFG_TIMER_TICK_MS)
        {
            CanTxMsg fastMsgs[CAN_FAST_FRAME_COUNT];
            BMS_CAN_SendFast(fastMsgs, BMS_GetFastCanData(fastMsgs));
            fastTiming = 0;
            sentThisTick = true;
            responseSentThisPeriod = false;
        }

        // Inverter limits, then the BMS_Limits frame on the next tick
        static bool limitsFramePending = false;
        if (++inverterTiming >= CFG_INVERTER_LIMIT_PERIOD_MS / CFG_TIMER_TICK_MS)
        {
            BMS_Inverter_Update();
            if (CFG_INVERTER_LIMITS_ENABLED)
            {
                CanTxMsg limitMsgs[INVERTER_LIMIT_FRAME_COUNT];
                BMS_CAN_SendFast(limitMsgs, BMS_Inverter_GetCanFrames(limitMsgs));
                sentThisTick = true;
            }
            limitsFramePending = true;
            inverterTiming = 0;
        }
        else if (limitsFramePending)
        {
            CanTxMsg limitsFrame;
            BMS_Inverter_GetLimitsFrame(&limitsFrame);
            BMS_CAN_SendFast(&limitsFrame, 1);
            limitsFramePending = false;
            sentThisTick = true;
        }

        // Settings responses: at most one per fast period, on a tick with nothing else
        CanTxMsg response;
        if (!sentThisTick && !responseSentThisPeriod && BMS_Settings_GetResponse(&response))
        {
            BMS_CAN_SendFast(&response, 1);
            responseSentThisPeriod = true;
        }

        if (++safetyTiming < CFG_SAFETY_TICK_MS / CFG_TIMER_TICK_MS)
        {
            return;
        }
        safetyTiming = 0;

        // Every CFG_SAFETY_TICK_MS from here
        // Fault evaluation, AMS fault output and TSAC ready relay, then precharge (same tick)
        BMS_Safety_Tick();
        BMS_Precharge_Tick();
        BMS_Charger_Tick();
        BMS_CAN_RecoverBusOff();

        if (++secondsDiv >= 1000 / CFG_SAFETY_TICK_MS)
        {
            programStats.runtime_sec += 1;
            secondsDiv = 0;
        }

        // CAN Timing Divider
        if (++canTiming >= CFG_CAN_TX_PERIOD_MS / CFG_SAFETY_TICK_MS)
        {
            CanTxMsg *msgArr = NULL;
            uint32_t len = 0;
            BMS_GetCanData(&msgArr, &len);
            BMS_CAN_SendBuffer(msgArr, len);

            CanTxMsg charger;
            BMS_GetChargerCanMsg(&charger);
            BMS_CAN_SendCharger(&charger);          // CAN3
            BMS_Charger_GetInhibit(&charger); BMS_CAN_SendCharger(&charger);
            canTiming = 0;
        }

        BMS_PollChargerButton();

        // If charging and SDC is disconnected:
        if (BMS_IsCharging() && !BMS_GPIO_IsSdcClosed())
        {
            printfDma("SDC Disconnected, Disabling Charging \n");
            BMS_EnableCharging(false);
        }
    }
}


void BMS_ProcessImdCapture(void)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();
    bool pending=imdCapture.pending;uint32_t period=imdCapture.period,high=imdCapture.high;
    imdCapture.pending=false;__set_PRIMASK(mask);
    if(!pending)return;
    imdStatus.period=period;
    if(period) {imdStatus.duty=(float)(((double)high*100.0)/(double)period);imdStatus.freq=CFG_IMD_TIMER_CLOCK_HZ/period;}
}
BmsImdStatus BMS_GetImdStatus(void)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();BmsImdStatus status=imdStatus;__set_PRIMASK(mask);return status;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim15) {
        imdCapture.period=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
        imdCapture.high=imdCapture.period ? HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2):0;
        imdCapture.pending=true;
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currentTime = HAL_GetTick();

    const uint32_t DEBOUNCE_DELAY = 500;
    static uint32_t lastDebounceTime_B1 = 0;

    switch (GPIO_Pin)
    {
    case B1_Pin:                // Blue onboard button (for debugging mainly)
        // Debounce check
        if (currentTime - lastDebounceTime_B1 < DEBOUNCE_DELAY) break;
        lastDebounceTime_B1 = currentTime;
        printfDma("Blue Button pressed\n");
        break;

    default:
        break;
    }
}


// Charger button, polled from the timer so its pin can be set in bms_config.h
static void BMS_PollChargerButton(void)
{
    const uint32_t DEBOUNCE_TICKS = 50 / CFG_SAFETY_TICK_MS;
    static uint32_t pressedTicks = 0;
    static bool handled = false;

    if (!BMS_GPIO_IsChargerButtonPressed())
    {
        pressedTicks = 0;
        handled = false;
        return;
    }

    if (pressedTicks < DEBOUNCE_TICKS)
    {
        pressedTicks++;
    }
    else if (!handled)
    {
        handled = true;
        printfDma("Charger Button pressed\n");
        BMS_ChargingButtonLogic();
    }
}


// Non-blocking: the safety tick faults on stale data, this only gets the chain talking again.
// A single PEC error is usually noise, so it is just retried on the next cycle. If the chain keeps
// failing it is reset and re-initialised.
void BMS_CommsRecovery(void)
{
    programStats.counter_commsError++;
    programStats.counter_commsErrorCumulative++;

    if (programStats.counter_commsError >= CFG_COMMS_RETRIES)
    {
        bms_softReset();
        initRequired = true;
    }
}

void BMS_WriteFanDuty(float duty)
{
    if (!isfinite(duty) || duty > 1 || duty <= 0)
        return;

    duty = 1 - duty;

    TIM3->CCR2 = (uint16_t)(255.0 * duty);
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
  BMS_Safety_ForceFault();      // Safe outputs
  // The watchdog then resets the MCU. Started here if it isn't running yet (a clock or peripheral init failed,
  // e.g. the HSE crystal starting too slowly): a reset tries again instead of hanging until a power cycle.
  BMS_Watchdog_Start();
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
