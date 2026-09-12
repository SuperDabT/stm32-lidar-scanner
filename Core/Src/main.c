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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "servo.h"
#include "tfluna.h"
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
/* USER CODE BEGIN PFP */
bool elapsed(uint32_t *last, uint32_t interval);
int _write(int file,char *ptr, int len);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
servo_init();
tfluna_init();




/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		while (1) {

    /* Static so they survive between iterations. The loop body runs
       thousands of times a second; the timed block inside is what actually
       steps the scan. */
    
    static bool halted=false;
    static float    pan_angle     = 0.0f;
    static float    pan_step      = 2.0f;
    static uint32_t pan_last_move = 0;
    static uint32_t warn_last=0;

    static float tilt_angle = TILT_MAX;
    static float tilt_step  = -3.0f;

    bool reversed = false;

    /* 50 ms is measured, not chosen. A servo commanded to a new angle is
       still travelling when the next reading arrives, so the reading
       belongs to a position the code has already left. The error shows up
       as a sharp edge landing at different bearings depending on sweep
       direction: 40 ms gives 6 degrees of split, 45 gives 4, 50 gives 2,
       65 gives none. 50 is the fastest interval holding the split within
       one 2-degree step, which is the floor either way. */
    
    /* Two thresholds, not one. The TF-Luna's ceiling is 60 C and it runs
         43-52 in normal operation, so a single threshold would flap on and
         off at the boundary. Trips at 60, clears at 55; between the two it
         holds whatever state it's in. */

    
    if (elapsed(&pan_last_move, 50)) {
      if(tfluna_temperature()>=60.0f){
        halted=true;
    }
    if(tfluna_temperature()<=55.0f){
      halted=false;
    }
    
    if (halted){
      if (elapsed(&warn_last, 1000)){
      printf("WARNING: TF-Luna at %d C, SCANNING HALTED\r\n",
      (int)tfluna_temperature());
    }
  }
    else{

        /* Report before commanding. At this point pan_angle is still the
           angle the head has been parked at for the last 50 ms, so the
           reading and the label match. Printing after the move would
           attach each reading to the position it was heading toward
           rather than the one it came from. */
        printf("%d,%d,%d,%d,%d\r\n",
          (int)pan_angle,
          (int)tilt_angle,
          tfluna_distance(),
          (int)tfluna_temperature(),
          tfluna_valid());

        pan_angle += pan_step;
        servo_write(pan_angle, 'p');

        /* Limit test after the command, so the angle written is always the
           limit itself and never one step past it. */
        if (pan_angle >= PAN_MAX) {
            pan_step = -pan_step;
            reversed = true;
        }
        if (pan_angle <= PAN_MIN) {
            pan_step = -pan_step;
            reversed = true;
        }

        /* Tilt advances once per completed pan sweep, giving stacked
           horizontal rows rather than the diagonal smear you get from
           stepping both axes together. Step stays at 3 degrees: the beam
           is about 2 wide, so a larger step would leave unscanned gaps
           between rows. */
        if (reversed) {
            servo_write(tilt_angle, 't');

            tilt_angle += tilt_step;
            if (tilt_angle >= TILT_MAX) {
              tilt_angle=TILT_MAX;
              tilt_step = -tilt_step;
            }
            if (tilt_angle <= TILT_MIN) {
              tilt_angle=TILT_MIN;
              tilt_step = -tilt_step;

            }
        }
    }
  }
	}


            
  

    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
	
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/* USER CODE BEGIN 4 */

/* Non-blocking interval timer. Takes a pointer so each caller keeps its own
   schedule in the same loop — no blocking delays anywhere in normal
   operation. Everything stays uint32_t deliberately: HAL_GetTick wraps
   after about 49 days, and unsigned subtraction wraps with it, so the
   elapsed time stays correct across the rollover. Storing the result in a
   signed type would make the comparison fail permanently. */

bool elapsed(uint32_t *last, uint32_t interval) {
	if (HAL_GetTick() - *last >= interval) {
		*last = HAL_GetTick();
		return true;
	} else
		return false;
}

/* printf retarget. newlib calls _write for stdout, so this puts printf on
   USART2 and the ST-Link virtual COM port. Blocking: a 20-character CSV
   line is about 1.7 ms at 115200, which sits inside the 50 ms budget but
   is real time spent, and it grows if fields are added. */

int _write(int file,char *ptr, int len){
	HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,HAL_MAX_DELAY);
	return len;
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
