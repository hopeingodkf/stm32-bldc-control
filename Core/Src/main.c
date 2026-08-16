/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @note           : керування параметрами стробів через USB.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid_regulator.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "critical.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLAMP_VALUE(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_pulse_width_permille = 150;
volatile uint32_t g_phase_shift_permille = 250;
volatile uint32_t g_start_delay_permille = 150;
volatile uint16_t adc_buf[3];
volatile uint8_t g_is_main_pulse_active = 0;
volatile uint8_t g_is_duplicate_pulse_active = 0;
char g_command_buffer[COMMAND_BUFFER_SIZE];
volatile uint8_t g_new_command_flag = 0;
uint32_t g_last_telemetry_time = 0;
volatile float g_current_real_frequency = 0.0f;
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
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // Потенціометри не використовуються: параметри надходять з програми по USB
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
  PID_Init(&htim4, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    if (g_new_command_flag)
    {
        g_new_command_flag = 0;
        int value_int;
        float value_float;
        char* equals_ptr = strchr(g_command_buffer, '=');
        if (equals_ptr != NULL)
        {
            // Поточні значення зчитуються до розгалуження за режимом
            float current_kp, current_ki, current_kd;
            PID_GetTunings(&current_kp, &current_ki, &current_kd);

            CRITICAL_ENTER();
            switch (g_command_buffer[0])
            {
                case 'p':
                    value_float = atof(equals_ptr + 1);
                    PID_SetTunings(value_float, current_ki, current_kd);
                    break;
                case 'i':
                    value_float = atof(equals_ptr + 1);
                    PID_SetTunings(current_kp, value_float, current_kd);
                    break;
                case 'd':
                    value_float = atof(equals_ptr + 1);
                    PID_SetTunings(current_kp, current_ki, value_float);
                    break;
                case 't':
                    value_float = atof(equals_ptr + 1);
                    PID_SetTarget(value_float);
                    break;
                case 'l': value_int = atoi(equals_ptr + 1); g_start_delay_permille = CLAMP_VALUE(value_int, 0, 300); break;
                case 'w': value_int = atoi(equals_ptr + 1); g_pulse_width_permille = CLAMP_VALUE(value_int, 0, 300); break;
                case 's': value_int = atoi(equals_ptr + 1); g_phase_shift_permille = CLAMP_VALUE(value_int, 0, 500); break;
            }
             CRITICAL_EXIT();
        }
    }

    if (HAL_GetTick() - g_last_telemetry_time > 200)
    {
        g_last_telemetry_time = HAL_GetTick();
        char telemetry_buf[128];
        float kp, ki, kd, target_freq;
        uint32_t delay_pm, width_pm, shift_pm;
        uint32_t current_pwm_val;
        uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim4); // Отримуємо період ШІМ (4420)

        CRITICAL_ENTER();
        PID_GetTunings(&kp, &ki, &kd);
        target_freq = PID_GetTargetFrequency();
        current_pwm_val = PID_GetOutputPWM(); // Отримуємо значення ШІМ
        delay_pm = g_start_delay_permille;
        width_pm = g_pulse_width_permille;
        shift_pm = g_phase_shift_permille;
        CRITICAL_EXIT();

        int32_t freq_int = (int32_t)(g_current_real_frequency * 100);
        int32_t target_int = (int32_t)(target_freq * 100);
        int32_t kp_int = (int32_t)(kp * 10000);
        int32_t ki_int = (int32_t)(ki * 10000);
        int32_t kd_int = (int32_t)(kd * 10000);
        uint32_t duty_permille = 0;
        if (pwm_period > 0) { duty_permille = (current_pwm_val * 1000) / pwm_period; }

        sprintf(telemetry_buf, "iFreq:%ld;iTarget:%ld;iKp:%ld;iKi:%ld;iKd:%ld;iDelay:%lu;iWidth:%lu;iShift:%lu;iDuty:%lu\n",
                freq_int, target_int, kp_int, ki_int, kd_int,
                delay_pm, width_pm, shift_pm, duty_permille);
        CDC_Transmit_FS((uint8_t*)telemetry_buf, strlen(telemetry_buf));
    }

    // Формування вихідних стробів
    uint32_t is_pwm_high = (__HAL_TIM_GET_COUNTER(&htim4) < __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1));
    if (g_is_main_pulse_active && is_pwm_high) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); }
    else { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); }
    if (g_is_duplicate_pulse_active && is_pwm_high) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET); }
    else { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET); }
    /* USER CODE END 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */
// Обробник завершення перетворення АЦП
/* // ЗАКОМЕНТОВАНО: Тепер керуємо через USB
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    g_start_delay_permille = ((uint32_t)adc_buf[0] * 300) / 4095;
    g_pulse_width_permille = ((uint32_t)adc_buf[1] * 300) / 4095;
    g_phase_shift_permille = ((uint32_t)adc_buf[2] * 500) / 4095;
}
*/

// Обробник зовнішнього переривання від датчика Холла (PB10)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == HALL_SENSOR_Pin)
    {
        uint32_t hall_period_ticks = __HAL_TIM_GET_COUNTER(&htim1);
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        if (hall_period_ticks < 50) return;

        PID_Update(hall_period_ticks);

        // Використовуємо TIMER_TICK_FREQUENCY (1MHz) і множимо на 2.0f
        g_current_real_frequency = (TIMER_TICK_FREQUENCY / hall_period_ticks);

        // Розрахунок моментів комутації.
        //
        // Реалізація в цій копії не наводиться.
        //
        // За виміряним періодом датчика Холла і трьома параметрами — затримкою
        // старту, шириною імпульсу та фазовим зсувом — розраховуються чотири
        // моменти часу на період: вмикання і вимикання основного імпульсу та
        // вмикання і вимикання дублюючого. Значення записуються в регістри
        // порівняння таймера, тому самі імпульси формуються апаратно, без
        // участі ядра.
        //
        // Роботу алгоритму показано у відеозаписі, посилання в README.

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

// Обробник переривання від таймера TIM1
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1: g_is_main_pulse_active = 1; break;
            case HAL_TIM_ACTIVE_CHANNEL_2: g_is_main_pulse_active = 0; break;
            case HAL_TIM_ACTIVE_CHANNEL_3: g_is_duplicate_pulse_active = 1; break;
            case HAL_TIM_ACTIVE_CHANNEL_4: g_is_duplicate_pulse_active = 0; break;
        }
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
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
