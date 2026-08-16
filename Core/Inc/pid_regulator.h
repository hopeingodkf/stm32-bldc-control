#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Використовуємо константу з main.h, але залишаємо цю для сумісності */
#ifndef HALL_TICK_HZ
#define HALL_TICK_HZ 10000.0f
#endif

typedef struct {
    /* налаштування */
    float Kp, Ki, Kd;
    float target_hz;
    float ema_alpha;
    /* стан */
    float i_term;
    float prev_err;
    float freq_filt;
    float out_min;
    float out_max;
    /* PWM */
    TIM_HandleTypeDef *pwm_htim;
    uint32_t pwm_channel;
    uint32_t pwm_arr;
    /* прапори */
    uint8_t enabled;
} PIDCtrl_t;

/* API */
void PID_Init(TIM_HandleTypeDef *htim_pwm, uint32_t channel);
void PID_Update(uint32_t hall_period_ticks);
void PID_SetTunings(float kp, float ki, float kd);
void PID_SetTarget(float hz);
void PID_SetEMA(float alpha);
void PID_SetOutputLimits(float min_frac, float max_frac);
void PID_Enable(uint8_t en);

/* --- ФУНКЦІЇ ДЛЯ ТЕЛЕМЕТРІЇ --- */
void PID_GetTunings(float *Kp, float *Ki, float *Kd);
float PID_GetTargetFrequency(void);
uint32_t PID_GetOutputPWM(void); // <-- ОСЬ ФУНКЦІЯ, ЯКОЇ НЕ ВИСТАЧАЄ
/* ------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* PID_REGULATOR_H */
