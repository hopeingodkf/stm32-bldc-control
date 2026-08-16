#include "pid_regulator.h"
#include "main.h" // Потрібен для TIMER_TICK_FREQUENCY (хоча в цій версії не використовується, але може бути)
#include <math.h>
#include "critical.h"

/* --- швидке налаштування --- */
#ifndef PID_KP
#define PID_KP   0.80f
#endif
#ifndef PID_KI
#define PID_KI   0.15f
#endif
#ifndef PID_KD
#define PID_KD   0.05f
#endif
#ifndef PID_TARGET_HZ
#define PID_TARGET_HZ 150.0f
#endif
#ifndef PID_EMA_ALPHA
#define PID_EMA_ALPHA 0.40f
#endif
#ifndef PID_OUT_MIN
#define PID_OUT_MIN    0.05f
#endif
#ifndef PID_OUT_MAX
#define PID_OUT_MAX    0.95f
#endif

#define CLAMP(v, lo, hi)  do{ if((v) < (lo)) (v) = (lo); else if((v) > (hi)) (v) = (hi); }while(0)

static PIDCtrl_t s_pid;
static uint32_t g_last_output_pwm = 0; // <-- ДОДАНО: Змінна для зберігання скважності

static inline float pid_step(PIDCtrl_t *p, float err, float dt)
{
    p->i_term += p->Ki * err * dt;
    CLAMP(p->i_term, p->out_min, p->out_max); // Anti-windup
    float d_term = p->Kd * (err - p->prev_err) / (dt > 1e-6f ? dt : 1e-6f);
    p->prev_err = err;
    float out = (p->Kp * err) + p->i_term + d_term;
    CLAMP(out, p->out_min, p->out_max);
    return out;
}

/* ---------- Публічне API ---------- */

void PID_Init(TIM_HandleTypeDef *htim_pwm, uint32_t channel)
{
    PIDCtrl_t *p = &s_pid;
    p->Kp = PID_KP;
    p->Ki = PID_KI;
    p->Kd = PID_KD;
    p->target_hz = PID_TARGET_HZ;
    p->ema_alpha = PID_EMA_ALPHA;
    p->i_term = 0.0f;
    p->prev_err = 0.0f;
    p->freq_filt = 0.0f;
    p->out_min = PID_OUT_MIN;
    p->out_max = PID_OUT_MAX;
    p->pwm_htim   = htim_pwm;
    p->pwm_channel= channel;
    p->pwm_arr    = __HAL_TIM_GET_AUTORELOAD(htim_pwm);
    g_last_output_pwm = 0; // Ініціалізуємо
    HAL_TIM_PWM_Start(p->pwm_htim, p->pwm_channel);
    uint32_t ccr = (uint32_t)((0.5f * (p->out_min + p->out_max)) * (float)p->pwm_arr);
    __HAL_TIM_SET_COMPARE(p->pwm_htim, p->pwm_channel, ccr);
    g_last_output_pwm = ccr; // Зберігаємо початкове
    p->enabled = 1u;
}

void PID_SetTunings(float kp, float ki, float kd)
{
    // Ця функція викликається з main.c з ПОВНИМ набором коефіцієнтів
    CRITICAL_ENTER();
    s_pid.Kp = kp;
    s_pid.Ki = ki;
    s_pid.Kd = kd;
    CRITICAL_EXIT();
}

void PID_SetTarget(float hz)
{
    CRITICAL_ENTER();
    s_pid.target_hz = hz;
    s_pid.i_term = 0.0f; // Скидання інтегратора
    CRITICAL_EXIT();
}

void PID_SetEMA(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_pid.ema_alpha = alpha;
}

void PID_SetOutputLimits(float min_frac, float max_frac)
{
    if (min_frac > max_frac) { float t = min_frac; min_frac = max_frac; max_frac = t; }
    CLAMP(min_frac, 0.0f, 1.0f);
    CLAMP(max_frac, 0.0f, 1.0f);
    s_pid.out_min = min_frac;
    s_pid.out_max = max_frac;
}

void PID_Enable(uint8_t en)
{
    s_pid.enabled = en ? 1u : 0u;
}

void PID_Update(uint32_t hall_period_ticks)
{
    PIDCtrl_t *p = &s_pid;
    if (!p->enabled) return;
    if (hall_period_ticks < 2u) return;

    float dt = (float)hall_period_ticks / HALL_TICK_HZ;
    float freq = 1.0f / dt;

    if (p->freq_filt <= 0.0f) p->freq_filt = freq;
    else p->freq_filt = p->ema_alpha * freq + (1.0f - p->ema_alpha) * p->freq_filt;

    float err = p->target_hz - p->freq_filt;
    float duty_frac = pid_step(p, err, dt);

    g_last_output_pwm = (uint32_t)(duty_frac * (float)p->pwm_arr); // <-- ЗБЕРІГАЄМО ЗНАЧЕННЯ
    __HAL_TIM_SET_COMPARE(p->pwm_htim, p->pwm_channel, g_last_output_pwm);
}

/* --- ФУНКЦІЇ, ДОДАНІ ДЛЯ ТЕЛЕМЕТРІЇ --- */
void PID_GetTunings(float *Kp, float *Ki, float *Kd)
{
    if (Kp) *Kp = s_pid.Kp;
    if (Ki) *Ki = s_pid.Ki;
    if (Kd) *Kd = s_pid.Kd;
}

float PID_GetTargetFrequency(void)
{
    return s_pid.target_hz;
}

uint32_t PID_GetOutputPWM(void)
{
    return g_last_output_pwm; // Повертаємо останнє розраховане значення
}
/* ------------------------------------------- */
