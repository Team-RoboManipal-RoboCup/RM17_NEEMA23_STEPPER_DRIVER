/**
 * @file    stepper_control.c
 * @brief   Zero-Jitter, Glitch-Free Stepper Motor Controller with Synchronous
 * Profile TIM2 Channel 1 (PA0) provides pure hardware PWM pulse generation.
 *          Frequency profiling updates strictly at pulse boundaries (CNT = 0)
 * in TIM2 ISR, eliminating asynchronous register writes and mid-cycle
 * comparator glitches.
 */

#include "stepper_control.h"
#include "stepper_config.h"
#include "stepper_direction.h"
#include <math.h>

/* ==================== HARDWARE INITIALIZATION ==================== */

void Stepper_Init(StepperMotor *m) {
  /* Initialize motor configuration */
  m->pulses_per_rev = DEFAULT_PULSES_PER_REV;
  m->output_accel = DEFAULT_OUTPUT_ACCEL;
  m->output_speed = DEFAULT_OUTPUT_SPEED;
  m->current_position_deg = 0.0f; /* Starting position assumed as zero */
  m->moving = 0;
  m->state = STEPPER_IDLE;
  m->steps_done = 0;
  m->target_steps = 0;

  /* 1. GPIO Configuration:
   *    Primary Set:
   *      PA0  -> TIM2_CH1  (Primary PWM / STEP)
   *      PA1  -> DIR 1     (Primary DIRECTION)
   *      PA3  -> ENA 1     (Primary ENABLE)
   *    Secondary Set (Simultaneous):
   *      PB13 -> TIM1_CH1N (Secondary PWM / STEP)
   *      PB12 -> DIR 2     (Secondary DIRECTION)
   *      PA7  -> ENA 2     (Secondary ENABLE)
   *
   *    Speed set to MEDIUM (10 MHz) to eliminate high-frequency
   * transmission-line ringing, capacitive reflections, and ground bounce on
   * step pulses.
   */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* PA0: TIM2 CH1 Alternate Function Push-Pull (Primary PWM) */
  GPIO_InitStruct.Pin = STEP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(STEP_PORT, &GPIO_InitStruct);

  /* PA1: Primary DIR Pin */
  GPIO_InitStruct.Pin = DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DIR_PORT, &GPIO_InitStruct);

  /* PA3: Primary ENA Pin */
  GPIO_InitStruct.Pin = ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ENA_PORT, &GPIO_InitStruct);

  /* PA7: Secondary ENA Pin */
  GPIO_InitStruct.Pin = ENA2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ENA2_PORT, &GPIO_InitStruct);

  /* PB12: Secondary DIR Pin */
  GPIO_InitStruct.Pin = DIR2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DIR2_PORT, &GPIO_InitStruct);

  /* PB13: TIM1 CH1N Alternate Function Push-Pull (Secondary PWM) */
  GPIO_InitStruct.Pin = STEP2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(STEP2_PORT, &GPIO_InitStruct);

  /* Enable motor drivers and set initial direction on both outputs */
  MOTOR_ENABLE();
  DIR_CW();

  /* 2. TIM2 Configuration (Primary Hardware PWM Step Generator - Master):
   *    Clock = 72MHz. PSC = 71 -> 1 MHz timer tick (1 count = 1 µs).
   *    ARPE = 1 (Auto-Reload Preload Enable) ensures zero pulse-width jitter.
   *    Master Mode: MMS = 001 (Enable / CNT_EN signal output as TRGO).
   */
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM2->CR1 = 0;
  TIM2->CR2 = TIM_CR2_MMS_0; /* Master Mode: TRGO on Counter Enable (CEN) */
  TIM2->SMCR = 0;
  TIM2->PSC = 71;  /* 72 MHz / (71 + 1) = 1 MHz (1 µs tick) */
  TIM2->ARR = 999; /* Default 1000 µs = 1 kHz */
  TIM2->CCR1 = 0;  /* 0 duty cycle when idle */

  /* CCMR1: Channel 1 in PWM Mode 1 (OC1M = 110b), OC1PE = 1 (Preload Enable) */
  TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

  /* CCER: Keep Channel 1 output disabled until a move starts so PA0 stays LOW
   */
  TIM2->CCER = 0;

  /* CR1: Auto-reload preload enable */
  TIM2->CR1 = TIM_CR1_ARPE;

  /* Latch PSC and ARR immediately into active shadow registers */
  TIM2->EGR = TIM_EGR_UG;

  /* DIER: Enable Update Interrupt (fires strictly at completion of each step
   * pulse) */
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->SR = 0;

  /* 3. TIM1 Configuration (Secondary Hardware PWM on PB13 / TIM1_CH1N - Slave):
   *    Clock = 72MHz. PSC = 71 -> 1 MHz timer tick (1 count = 1 µs).
   *    Slave Mode: ITR1 (TIM2 TRGO) in Gated Mode (SMS = 101b).
   *    TIM1 starts & stops in hardware lockstep with TIM2.
   *    CH1N active low polarity (CC1NP = 1) yields identical positive pulse to
   * PA0. MOE = 1 in BDTR enables advanced timer complementary outputs.
   */
  __HAL_RCC_TIM1_CLK_ENABLE();

  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  /* Slave: TS = 001 (ITR1 -> TIM2), SMS = 101 (Gated Mode) */
  TIM1->SMCR = (1U << TIM_SMCR_TS_Pos) | (5U << TIM_SMCR_SMS_Pos);
  TIM1->PSC = 71;
  TIM1->ARR = 999;
  TIM1->CCR1 = 0;

  /* CCMR1: PWM Mode 1, Preload Enable */
  TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

  /* CCER: Keep output disabled until move starts, configure CC1NP = 1 for
   * correct polarity */
  TIM1->CCER = TIM_CCER_CC1NP;

  /* BDTR: Main Output Enable for TIM1 advanced timer */
  TIM1->BDTR = TIM_BDTR_MOE;

  /* CR1: Auto-reload preload enable + CEN (ready for TIM2 TRGO hardware gating)
   */
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  /* Latch PSC and ARR */
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = 0;

  /* NVIC configuration for TIM2 (Highest priority for cycle accuracy) */
  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ(TIM2_IRQn);
}

/* ==================== MOTION PLANNING & EXECUTION ==================== */

void Stepper_MoveTo(StepperMotor *m, float target_deg, uint32_t ppr,
                    float accel, float speed) {
  /* If already moving, ignore or stop first */
  if (m->moving) {
    return;
  }

  /* Apply microstepping PPR if specified */
  if (ppr > 0) {
    m->pulses_per_rev = ppr;
  }

  /* Apply output acceleration (rad/s^2) or fallback to default */
  if (accel > 0.0f) {
    m->output_accel = accel;
  } else {
    m->output_accel = DEFAULT_OUTPUT_ACCEL;
  }

  /* Apply output speed (rad/s) or fallback to default */
  if (speed > 0.0f) {
    m->output_speed = speed;
  } else {
    m->output_speed = DEFAULT_OUTPUT_SPEED;
  }

  /* Calculate delta angle in degrees at the output of the 1:10 gearbox */
  float delta_deg = target_deg - m->current_position_deg;

  /* =========================================================================
   * N-MOTION ROTARY MODULO 360 LOGIC CONTROL
   * =========================================================================
   * Inspired by N-Motion / industrial rotary axis positioning documentation:
   * On a 360-degree rotary axis, any orientation is cyclic with period 360°.
   * If the difference between current position and commanded position is any
   * integer multiple of 360 degrees:
   *   diff = k * 360.0 deg (where k = 0, +/-1, +/-2, ...)
   * the physical output shaft is ALREADY at the commanded orientation.
   * Examples:
   *   - Current = 0°,   Target = 360°  -> diff = 360°  (k = 1)  -> DO NOT MOVE.
   *   - Current = 0°,   Target = -360° -> diff = -360° (k = -1) -> DO NOT MOVE.
   *   - Current = 90°,  Target = -270° -> diff = -360° (k = -1) -> DO NOT MOVE.
   *   - Current = 90°,  Target = 450°  -> diff = 360°  (k = 1)  -> DO NOT MOVE.
   * =========================================================================
   */
#if (N_MOTION_SHORTEST_PATH == 1)
  /* Shortest-Path Modulo Mode:
   * Traverses the shortest circular arc (<= 180 deg) on the 360 deg circle
   */
  float k_turns = roundf(delta_deg / 360.0f);
  float modulo_diff = delta_deg - (k_turns * 360.0f);
  delta_deg = modulo_diff;
#endif

  /* Total output pulses per revolution: GEAR_RATIO (10) * motor PPR */
  uint32_t total_ppr_output = GEAR_RATIO * m->pulses_per_rev;

  /* Degrees per motor step at the gearbox output */
  m->degrees_per_step = 360.0f / (float)total_ppr_output;

  /* Compute total integer steps required for this move */
  int32_t steps = (int32_t)roundf(fabsf(delta_deg) / m->degrees_per_step);
  if (steps <= 0) {
    return;
  }

  /* Decide rotation direction using the isolated direction solver */
  m->direction = Stepper_DecideDirection(delta_deg);

  if (m->direction == 0) {
    DIR_CW();
    m->move_sign = 1.0f;
  } else {
    DIR_CCW();
    m->move_sign = -1.0f;
  }

  /* Ensure motor driver is enabled */
  MOTOR_ENABLE();

  /* CRITICAL DM860H TIMING:
   * Wait at least 10 µs setup time between DIR pin change and first step pulse.
   * (DM860H datasheet requires min 5.0 µs setup time).
   */
  delay_us(10);

  /* Convert kinematic parameters from Output (rad/s, rad/s^2) to Step Rates */
  float steps_per_rad = (float)total_ppr_output / (2.0f * PI_F);

  m->step_accel = m->output_accel * steps_per_rad;
  m->max_step_speed = m->output_speed * steps_per_rad;
  m->min_step_speed = MIN_STEP_FREQ;

  if (m->max_step_speed < m->min_step_speed) {
    m->max_step_speed = m->min_step_speed;
  }
  if (m->max_step_speed > MAX_STEP_FREQ) {
    m->max_step_speed = MAX_STEP_FREQ;
  }

  /* Compute trapezoidal acceleration / deceleration profile */
  /* s_accel = (v_max^2 - v_min^2) / (2 * a) */
  float s_accel = (m->max_step_speed * m->max_step_speed -
                   m->min_step_speed * m->min_step_speed) /
                  (2.0f * m->step_accel);

  if ((2.0f * s_accel) > (float)steps) {
    /* Triangle profile (cruise speed is not reached) */
    m->accel_steps = steps / 2;
    m->decel_start_step = steps - m->accel_steps;

    /* Recompute peak speed reached */
    float peak_sq = (m->min_step_speed * m->min_step_speed) +
                    (2.0f * m->step_accel * (float)m->accel_steps);
    if (peak_sq > 0.0f) {
      m->max_step_speed = sqrtf(peak_sq);
    }
  } else {
    /* Trapezoidal profile with cruise phase */
    m->accel_steps = (int32_t)s_accel;
    m->decel_start_step = steps - m->accel_steps;
  }

  /* Store move parameters */
  m->target_steps = steps;
  m->steps_done = 0;
  m->current_speed = m->min_step_speed;
  m->move_target_deg = target_deg;
  m->state = STEPPER_ACCEL;
  m->moving = 1;

  /* Set initial period for starting step rate (in microseconds) */
  uint32_t period_us = (uint32_t)(1000000.0f / m->min_step_speed);
  if (period_us < 10)
    period_us = 10;
  if (period_us > 65535)
    period_us = 65535;

  TIM2->ARR = period_us - 1;
  TIM2->CCR1 = period_us / 2;
  TIM1->ARR = period_us - 1;
  TIM1->CCR1 = period_us / 2;

  TIM2->CNT = 0;
  TIM1->CNT = 0;

  TIM2->EGR =
      TIM_EGR_UG; /* Latch preload into shadow registers simultaneously */
  TIM1->EGR = TIM_EGR_UG;
  TIM2->SR = 0; /* Clear update flag caused by UG */
  TIM1->SR = 0;

  /* Enable Channel outputs: PA0 (TIM2_CH1) and PB13 (TIM1_CH1N) */
  TIM2->CCER |= TIM_CCER_CC1E;
  TIM1->CCER |= TIM_CCER_CC1NE;

  /* Arm TIM1 slave and start TIM2 master (hardware TRGO starts both timers in
   * lockstep) */
  TIM1->CR1 |= TIM_CR1_CEN;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void Stepper_Stop(StepperMotor *m) {
  /* Immediately stop TIM2 hardware PWM and disable output */
  TIM2->CR1 &= ~TIM_CR1_CEN;
  TIM2->CCER &= ~TIM_CCER_CC1E;
  TIM2->CCR1 = 0;
  TIM2->SR = 0;

  /* Immediately stop TIM1 secondary hardware PWM and disable output */
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->CCER &= ~TIM_CCER_CC1NE;
  TIM1->CCR1 = 0;
  TIM1->SR = 0;

  m->moving = 0;
  m->state = STEPPER_IDLE;
}

float Stepper_GetPosition(StepperMotor *m) { return m->current_position_deg; }

void Stepper_SetHome(StepperMotor *m) {
  Stepper_Stop(m);
  m->current_position_deg = 0.0f;
}

/* ==================== SYNCHRONOUS PULSE CALLBACK ==================== */

/**
 * @brief  TIM2 Update ISR: Called strictly at counter overflow (CNT = 0).
 *         Updates motion profile synchronously between steps.
 *         DURING the pulse itself, NO timer register is ever modified,
 *         completely eliminating any mid-cycle comparator glitch!
 */
void Stepper_PulseCallback(StepperMotor *m) {
  if (!m->moving) {
    return;
  }

  m->steps_done++;

  /* Update real-time output shaft position */
  if (m->direction == 0) {
    m->current_position_deg += m->degrees_per_step;
  } else {
    m->current_position_deg -= m->degrees_per_step;
  }

  /* Check if target step count reached */
  if (m->steps_done >= m->target_steps) {
    /* Stop TIM2 hardware PWM immediately and disable output */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CCER &= ~TIM_CCER_CC1E;
    TIM2->CCR1 = 0;
    TIM2->SR = 0;

    /* Stop TIM1 secondary PWM immediately and disable output */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCER &= ~TIM_CCER_CC1NE;
    TIM1->CCR1 = 0;
    TIM1->SR = 0;

    m->moving = 0;
    m->state = STEPPER_IDLE;

    /* Snap to precise commanded target angle to prevent float rounding drift */
    m->current_position_deg = m->move_target_deg;
    return;
  }

  /* Synchronous step profile update:
   * dt is the actual duration of the current step in seconds.
   */
  float dt = 1.0f / m->current_speed;

  if (m->steps_done < m->accel_steps) {
    m->state = STEPPER_ACCEL;
    m->current_speed += m->step_accel * dt;
    if (m->current_speed > m->max_step_speed) {
      m->current_speed = m->max_step_speed;
    }
  } else if (m->steps_done >= m->decel_start_step) {
    m->state = STEPPER_DECEL;
    m->current_speed -= m->step_accel * dt;
    if (m->current_speed < m->min_step_speed) {
      m->current_speed = m->min_step_speed;
    }
  } else {
    m->state = STEPPER_CRUISE;
    m->current_speed = m->max_step_speed;
  }

  /* Calculate next step period in microseconds */
  uint32_t period_us = (uint32_t)(1000000.0f / m->current_speed);
  if (period_us < 10)
    period_us = 10;
  if (period_us > 65535)
    period_us = 65535;

  /* Preload ARR and CCR1 for the NEXT pulse on both timers simultaneously.
   * Written at CNT = 0, so the pulse runs 100% uninterrupted without any
   * glitches!
   */
  TIM2->ARR = period_us - 1;
  TIM2->CCR1 = period_us / 2;
  TIM1->ARR = period_us - 1;
  TIM1->CCR1 = period_us / 2;
}

/**
 * @brief  Kept for backward compatibility if TIM3 ISR is invoked.
 */
void Stepper_RampUpdate(StepperMotor *m) {
  /* Profile calculation is now handled synchronously in Stepper_PulseCallback
   */
  (void)m;
}
