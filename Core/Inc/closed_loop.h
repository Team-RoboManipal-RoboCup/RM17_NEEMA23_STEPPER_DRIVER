/**
 * @file    closed_loop.h
 * @brief   Closed-Loop PID Controller with AS5600 Multi-Turn Feedback for Stepper Motor
 *          Non-blocking, flag-driven architecture polled in main loop.
 */

#ifndef CLOSED_LOOP_H
#define CLOSED_LOOP_H

#include "stm32f1xx_hal.h"
#include "stepper_control.h"
#include "as5600.h"
#include <stdint.h>

/* ==================== CONFIGURATION CONSTANTS ==================== */
/* Static angle offset: difference between actual physical position of the
 * output shaft and calculated position from the magnetic encoder (in degrees).
 * Default: 0.0 degrees (can be tuned via serial command).
 */
#define DEFAULT_STATIC_ANGLE_OFFSET    0.0f

/* Default PID gains (appropriate for stepper position correction) */
#define DEFAULT_PID_KP                 1.00f   /* Direct proportional step recovery */
#define DEFAULT_PID_KI                 0.00f   /* Integral gain (prevents hunting when 0) */
#define DEFAULT_PID_KD                 0.05f   /* Derivative damping */
#define DEFAULT_PID_DEADBAND           0.05f   /* Deadband in degrees (~5.7 encoder counts) */
#define DEFAULT_PID_MAX_INTEGRAL       2.00f   /* Anti-windup limit in degrees */
#define DEFAULT_PID_MAX_OUTPUT         36.0f   /* Max correction move in single burst */

/* PID & Sensor update loop frequency (fixed interval timer) */
#define CL_TIMER_FREQ_HZ               100U    /* 100 Hz = 10 ms interval */
#define CL_SAMPLE_TIME_S               (1.0f / (float)CL_TIMER_FREQ_HZ)

/* Direction polarity of encoder (1 = normal, -1 = reversed) */
#define DEFAULT_ENCODER_POLARITY       1

/* ==================== TYPES ==================== */

typedef struct {
    float kp;
    float ki;
    float kd;
    float deadband;
    float max_integral;
    float max_output;
    float integral;
    float last_error;
    uint8_t enabled;
} PID_Controller;

typedef struct {
    /* Multi-turn encoder tracking */
    int64_t   accumulated_counts;
    uint16_t  last_raw_angle;
    int8_t    encoder_polarity;       /* +1 or -1 */
    uint8_t   sensor_connected;

    /* Kinematic positions (gearbox output shaft in degrees) */
    float     measured_output_deg;    /* Absolute position from encoder */
    float     static_angle_offset;    /* Calibration offset (degrees) */
    float     following_error_deg;    /* target - measured */
    float     last_correction_deg;

    /* Settling & stability */
    uint16_t  settle_delay_cycles;
    uint32_t  correction_count;

    /* PID Controller */
    PID_Controller pid;
} ClosedLoopState;

/* ==================== GLOBAL FLAG ==================== */
/* Flag set by TIM3 ISR at 100 Hz, polled non-blocking inside main loop */
extern volatile uint8_t g_cl_timer_flag;

/* ==================== PUBLIC API ==================== */

/**
 * @brief  Initializes AS5600, reads power-on position, sets initial motor position,
 *         and starts TIM3 fixed-interval flag generation at 100 Hz.
 * @param  m: Pointer to global StepperMotor instance.
 */
void ClosedLoop_Init(StepperMotor *m);

/**
 * @brief  Called non-blocking in main() while(1) whenever g_cl_timer_flag is set.
 *         Reads AS5600 over I2C, updates multi-turn angle, and executes PID
 *         missed-step correction when needed.
 * @param  m: Pointer to global StepperMotor instance.
 */
void ClosedLoop_Update(StepperMotor *m);

/* --- Configuration Getters and Setters (Accessible via Serial Commands) --- */
void  ClosedLoop_SetStaticOffset (float offset_deg, StepperMotor *m);
float ClosedLoop_GetStaticOffset (void);

void  ClosedLoop_SetPID          (float kp, float ki, float kd);
void  ClosedLoop_GetPID          (float *kp, float *ki, float *kd);

void  ClosedLoop_SetDeadband     (float deadband_deg);
float ClosedLoop_GetDeadband     (void);

void  ClosedLoop_SetEnabled      (uint8_t enable);
uint8_t ClosedLoop_IsEnabled     (void);

void  ClosedLoop_SetPolarity     (int8_t pol);
int8_t ClosedLoop_GetPolarity    (void);

float ClosedLoop_GetMeasuredDeg  (void);
float ClosedLoop_GetFollowingError(void);
uint16_t ClosedLoop_GetRawAngle  (void);
int64_t ClosedLoop_GetCounts     (void);
uint8_t ClosedLoop_IsSensorOnline(void);
void  ClosedLoop_ResetTracking   (StepperMotor *m);

#endif /* CLOSED_LOOP_H */
