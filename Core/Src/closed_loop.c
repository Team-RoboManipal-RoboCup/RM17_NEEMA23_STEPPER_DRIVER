/**
 * @file    closed_loop.c
 * @brief   Closed-Loop PID Controller with AS5600 Multi-Turn Feedback for Stepper Motor
 *          Non-blocking, flag-driven architecture polled in main loop.
 */

#include "closed_loop.h"
#include "stepper_config.h"
#include <math.h>

/* Global Flag set by TIM3 ISR at 100 Hz */
volatile uint8_t g_cl_timer_flag = 0;

/* Internal Closed Loop State Instance */
static ClosedLoopState g_cl;

/* ==================== HARDWARE TIMER INITIALIZATION ==================== */

/**
 * @brief  Initializes TIM3 as a 100 Hz (10 ms) periodic timer.
 *         TIM3 generates an update interrupt purely to set g_cl_timer_flag.
 *         No heavy processing or I2C calls occur inside the ISR!
 */
static void ClosedLoop_TimerInit(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->SMCR = 0;

    /* APB1 Timer Clock = 72 MHz:
     * PSC = 7199 -> Counter frequency = 72,000,000 / 7200 = 10,000 Hz (100 µs tick).
     * ARR = 99   -> Overflow period = (99 + 1) * 100 µs = 10 ms (100 Hz).
     */
    TIM3->PSC = 7199;
    TIM3->ARR = 99;

    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->DIER = TIM_DIER_UIE;

    /* Priority 3 (Lower than TIM2 step generator priority 1) */
    NVIC_SetPriority(TIM3_IRQn, 3);
    NVIC_EnableIRQ(TIM3_IRQn);

    /* Start TIM3 counter */
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* ==================== PUBLIC API IMPLEMENTATION ==================== */

void ClosedLoop_Init(StepperMotor *m) {
    /* 1. Initialize State Variables */
    g_cl.static_angle_offset = DEFAULT_STATIC_ANGLE_OFFSET;
    g_cl.encoder_polarity = DEFAULT_ENCODER_POLARITY;
    g_cl.sensor_connected = 0;
    g_cl.accumulated_counts = 0;
    g_cl.last_raw_angle = 0;
    g_cl.measured_output_deg = 0.0f;
    g_cl.following_error_deg = 0.0f;
    g_cl.last_correction_deg = 0.0f;
    g_cl.settle_delay_cycles = 5;
    g_cl.correction_count = 0;

    /* 2. Initialize PID Controller with default parameters */
    g_cl.pid.kp = DEFAULT_PID_KP;
    g_cl.pid.ki = DEFAULT_PID_KI;
    g_cl.pid.kd = DEFAULT_PID_KD;
    g_cl.pid.deadband = DEFAULT_PID_DEADBAND;
    g_cl.pid.max_integral = DEFAULT_PID_MAX_INTEGRAL;
    g_cl.pid.max_output = DEFAULT_PID_MAX_OUTPUT;
    g_cl.pid.integral = 0.0f;
    g_cl.pid.last_error = 0.0f;
    g_cl.pid.enabled = 1; /* Closed-loop enabled by default */

    /* 3. Initialize AS5600 hardware on PB6 (SCL) and PB7 (SDA) */
    uint8_t err = AS5600_Init();
    if (err == AS5600_OK) {
        g_cl.sensor_connected = 1;

        /* Take current position from sensor to know where it is at power-up */
        uint16_t raw0 = 0;
        if (AS5600_ReadRawAngle(&raw0) == AS5600_OK) {
            g_cl.last_raw_angle = raw0;
            g_cl.accumulated_counts = (int64_t)raw0;

            /* Conversion for 1:10 gearbox:
             * 1 motor turn = 4096 counts = 360 deg motor = 36 deg output.
             * Output angle = (raw0 / 4096) * (360 / GEAR_RATIO) - offset
             */
            float deg_per_count = 360.0f / (4096.0f * (float)GEAR_RATIO);
            float initial_output_deg = ((float)raw0 * deg_per_count) - g_cl.static_angle_offset;

            g_cl.measured_output_deg = initial_output_deg;

            /* Sync motor's logical coordinate system to real absolute physical position */
            m->current_position_deg = initial_output_deg;
            m->move_target_deg = initial_output_deg;
        }
    } else {
        g_cl.sensor_connected = 0;
    }

    /* 4. Start 100 Hz fixed-interval non-blocking timer */
    ClosedLoop_TimerInit();
}

void ClosedLoop_Update(StepperMotor *m) {
    /* 1. Read Raw Angle from AS5600 via I2C */
    uint16_t raw = 0;
    uint8_t res = AS5600_ReadRawAngle(&raw);

    if (res != AS5600_OK) {
        g_cl.sensor_connected = 0;
        return;
    }
    g_cl.sensor_connected = 1;

    /* 2. Multi-turn tracking unwrapper */
    int32_t diff = (int32_t)raw - (int32_t)g_cl.last_raw_angle;
    if (diff > 2048) {
        diff -= 4096; /* Wrapped backward */
    } else if (diff < -2048) {
        diff += 4096; /* Wrapped forward */
    }

    /* Apply encoder direction polarity */
    if (g_cl.encoder_polarity < 0) {
        diff = -diff;
    }

    g_cl.accumulated_counts += (int64_t)diff;
    g_cl.last_raw_angle = raw;

    /* 3. Compute absolute position in degrees at the output shaft of the 1:10 gearbox */
    float deg_per_count = 360.0f / (4096.0f * (float)GEAR_RATIO);
    g_cl.measured_output_deg = ((float)g_cl.accumulated_counts * deg_per_count) - g_cl.static_angle_offset;

    /* 4. Tracking and Closed-Loop PID Logic */
    if (m->moving) {
        /* Motor is actively stepping: compute following error for diagnostics */
        g_cl.following_error_deg = m->current_position_deg - g_cl.measured_output_deg;
        g_cl.settle_delay_cycles = 5; /* Reset settling timer while moving */
        g_cl.pid.integral = 0.0f;     /* Clear integrator during dynamic trajectory */
        g_cl.pid.last_error = g_cl.following_error_deg;
        return;
    }

    /* Motor is stationary (Idle or holding position):
     * Error = Command target position - Actual measured encoder position
     */
    float error = m->move_target_deg - g_cl.measured_output_deg;
    g_cl.following_error_deg = error;

    /* Wait for mechanical vibrations / rotor ringing to settle after a move */
    if (g_cl.settle_delay_cycles > 0) {
        g_cl.settle_delay_cycles--;
        return;
    }

    /* Check if closed loop is enabled */
    if (!g_cl.pid.enabled) {
        return;
    }

    /* Deadband check to eliminate jitter/hunting on encoder noise */
    if (fabsf(error) <= g_cl.pid.deadband) {
        g_cl.pid.integral = 0.0f;
        g_cl.pid.last_error = 0.0f;
        return;
    }

    /* 5. PID Calculation for Missed Step Correction */
    float P = g_cl.pid.kp * error;

    g_cl.pid.integral += g_cl.pid.ki * error * CL_SAMPLE_TIME_S;
    if (g_cl.pid.integral > g_cl.pid.max_integral) {
        g_cl.pid.integral = g_cl.pid.max_integral;
    } else if (g_cl.pid.integral < -g_cl.pid.max_integral) {
        g_cl.pid.integral = -g_cl.pid.max_integral;
    }

    float D = g_cl.pid.kd * (error - g_cl.pid.last_error) / CL_SAMPLE_TIME_S;
    g_cl.pid.last_error = error;

    float u = P + g_cl.pid.integral + D;

    /* Limit maximum correction magnitude in a single step */
    if (u > g_cl.pid.max_output) {
        u = g_cl.pid.max_output;
    } else if (u < -g_cl.pid.max_output) {
        u = -g_cl.pid.max_output;
    }

    /* Check minimum single microstep threshold before commanding motion */
    uint32_t total_ppr = GEAR_RATIO * m->pulses_per_rev;
    float deg_per_step = 360.0f / (float)total_ppr;

    if (fabsf(u) >= (0.8f * deg_per_step)) {
        /* Sync motor open-loop position tracker to actual encoder measured position */
        m->current_position_deg = g_cl.measured_output_deg;

        /* Calculate corrective destination angle */
        float corr_target = g_cl.measured_output_deg + u;
        g_cl.last_correction_deg = u;
        g_cl.correction_count++;

        /* Trigger corrective move using identical pulse generation method */
        Stepper_MoveTo(m, corr_target, m->pulses_per_rev, m->output_accel, m->output_speed);

        /* Set holdoff cycles to allow correction move to start smoothly */
        g_cl.settle_delay_cycles = 5;
    }
}

/* ==================== CONFIGURATION GETTERS & SETTERS ==================== */

void ClosedLoop_SetStaticOffset(float offset_deg, StepperMotor *m) {
    g_cl.static_angle_offset = offset_deg;
    float deg_per_count = 360.0f / (4096.0f * (float)GEAR_RATIO);
    g_cl.measured_output_deg = ((float)g_cl.accumulated_counts * deg_per_count) - g_cl.static_angle_offset;
    if (m && !m->moving) {
        m->current_position_deg = g_cl.measured_output_deg;
        m->move_target_deg = g_cl.measured_output_deg;
    }
}

float ClosedLoop_GetStaticOffset(void) {
    return g_cl.static_angle_offset;
}

void ClosedLoop_SetPID(float kp, float ki, float kd) {
    g_cl.pid.kp = kp;
    g_cl.pid.ki = ki;
    g_cl.pid.kd = kd;
    g_cl.pid.integral = 0.0f;
    g_cl.pid.last_error = 0.0f;
}

void ClosedLoop_GetPID(float *kp, float *ki, float *kd) {
    if (kp) *kp = g_cl.pid.kp;
    if (ki) *ki = g_cl.pid.ki;
    if (kd) *kd = g_cl.pid.kd;
}

void ClosedLoop_SetDeadband(float deadband_deg) {
    if (deadband_deg >= 0.0f) {
        g_cl.pid.deadband = deadband_deg;
    }
}

float ClosedLoop_GetDeadband(void) {
    return g_cl.pid.deadband;
}

void ClosedLoop_SetEnabled(uint8_t enable) {
    g_cl.pid.enabled = enable ? 1U : 0U;
    g_cl.pid.integral = 0.0f;
    g_cl.pid.last_error = 0.0f;
}

uint8_t ClosedLoop_IsEnabled(void) {
    return g_cl.pid.enabled;
}

void ClosedLoop_SetPolarity(int8_t pol) {
    g_cl.encoder_polarity = (pol < 0) ? -1 : 1;
}

int8_t ClosedLoop_GetPolarity(void) {
    return g_cl.encoder_polarity;
}

float ClosedLoop_GetMeasuredDeg(void) {
    return g_cl.measured_output_deg;
}

float ClosedLoop_GetFollowingError(void) {
    return g_cl.following_error_deg;
}

uint16_t ClosedLoop_GetRawAngle(void) {
    return g_cl.last_raw_angle;
}

int64_t ClosedLoop_GetCounts(void) {
    return g_cl.accumulated_counts;
}

uint8_t ClosedLoop_IsSensorOnline(void) {
    return g_cl.sensor_connected;
}

void ClosedLoop_ResetTracking(StepperMotor *m) {
    uint16_t raw = 0;
    if (AS5600_ReadRawAngle(&raw) == AS5600_OK) {
        g_cl.last_raw_angle = raw;
        g_cl.accumulated_counts = (int64_t)raw;
        float deg_per_count = 360.0f / (4096.0f * (float)GEAR_RATIO);
        g_cl.measured_output_deg = ((float)raw * deg_per_count) - g_cl.static_angle_offset;
        if (m) {
            m->current_position_deg = g_cl.measured_output_deg;
            m->move_target_deg = g_cl.measured_output_deg;
        }
    }
}
