/**
 * @file    command_parser.c
 * @brief   USB CDC Command Parser for Stepper Motion Controller
 *
 * Supported Command Formats:
 *   SET_POS <angle> <microstepping>
 *   GET_POS         -> Query current output angle (absolute in degrees)
 *   <angle>, <ppr> [, <accel>] [, <speed>]
 *   G <angle>, <ppr> [, <accel>] [, <speed>]
 *   P or POS?       -> Query current output angle
 *   H or HOME       -> Set current position to 0.0 degrees
 *   S or STOP       -> Emergency stop
 *   ? or STATUS     -> Query motor status
 */

#include "command_parser.h"
#include "stepper_config.h"
#include "stm32f1xx_hal.h"
#include "usb_cdc.h"
#include "closed_loop.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define CMD_BUFFER_SIZE 128

static char cmd_buffer[CMD_BUFFER_SIZE];
static uint16_t cmd_index = 0;
static uint32_t last_byte_time = 0;

void CMD_Init(void) {
  cmd_index = 0;
  cmd_buffer[0] = '\0';
  last_byte_time = 0;
}

static void trim_whitespace(char *str) {
  char *end;
  while (isspace((unsigned char)*str))
    str++;
  if (*str == 0)
    return;
  end = str + strlen(str) - 1;
  while (end > str && isspace((unsigned char)*end))
    end--;
  end[1] = '\0';
}

void CMD_FormatFloat(char *out, size_t out_sz, float val, uint8_t decimals) {
  int32_t sign = (val < 0.0f) ? -1 : 1;
  float abs_val = fabsf(val);
  int32_t whole = (int32_t)abs_val;
  float scale = (decimals == 2) ? 100.0f : 1000.0f;
  int32_t max_frac = (decimals == 2) ? 100 : 1000;
  int32_t frac = (int32_t)roundf((abs_val - (float)whole) * scale);

  if (frac >= max_frac) {
    whole += 1;
    frac -= max_frac;
  }

  if (sign < 0 && (whole > 0 || frac > 0)) {
    if (decimals == 2) {
      snprintf(out, out_sz, "-%ld.%02ld", (long)whole, (long)frac);
    } else {
      snprintf(out, out_sz, "-%ld.%03ld", (long)whole, (long)frac);
    }
  } else {
    if (decimals == 2) {
      snprintf(out, out_sz, "%ld.%02ld", (long)whole, (long)frac);
    } else {
      snprintf(out, out_sz, "%ld.%03ld", (long)whole, (long)frac);
    }
  }
}

static void trim_float_str(char *str) {
  char *dot = strchr(str, '.');
  if (!dot)
    return;
  char *p = str + strlen(str) - 1;
  /* Trim trailing '0's but preserve at least one digit after the decimal point
   */
  while (p > (dot + 1) && *p == '0') {
    *p = '\0';
    p--;
  }
}

static void execute_command(char *line, StepperMotor *motor) {
  char reply[192];
  char pos_str[32];

  /* Trim leading and trailing whitespace */
  trim_whitespace(line);
  if (strlen(line) == 0) {
    return;
  }

  /* 0. Ping Command: "PING" */
  if (strcasecmp(line, "PING") == 0) {
    USB_CDC_Print("OK\r\n");
    return;
  }

  /* 1. Query Position: "GET_POS" -> outputs bare float string, e.g. "0.0\n" */
  if (strcasecmp(line, "GET_POS") == 0 || strcasecmp(line, "GET_POS?") == 0) {
    CMD_FormatFloat(pos_str, sizeof(pos_str), Stepper_GetPosition(motor), 3);
    trim_float_str(pos_str);
    snprintf(reply, sizeof(reply), "%s\n", pos_str);
    USB_CDC_Print(reply);
    return;
  }

  /* Query Position (Text Format): "P", "POS", or "POS?" */
  if (strcasecmp(line, "P") == 0 || strcasecmp(line, "POS") == 0 ||
      strcasecmp(line, "POS?") == 0) {
    CMD_FormatFloat(pos_str, sizeof(pos_str), Stepper_GetPosition(motor), 3);
    snprintf(reply, sizeof(reply), "POS: %s DEG\r\n", pos_str);
    USB_CDC_Print(reply);
    return;
  }

  /* 2. Set Home / Zero: "H" or "HOME" or "ZERO" */
  if (strcasecmp(line, "H") == 0 || strcasecmp(line, "HOME") == 0 ||
      strcasecmp(line, "ZERO") == 0) {
    Stepper_SetHome(motor);
    ClosedLoop_ResetTracking(motor);
    USB_CDC_Print("HOME: Position and Encoder reset to 0.0 DEG\r\n");
    return;
  }

  /* 3. Emergency Stop: "S" or "STOP" */
  if (strcasecmp(line, "S") == 0 || strcasecmp(line, "STOP") == 0) {
    Stepper_Stop(motor);
    CMD_FormatFloat(pos_str, sizeof(pos_str), Stepper_GetPosition(motor), 3);
    snprintf(reply, sizeof(reply), "STOP: Motor halted at %s DEG\r\n", pos_str);
    USB_CDC_Print(reply);
    return;
  }

  /* 4. Status Query: "?" or "STATUS" */
  if (strcmp(line, "?") == 0 || strcasecmp(line, "STATUS") == 0) {
    char enc_str[32], err_str[32];
    CMD_FormatFloat(pos_str, sizeof(pos_str), Stepper_GetPosition(motor), 3);
    CMD_FormatFloat(enc_str, sizeof(enc_str), ClosedLoop_GetMeasuredDeg(), 3);
    CMD_FormatFloat(err_str, sizeof(err_str), ClosedLoop_GetFollowingError(), 3);
    snprintf(reply, sizeof(reply),
             "STATUS: Pos=%s | Enc=%s | Err=%s DEG | Moving=%d | State=%d | PPR=%lu | CL=%d\r\n",
             pos_str, enc_str, err_str, motor->moving, motor->state,
             (unsigned long)motor->pulses_per_rev, ClosedLoop_IsEnabled());
    USB_CDC_Print(reply);
    return;
  }

  /* 4a. Static Angle Offset Query: "GET_OFFSET" or "OFFSET?" */
  if (strcasecmp(line, "GET_OFFSET") == 0 || strcasecmp(line, "OFFSET?") == 0) {
    char off_str[32];
    CMD_FormatFloat(off_str, sizeof(off_str), ClosedLoop_GetStaticOffset(), 3);
    snprintf(reply, sizeof(reply), "OFFSET: %s DEG\r\n", off_str);
    USB_CDC_Print(reply);
    return;
  }

  /* 4b. Static Angle Offset Set: "SET_OFFSET <val>" or "OFFSET <val>" */
  if (strncasecmp(line, "SET_OFFSET", 10) == 0 ||
      (strncasecmp(line, "OFFSET", 6) == 0 && (line[6] == ' ' || line[6] == '='))) {
    char *v_ptr = (strncasecmp(line, "SET_OFFSET", 10) == 0) ? (line + 10) : (line + 6);
    while (isspace((unsigned char)*v_ptr) || *v_ptr == '=') v_ptr++;
    char *endp;
    float new_offset = strtof(v_ptr, &endp);
    if (endp != v_ptr) {
      ClosedLoop_SetStaticOffset(new_offset, motor);
      char off_str[32];
      CMD_FormatFloat(off_str, sizeof(off_str), new_offset, 3);
      snprintf(reply, sizeof(reply), "OK: Static Offset set to %s DEG\r\n", off_str);
      USB_CDC_Print(reply);
    } else {
      USB_CDC_Print("ERR: Expected SET_OFFSET <degrees>\r\n");
    }
    return;
  }

  /* 4c. PID Gains Query: "GET_PID" or "PID?" */
  if (strcasecmp(line, "GET_PID") == 0 || strcasecmp(line, "PID?") == 0) {
    float kp, ki, kd;
    ClosedLoop_GetPID(&kp, &ki, &kd);
    char kp_s[32], ki_s[32], kd_s[32], db_s[32];
    CMD_FormatFloat(kp_s, sizeof(kp_s), kp, 3);
    CMD_FormatFloat(ki_s, sizeof(ki_s), ki, 3);
    CMD_FormatFloat(kd_s, sizeof(kd_s), kd, 3);
    CMD_FormatFloat(db_s, sizeof(db_s), ClosedLoop_GetDeadband(), 3);
    snprintf(reply, sizeof(reply),
             "PID: Kp=%s Ki=%s Kd=%s Deadband=%s DEG (CL=%d)\r\n",
             kp_s, ki_s, kd_s, db_s, ClosedLoop_IsEnabled());
    USB_CDC_Print(reply);
    return;
  }

  /* 4d. PID Gains Set: "SET_PID <kp> <ki> <kd>" or "PID <kp> <ki> <kd>" */
  if (strncasecmp(line, "SET_PID", 7) == 0 ||
      (strncasecmp(line, "PID", 3) == 0 && (line[3] == ' ' || line[3] == '='))) {
    char *p_ptr = (strncasecmp(line, "SET_PID", 7) == 0) ? (line + 7) : (line + 3);
    while (isspace((unsigned char)*p_ptr) || *p_ptr == '=') p_ptr++;
    char *t1 = strtok(p_ptr, " ,\t");
    char *t2 = strtok(NULL, " ,\t");
    char *t3 = strtok(NULL, " ,\t");
    if (t1 && t2 && t3) {
      char *endp;
      float kp = strtof(t1, &endp);
      float ki = strtof(t2, &endp);
      float kd = strtof(t3, &endp);
      ClosedLoop_SetPID(kp, ki, kd);
      char kp_s[32], ki_s[32], kd_s[32];
      CMD_FormatFloat(kp_s, sizeof(kp_s), kp, 3);
      CMD_FormatFloat(ki_s, sizeof(ki_s), ki, 3);
      CMD_FormatFloat(kd_s, sizeof(kd_s), kd, 3);
      snprintf(reply, sizeof(reply), "OK: PID gains set to Kp=%s Ki=%s Kd=%s\r\n",
               kp_s, ki_s, kd_s);
      USB_CDC_Print(reply);
    } else {
      USB_CDC_Print("ERR: Expected SET_PID <kp> <ki> <kd>\r\n");
    }
    return;
  }

  /* 4e. Deadband Query / Set: "DEADBAND" or "SET_DEADBAND <val>" */
  if (strcasecmp(line, "GET_DEADBAND") == 0 || strcasecmp(line, "DEADBAND?") == 0) {
    char db_s[32];
    CMD_FormatFloat(db_s, sizeof(db_s), ClosedLoop_GetDeadband(), 3);
    snprintf(reply, sizeof(reply), "DEADBAND: %s DEG\r\n", db_s);
    USB_CDC_Print(reply);
    return;
  }
  if (strncasecmp(line, "SET_DEADBAND", 12) == 0 ||
      (strncasecmp(line, "DEADBAND", 8) == 0 && (line[8] == ' ' || line[8] == '='))) {
    char *d_ptr = (strncasecmp(line, "SET_DEADBAND", 12) == 0) ? (line + 12) : (line + 8);
    while (isspace((unsigned char)*d_ptr) || *d_ptr == '=') d_ptr++;
    char *endp;
    float db = strtof(d_ptr, &endp);
    if (endp != d_ptr && db >= 0.0f) {
      ClosedLoop_SetDeadband(db);
      char db_s[32];
      CMD_FormatFloat(db_s, sizeof(db_s), db, 3);
      snprintf(reply, sizeof(reply), "OK: Deadband set to %s DEG\r\n", db_s);
      USB_CDC_Print(reply);
    } else {
      USB_CDC_Print("ERR: Expected SET_DEADBAND <degrees>\r\n");
    }
    return;
  }

  /* 4f. Closed Loop Enable/Disable: "CL_EN <1|0>" or "CLOSED_LOOP <1|0>" */
  if (strncasecmp(line, "CL_EN", 5) == 0 || strncasecmp(line, "CLOSED_LOOP", 11) == 0) {
    char *c_ptr = (strncasecmp(line, "CL_EN", 5) == 0) ? (line + 5) : (line + 11);
    while (isspace((unsigned char)*c_ptr) || *c_ptr == '=') c_ptr++;
    if (*c_ptr == '1' || strcasecmp(c_ptr, "ON") == 0 || strcasecmp(c_ptr, "ENABLE") == 0) {
      ClosedLoop_SetEnabled(1);
      USB_CDC_Print("OK: Closed-loop control ENABLED\r\n");
    } else if (*c_ptr == '0' || strcasecmp(c_ptr, "OFF") == 0 || strcasecmp(c_ptr, "DISABLE") == 0) {
      ClosedLoop_SetEnabled(0);
      USB_CDC_Print("OK: Closed-loop control DISABLED (Open-loop mode)\r\n");
    } else {
      snprintf(reply, sizeof(reply), "CL: Currently %s\r\n",
               ClosedLoop_IsEnabled() ? "ENABLED" : "DISABLED");
      USB_CDC_Print(reply);
    }
    return;
  }

  /* 4g. Encoder Diagnostics: "ENC" or "GET_ENC" or "ENC?" */
  if (strcasecmp(line, "ENC") == 0 || strcasecmp(line, "GET_ENC") == 0 ||
      strcasecmp(line, "ENC?") == 0) {
    char enc_s[32], err_s[32], off_s[32];
    CMD_FormatFloat(enc_s, sizeof(enc_s), ClosedLoop_GetMeasuredDeg(), 3);
    CMD_FormatFloat(err_s, sizeof(err_s), ClosedLoop_GetFollowingError(), 3);
    CMD_FormatFloat(off_s, sizeof(off_s), ClosedLoop_GetStaticOffset(), 3);
    snprintf(reply, sizeof(reply),
             "ENC: Raw=%u | Counts=%lld | Output=%s DEG | Err=%s DEG | Offset=%s DEG | Conn=%d | Pol=%d\r\n",
             ClosedLoop_GetRawAngle(), (long long)ClosedLoop_GetCounts(),
             enc_s, err_s, off_s, ClosedLoop_IsSensorOnline(), ClosedLoop_GetPolarity());
    USB_CDC_Print(reply);
    return;
  }

  /* 4h. Encoder Polarity: "SET_ENC_DIR <1|-1>" or "ENC_DIR <1|-1>" */
  if (strncasecmp(line, "SET_ENC_DIR", 11) == 0 || strncasecmp(line, "ENC_DIR", 7) == 0) {
    char *e_ptr = (strncasecmp(line, "SET_ENC_DIR", 11) == 0) ? (line + 11) : (line + 7);
    while (isspace((unsigned char)*e_ptr) || *e_ptr == '=') e_ptr++;
    int8_t pol = (int8_t)atoi(e_ptr);
    if (pol == 1 || pol == -1) {
      ClosedLoop_SetPolarity(pol);
      snprintf(reply, sizeof(reply), "OK: Encoder polarity set to %d\r\n", pol);
      USB_CDC_Print(reply);
    } else {
      USB_CDC_Print("ERR: Expected SET_ENC_DIR 1 or -1\r\n");
    }
    return;
  }

  /* 5. Move Command:
   *    Format: SET_POS <angle> <microstepping>
   *            [G/MOVE] <angle>, <ppr> [, <accel>] [, <speed>]
   *    Optional fields can be omitted or left blank.
   */
  char *ptr = line;

  /* Skip optional 'SET_POS', 'MOVE', or 'G' prefix */
  if (strncasecmp(ptr, "SET_POS", 7) == 0) {
    ptr += 7;
  } else if (strncasecmp(ptr, "MOVE", 4) == 0) {
    ptr += 4;
  } else if (*ptr == 'G' || *ptr == 'g') {
    ptr++;
  }
  while (isspace((unsigned char)*ptr))
    ptr++;

  /* If motor is currently executing a move, reject new move */
  if (motor->moving) {
    USB_CDC_Print("BUSY: Motor is currently moving\r\n");
    return;
  }

  /* Tokenize by commas or spaces */
  char *token1 = NULL;
  char *token2 = NULL;
  char *token3 = NULL;
  char *token4 = NULL;

  /* Check if line contains commas */
  if (strchr(ptr, ',') != NULL) {
    token1 = ptr;
    char *c1 = strchr(token1, ',');
    if (c1) {
      *c1 = '\0';
      token2 = c1 + 1;
      char *c2 = strchr(token2, ',');
      if (c2) {
        *c2 = '\0';
        token3 = c2 + 1;
        char *c3 = strchr(token3, ',');
        if (c3) {
          *c3 = '\0';
          token4 = c3 + 1;
        }
      }
    }
  } else {
    /* Space-separated tokens (e.g. SET_POS <angle> <microstepping>) */
    token1 = strtok(ptr, " \t");
    token2 = strtok(NULL, " \t");
    token3 = strtok(NULL, " \t");
    token4 = strtok(NULL, " \t");
  }

  if (!token1 || !token2) {
    USB_CDC_Print(
        "ERR: Expected SET_POS <angle> <microstepping> or <angle>, <ppr>\r\n");
    return;
  }

  trim_whitespace(token1);
  trim_whitespace(token2);
  if (token3)
    trim_whitespace(token3);
  if (token4)
    trim_whitespace(token4);

  /* Mandatory parameters: Angle (deg) and Microstepping PPR */
  char *endptr;
  float target_angle = strtof(token1, &endptr);
  if (endptr == token1) {
    USB_CDC_Print("ERR: Invalid angle value\r\n");
    return;
  }

  uint32_t ppr = (uint32_t)strtoul(token2, &endptr, 10);
  if (endptr == token2 || ppr == 0) {
    USB_CDC_Print("ERR: Invalid PPR value (must be > 0)\r\n");
    return;
  }

  /* Optional parameters: Output Accel (rad/s^2) and Output Speed (rad/s) */
  float accel = DEFAULT_OUTPUT_ACCEL;
  if (token3 && strlen(token3) > 0) {
    float parsed_accel = strtof(token3, &endptr);
    if (endptr != token3 && parsed_accel > 0.0f) {
      accel = parsed_accel;
    }
  }

  float speed = DEFAULT_OUTPUT_SPEED;
  if (token4 && strlen(token4) > 0) {
    float parsed_speed = strtof(token4, &endptr);
    if (endptr != token4 && parsed_speed > 0.0f) {
      speed = parsed_speed;
    }
  }

  /* Execute the move */
  Stepper_MoveTo(motor, target_angle, ppr, accel, speed);

  /* If move was suppressed (e.g. integer multiple of 360 deg), report
   * in-position */
  if (!motor->moving) {
    CMD_FormatFloat(pos_str, sizeof(pos_str), Stepper_GetPosition(motor), 3);
    snprintf(
        reply, sizeof(reply),
        "OK: Already at target position (%s DEG | Modulo 360 - No move)\r\n",
        pos_str);
    USB_CDC_Print(reply);
  }

  else {
    char angle_str[32], accel_str[32], speed_str[32];
    CMD_FormatFloat(angle_str, sizeof(angle_str), target_angle, 3);
    CMD_FormatFloat(accel_str, sizeof(accel_str), accel, 2);
    CMD_FormatFloat(speed_str, sizeof(speed_str), speed, 2);
    snprintf(
        reply, sizeof(reply),
        "OK: Moving to %s DEG | PPR=%lu | Accel=%s rad/s2 | Speed=%s rad/s\r\n",
        angle_str, (unsigned long)ppr, accel_str, speed_str);
    USB_CDC_Print(reply);
  }
}

void CMD_ProcessByte(uint8_t byte, StepperMotor *motor) {
  last_byte_time = HAL_GetTick();

  /* End of command line */
  if (byte == '\r' || byte == '\n') {
    if (cmd_index > 0) {
      cmd_buffer[cmd_index] = '\0';
      execute_command(cmd_buffer, motor);
      cmd_index = 0;
    }
  } else if (byte == 0x08 || byte == 0x7F) {
    /* Backspace support */
    if (cmd_index > 0) {
      cmd_index--;
    }
  } else {
    /* Append character to line buffer */
    if (cmd_index < (CMD_BUFFER_SIZE - 1)) {
      cmd_buffer[cmd_index++] = (char)byte;
    }
  }
}

void CMD_Poll(StepperMotor *motor) {
  /* If characters have been in the buffer for > 250ms without \r or \n,
   * auto-execute */
  if (cmd_index > 0 && (HAL_GetTick() - last_byte_time) > 250) {
    cmd_buffer[cmd_index] = '\0';
    execute_command(cmd_buffer, motor);
    cmd_index = 0;
  }
}
