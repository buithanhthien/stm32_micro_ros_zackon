/*
 * robot_params.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_ROBOT_PARAMS_H_
#define INC_ROBOT_PARAMS_H_

#include "pid.h"
// Encoder & cơ khí
#define ENCODER_PPR        500       // xung / vòng / kênh
#define QUAD_FACTOR        4         // TI12: x4
#define GEAR_RATIO         14.0f
#define TICKS_PER_REV      (ENCODER_PPR * QUAD_FACTOR * GEAR_RATIO) // 28000

#define WHEEL_DIAMETER_M   0.10f     // 10 cm
#define WHEEL_CIRC_M       (3.1415926f * WHEEL_DIAMETER_M)
#define WHEEL_BASE_M   0.292f

//sample time for each tasks
#define ODOMETRY_PERIOD_MS   5
#define MOTORCONTROL_PERIOD_MS 5
#define ENCODER_PERIOD_MS 5
#define UART_PERIOD_MS 50
#define DT_TIME 0.005f

//maximum speed and acceleration of each wheel
#define MAXSPEED 1.00f
#define V_MAX               1.00f     // max linear m/s  -- set this one similar to MAXSPEED
#define V_STEP              0.50f     // d-pad speed

#define MAXACC 2.5f

//ps2 controller constants
#define W_MAX               2.00f     // max angular rad/s (tune)
#define W_STEP              0.50f     // d-pad turn rate

#define PS2_TIMEOUT_MS      500
#define STICK_DEADBAND      12        // raw units (0..255), tune ~10-20

// Heading hold enabling thresholds
#define EPS_TURN_MPS      0.03f   // if |vR_ref - vL_ref| < this => "straight command"
#define EPS_MOVE_MPS      0.05f   // if avg speed below this => don't bother holding heading

// Heading controller gains (start small)
#define HEADING_KP        2.0f
#define HEADING_KD        0.1f

// Heading loop rate (outer loop slower than wheel PID)
#define HEADING_DT        0.02f   // 20 ms (50 Hz)
#define HEADING_DIV       4       // if wheel PID runs at 5 ms, 4 ticks => 20 ms

// Output limit for correction (rad/s). Prevents crazy commands.
#define OMEGA_CORR_MAX    2.0f

//PID controller variables
extern PID_TypeDef PosPID1, PosPID2;
extern PID_TypeDef SpeedPID1, SpeedPID2;

extern double CurPos1, CurSpeed1, PosPIDOut1, SpeedPIDOut1, DesiredPos1, DesiredSpeed1;
extern double CurPos2, CurSpeed2, PosPIDOut2, SpeedPIDOut2, DesiredPos2, DesiredSpeed2;

extern volatile double SetPos1, SetPos2, SetSpeed1, SetSpeed2;
extern double Spd1_cmdfromROS, Spd2_cmdfromROS;

extern double Kp1, Ki1, Kd1;
extern double Kp2, Ki2, Kd2;

extern double Kp_fromLCD1, Ki_fromLCD1, Kd_fromLCD1;
extern double Kp_fromLCD2, Ki_fromLCD2, Kd_fromLCD2;
extern uint8_t pid_update_pending;

extern int32_t PID_sampletime;

extern int32_t g_encCount1;
extern int32_t g_encCount2;
extern float g_speed1_mps;
extern float g_speed2_mps;
extern float g_revCount1;
extern float g_revCount2;

#endif /* INC_ROBOT_PARAMS_H_ */
