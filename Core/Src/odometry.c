/*
 * odometry.c
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */
#include "odometry.h"
#include "encoder.h"
#include "robot_params.h"
#include <math.h>

// Nếu encoder 1/2 bị đảo chiều thì chỉnh SIGN
#define LEFT_SIGN      (+1.0f)
#define RIGHT_SIGN     (+1.0f)

// ==== BIẾN TRẠNG THÁI ====
volatile float odom_x = 0.0f;
volatile float odom_y = 0.0f;
volatile float odom_theta = 0.0f;

volatile float odom_vx = 0.0f;   // m/s  (base forward speed)
volatile float odom_wz = 0.0f;   // rad/s (yaw rate)

// Optional wheel speeds:
volatile float odom_vL = 0.0f;   // m/s
volatile float odom_vR = 0.0f;   // m/s

// Lưu encoder lần trước
static int32_t prevL = 0;
static int32_t prevR = 0;

static float wrapAngle(float a) {
    const float PI = 3.1415926f;
    const float TWO_PI = 2.0f * PI;

    while (a > PI)   a -= TWO_PI;
    while (a < -PI)  a += TWO_PI;
    return a;
}

void Odometry_Init(void)
{
    prevL = Encoder_GetCount(ENCODER_2);
    prevR = Encoder_GetCount(ENCODER_1);

    odom_x = 0.0f;
    odom_y = 0.0f;
    odom_theta = 0.0f;
}

void Odometry_Update(float dt)
{
    int32_t nowL = Encoder_GetCount(ENCODER_2);
    int32_t nowR = Encoder_GetCount(ENCODER_1);

    int32_t dTicksL = nowL - prevL;
    int32_t dTicksR = nowR - prevR;

    prevL = nowL;
    prevR = nowR;

    // distance each wheel traveled this cycle (meters)
    float dL = LEFT_SIGN  * ((float)dTicksL / (float)TICKS_PER_REV) * WHEEL_CIRC_M;
    float dR = RIGHT_SIGN * ((float)dTicksR / (float)TICKS_PER_REV) * WHEEL_CIRC_M;

    float dS  = 0.5f * (dL + dR);
    float dTh = (dR - dL) / WHEEL_BASE_M;

    float mid = odom_theta + 0.5f * dTh;

    odom_x     += dS * cosf(mid);
    odom_y     += dS * sinf(mid);
    odom_theta  = wrapAngle(odom_theta + dTh);

    // -----------------------------
    // TWIST (velocity) computation
    // -----------------------------
    if (dt > 1e-6f) {
        odom_vx = dS / dt;     // m/s
        odom_wz = dTh / dt;    // rad/s

        // Optional per-wheel speeds:
        odom_vL = dL / dt;
        odom_vR = dR / dt;
    } else {
        odom_vx = 0.0f;
        odom_wz = 0.0f;
        odom_vL = 0.0f;
        odom_vR = 0.0f;
    }
}

