#!/usr/bin/env python3
"""Compile actual control functions with a deterministic host task scheduler.
HAL/RTOS/PID are mocked: these tests do not validate physical PID response.
Run: python3 tests/test_motor_control.py
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
src = (ROOT / 'Core/Src/freertos.c').read_text()


def function(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


preamble = r'''
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "robot_params.h"
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1U
#define configTICK_RATE_HZ 1000U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define __DMB() ((void)0)
#define MP_MODE_POSITION 1
#define MOTOR_1 0
#define MOTOR_2 1
#define xSemaphoreTake(a,b) ((void)0)
#define xSemaphoreGive(a) ((void)0)
#define PID_SetTunings(...) ((void)0)
#define MotorData_Publish(...) ((void)0)
#define Motor_SetSpeed(...) ((void)0)
typedef uint32_t TickType_t;
static uint32_t tick, iterations, limit;
static jmp_buf finished, teleop_finished;
static int teleop_once;
static int scenario, modCtrl, pid_mutex;
static uint32_t stuck_ms, g_heading_div_ctr;
static uint8_t g_heading_hold_en;
static float g_theta, g_theta_ref, g_e_theta_prev;
static float g_heading_omega_corr, g_heading_elapsed_s;
static uint32_t dbg_cmd_cb_count, last_cmdvel_ms, dbg_cmd_gap_ms;
static uint32_t dbg_cmdvel_cb_cnt, dbg_last_cmdvel_ms;
typedef struct { double x, y, z; } Vector3;
typedef struct { Vector3 linear, angular; } geometry_msgs__msg__Twist;
static uint32_t dbg_cmd_gap_max_ms, dbg_cmd_gap_over_500ms;
static uint32_t num_timeout_ROScmd, autonomous_MODE;
static float dbg_last_cmd_v, dbg_last_cmd_w;
static uint32_t xTaskGetTickCount(void) { return tick; }
static uint8_t PID_Compute(PID_TypeDef *pid) { (void)pid; return 1; }
static void vTaskDelayUntil(TickType_t *last, TickType_t period);
static void osDelay(uint32_t ms);
static uint32_t age_ros_check;
#define CMDVEL_TIMEOUT_MS 500U
#define osKernelGetTickCount xTaskGetTickCount
#define millis_void xTaskGetTickCount
#define osOK 0
#define osMutexAcquire(...) 0
#define osMutexRelease(...) ((void)0)
enum { PSB_L1=1, PSB_UP=2, PSB_DOWN=4, PSB_LEFT=8, PSB_RIGHT=16 };
typedef struct { uint32_t tick; uint16_t buttons; uint8_t ok, rx, ry; } PS2X_State;
static PS2X_State g_ps2_state;
static int is_pressed(uint16_t b, uint16_t m) { return !!(b&m); }
void Apply_Robot_RampingDt(double *, double, double);
'''
mailbox = src[src.index('static volatile double motor_target1'):src.index('////Live debug variables')]
functions = '\n'.join(function(src, signature) for signature in (
    'static inline float clampf(', 'static inline float apply_deadband_and_min(',
    'static void cmdvel_cb(const void * msgin)\n{', 'static float wrap_pi(', 'void apply_stiction_kick(',
    'void MotorControlTask(void *argument)\n{',
    'void StartTeleopTask(void *argument)\n{'))
ramp = function((ROOT / 'Core/Src/motor.c').read_text(), 'void Apply_Robot_RampingDt(')
harness = r'''
static double previous1, previous2;
static void teleop_step(void)
{
    teleop_once = 1;
    if (!setjmp(teleop_finished)) StartTeleopTask(0);
    teleop_once = 0;
}
static void command(double v, double w)
{
    geometry_msgs__msg__Twist msg = { .linear.x=v, .angular.z=w };
    cmdvel_cb(&msg);
    teleop_step();
}
static void osDelay(uint32_t ms)
{
    (void)ms;
    if (teleop_once) longjmp(teleop_finished, 1);
    if (iterations == 0) { // exact boundary: preserve existing >500 policy
        assert(autonomous_MODE == 1 && num_timeout_ROScmd == 0);
        assert(fabs(motor_target1-.2) < 1e-6 && fabs(motor_target2-.1) < 1e-6);
    } else {
        assert(autonomous_MODE == 0 && num_timeout_ROScmd == 1);
        assert(motor_target1 == 0 && motor_target2 == 0);
    }
    tick++;
    if (++iterations == 3) longjmp(finished, 1);
}
static void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    assert(fabs(DesiredSpeed1 - previous1) <= MAXACC * .005 + 1e-7);
    assert(fabs(DesiredSpeed2 - previous2) <= MAXACC * .005 + 1e-7);
    assert(fabs(DesiredSpeed1) <= MAXSPEED + 1e-7);
    assert(fabs(DesiredSpeed2) <= MAXSPEED + 1e-7);
    if (scenario == 6 && iterations >= 50) {
        // Repeated identical ROS commands across simulated intermediate waypoints.
        assert(fabs(DesiredSpeed1-.1) < 1e-6);
        assert(fabs(DesiredSpeed2-.1) < 1e-6);
    }
    if (scenario == 6 && iterations % 10 == 0) command(.1, 0);
    const uint32_t stop_iteration = scenario == 10 ? 60 : 30;
    if ((scenario == 10 || scenario == 11) && iterations == stop_iteration) {
        if (scenario == 10) command(0, 0); // final goal / controller stop
        else { tick = last_cmdvel_ms + 501; teleop_step(); }
        assert(motor_target1 == 0 && motor_target2 == 0);
    }
    if ((scenario == 10 || scenario == 11) && iterations > stop_iteration) {
        assert(fabs(DesiredSpeed1) <= fabs(previous1) + 1e-7);
        assert(fabs(DesiredSpeed2) <= fabs(previous2) + 1e-7);
        if (iterations >= stop_iteration + 15) assert(DesiredSpeed1 == 0 && DesiredSpeed2 == 0);
    }
    if (scenario == 12 && iterations == 30) command(-.1, 0);
    if (scenario == 12 && iterations >= 55) {
        assert(fabs(DesiredSpeed1+.1) < 1e-6);
        assert(fabs(DesiredSpeed2+.1) < 1e-6);
    }
    if (scenario == 9 && iterations == 20) g_theta = .5f;
    previous1 = DesiredSpeed1; previous2 = DesiredSpeed2;
    if (scenario == 0 && iterations == 10) {
        // A held target must ramp on EVERY motor cycle, not only ROS/teleop updates.
        assert(fabs(DesiredSpeed1 - .125) < 1e-6);
        assert(fabs(DesiredSpeed2 - .125) < 1e-6);
    }
    if (scenario == 1 && iterations == 20) publish_motor_targets(-.1, -.1);
    if (scenario == 1 && iterations == 50) publish_motor_targets(0, 0);
    if (scenario == 1 && iterations >= 60) assert(DesiredSpeed1 == 0 && DesiredSpeed2 == 0);
    if (scenario == 2) {
        if (iterations == 20) g_theta = .01f;
        // After heading update, correction must persist on intervening cycles.
        if (iterations == 24 || iterations == 25 || iterations == 26)
            assert(DesiredSpeed1 < .2 && DesiredSpeed2 > .2);
    }
    if (scenario == 3 && iterations == 20) tick += 40; // delayed motor task
    if (++iterations >= limit) longjmp(finished, 1);
    *last += period; tick += period;
}
int main(int argc, char **argv)
{
    (void)argc; scenario = atoi(argv[1]); limit = 150;
    if (scenario == 4 || scenario == 5) {
        // Test ordinary time and unsigned tick wraparound.
        last_cmdvel_ms = scenario == 4 ? 100U : UINT32_MAX-200U;
        tick = last_cmdvel_ms + 500U;
        autonomous_MODE = 1; Spd1_cmdfromROS = .1; Spd2_cmdfromROS = .2;
        if (!setjmp(finished)) StartTeleopTask(0);
        puts("PASS watchdog"); return 0;
    }
    CurSpeed1 = CurSpeed2 = .1; // suppress kick unless explicitly tested
    publish_motor_targets(scenario == 2 ? .2 : .5, scenario == 2 ? .2 : .5);
    if (scenario == 6 || scenario >= 10) command(.1, 0);
    if (scenario == 7 || scenario == 8) {
        CurSpeed1 = CurSpeed2 = 0; // kick active after 200 ms
        command(scenario == 7 ? 2 : -2, 0); // teleop clamp then kick
    }
    if (scenario == 9) command(1, 0); // heading bias near limit
    if (scenario == 10) CurSpeed1 = CurSpeed2 = 0; // stop clears kick too
    if (!setjmp(finished)) MotorControlTask(0);
    assert(dbg_motor_trace_write_count == limit);
    if (scenario == 3) assert(dbg_motor_dt_max_ms == 45);
    double v = .01;
    Apply_Robot_RampingDt(&v, 0, .005); assert(v == 0);
    Apply_Robot_RampingDt(&v, .1, 0); assert(v == 0);
    Apply_Robot_RampingDt(&v, .1, .002); assert(fabs(v-.005) < 1e-8);
    puts("PASS");
}
'''
with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    # Substitute only PID's hardware-facing type; robot constants and globals are real.
    params = (ROOT / 'Core/Inc/robot_params.h').read_text().replace(
        '#include "pid.h"', '#include <stdint.h>\ntypedef struct { int unused; } PID_TypeDef;')
    (tmp / 'robot_params.h').write_text(params)
    (tmp / 'test.c').write_text(preamble + mailbox + functions + ramp + harness)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Wno-unused-parameter',
                    '-I', str(tmp), str(tmp / 'test.c'),
                    str(ROOT / 'Core/Src/robot_params.c'), '-lm', '-o', str(tmp / 'test')], check=True)
    for scenario in range(13):
        subprocess.run([str(tmp / 'test'), str(scenario)], check=True)
