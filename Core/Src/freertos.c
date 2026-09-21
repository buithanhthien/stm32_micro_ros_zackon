/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "robot_params.h"
#include "encoder.h"
#include "motor.h"
#include "motion_profile.h"
#include "encoder_speed.h"
#include "pid.h"
#include "odometry.h"
#include "semphr.h"
#include "touch_rtos.h"
#include "spi.h"
#include "touch_event.h"
#include "xpt2046.h"
#include "ps2xlib.h"
#include "ili9341.h"
#include "ui_screen.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "ethernetif.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>

static rcl_publisher_t odom_pub;
static nav_msgs__msg__Odometry odom_msg;

static bool g_odom_pub_initialized = false;
static bool g_odom_msg_initialized = false;

static volatile bool g_microros_ready = false;     // entities created and session ok
static volatile bool g_microros_run   = false;     // allow tasks to run
static volatile uint32_t g_odom_period_ms = 20;    // 50Hz default
static uint32_t odom_pub_fail_streak = 0;

//typedef enum
//{
//  MICROROS_WAIT_NET = 0,
//  MICROROS_WAIT_AGENT,
//  MICROROS_CREATE_SESSION,
//  MICROROS_RUN,
//  MICROROS_RECONNECT
//} microros_state_t;

// =========================
// Global micro-ROS objects
// =========================
static rcl_allocator_t g_allocator;
static rclc_support_t g_support;
static rcl_node_t g_node;
static rcl_subscription_t g_sub_cmdvel;
static rclc_executor_t g_executor;
static geometry_msgs__msg__Twist g_msg_cmdvel;

// Trạng thái lifecycle của các micro-ROS entities
static bool g_support_initialized = false;
static bool g_node_initialized = false;
static bool g_sub_cmdvel_initialized = false;
static bool g_executor_initialized = false;

// =========================
// CMSIS-RTOS v2 handles
// =========================
static osMutexId_t g_uros_mutex = NULL;
static osThreadId_t g_rx_task_handle = NULL;
static osThreadId_t g_odom_task_handle = NULL;

// =========================
// Shared flags
// =========================
volatile bool g_uros_connected = false;
volatile bool g_request_reconnect = false;

// =========================
// Debug
// =========================
volatile uint32_t dbg_rx_spin_err_cnt = 0;
volatile uint32_t dbg_pub_odom_cnt = 0;
volatile uint32_t dbg_manager_reconnect_cnt = 0;
//----------------------------------------

volatile uint32_t dbg_cmd_cb_count = 0;
volatile uint32_t dbg_last_cmdvel_ms = 0;
volatile uint32_t dbg_cmd_gap_ms = 0;
volatile uint32_t dbg_cmd_gap_max_ms = 0;
volatile uint32_t dbg_cmd_gap_over_500ms = 0;

volatile float dbg_last_cmd_v = 0.0f;
volatile float dbg_last_cmd_w = 0.0f;
//------------------------------------------------------

volatile uint32_t last_cmdvel_ms = 0;
volatile uint8_t autonomous_MODE = 0;
volatile uint32_t num_timeout_ROScmd = 0;


////Live debug variables - remove if done///////////////////
volatile uint32_t age_ros_check;   //just to check ROS--remove if unnecessary
volatile uint32_t dbg_cmdvel_cb_cnt = 0;
volatile uint32_t dbg_odom_pub_cnt  = 0;
volatile uint32_t dbg_spin_cnt      = 0;

//volatile uint32_t dbg_spin_dt_max_ms = 0;
//volatile uint32_t dbg_spin_dt_last_ms = 0;

//volatile char dbg_rcl_err[120];
volatile uint32_t dbg_rcl_err_cnt = 0;

volatile int dbg_last_odom_rc = 0;
volatile int dbg_odom_init_rc = -999;

//volatile uint32_t dbg_heap_free = 0;
//volatile uint32_t dbg_heap_min = 0;
//volatile uint32_t dbg_stack_hw = 0;
////Live debug variables - remove if done///////////////////


#define CMDVEL_TIMEOUT_MS  500

volatile MP_Mode_t modCtrl = MP_MODE_VELOCITY; //default mode
#define POS_CMD_RESYNC_TH    (0.01f)   // position command step that triggers resync (units of position)
#define SPD_CMD_RESYNC_TH    (0.05f)   // speed command step that triggers resync (units of speed)
static inline float fabsf_fast(double x){ return (x < 0.0f) ? -x : x; }

volatile double SetPos1 = 0.0f;
volatile double SetPos2 = 0.0f;
volatile double SetSpeed1 = 0.0f;
volatile double SetSpeed2 = 0.0f;

//these variables are used to correct the heading
static float g_theta = 0.0f;
static float g_theta_ref = 0.0f;
static float g_e_theta_prev = 0.0f;
static uint8_t g_heading_hold_en = 0;
static uint32_t g_heading_div_ctr = 0;
//////////////////////////////////////////////////

volatile int g_activeCell = -1;   // biến dùng cho update lcd

//static volatile uint8_t pid_update_pending = 0;  //biến dùng để update PID từ ROS
static SemaphoreHandle_t pid_mutex;  //kỹ thuật cho phép tạm thời lock trong quá trình thay đổi biến PID, tránh bị cập nhật nửa chừng

volatile uint16_t touch_x = 0;
volatile uint16_t touch_y = 0;
volatile uint8_t  touch_pressed = 0;

//UI variables/definition please put here////////////////////////////////////////////////
typedef struct {
    uint16_t x, y, w, h;
    const char* label;
    uint8_t id;
} UICell_t;

static inline int pointInRect(uint16_t px, uint16_t py, const UICell_t* r)
{
    return (px >= r->x) && (px < r->x + r->w) &&
           (py >= r->y) && (py < r->y + r->h);
}
enum {
    CELL_PWM1 = 0,
    CELL_PWM2,
    CELL_ENC1,
    CELL_ENC2,
    CELL_V1,
    CELL_V2,
    CELL_FIGURE,
    CELL_CONTROLLER,
    CELL_ODOMX,
    CELL_ODOMY,
    CELL_YAW,
    CELL_SETTING_PID,
    CELL_COUNT
};
//UI variables/definition please put here////////////////////////////////////////////////

////Odometry ROS2 sync time////////////////
static bool microros_time_synced = false;

static void microros_sync_time(void)
{
    if (rmw_uros_sync_session(1000) == RMW_RET_OK)
    {
        microros_time_synced = true;
    }
    else
    {
        microros_time_synced = false;
    }
}
/////////////////////////////////////////////////////////



//PS2 control variables -- PUT HERE////////////////////////
typedef struct {
  uint32_t tick;
  uint8_t  id;
  uint16_t buttons;
  uint8_t  lx, ly, rx, ry;
  uint8_t  ok;
  uint8_t  analog;
} PS2X_State;

static PS2X_Handle ps2;

// Shared snapshot for other tasks
static PS2X_State g_ps2_state;
static osMutexId_t g_ps2_mutex;

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float apply_deadband_and_min(float cmd, float deadband, float min_eff)
{
  float a = fabsf(cmd);
  if (a < deadband) return 0.0f;                     // true zero
  if (a < min_eff)  return copysignf(min_eff, cmd);  // stiction boost
  return cmd;
}

static float wrap_pi(float a)
{
    while (a > 3.1415926f)  a -= 2.0f * 3.1415926f;
    while (a < -3.1415926f) a += 2.0f * 3.1415926f;
    return a;
}

static inline int is_pressed(uint16_t buttons, uint16_t mask) {
    return (buttons & mask) ? 1 : 0;
}
//PS2 control variables -- PUT HERE////////////////////////

//////////This variables and functions are used to kick start the robot in case the cmd is too small////////
static uint32_t stuck_ms = 0;

static inline uint32_t ms_now(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }

void apply_stiction_kick(float *vL_cmd, float *vR_cmd, float vL_meas, float vR_meas)
{
    const float CMD_EPS   = 0.02f;   // command considered "wants to move"
    const float MEAS_EPS  = 0.01f;   // measured considered "still not moving"
    const uint32_t STUCK_TIME_MS = 200;

    const float KICK_ADD  = 0.03f;   // m/s extra push
    const float KICK_MAX  = 0.20f;   // m/s cap for kick effect

    float v_avg_cmd  = 0.5f * (*vL_cmd + *vR_cmd);
    float v_avg_meas = 0.5f * (vL_meas + vR_meas);

    if (fabsf(v_avg_cmd) > CMD_EPS && fabsf(v_avg_meas) < MEAS_EPS) {
        stuck_ms += MOTORCONTROL_PERIOD_MS;
        if (stuck_ms >= STUCK_TIME_MS) {
            float add = copysignf(KICK_ADD, v_avg_cmd);
            add = clampf(add, -KICK_MAX, KICK_MAX);
            *vL_cmd += add;
            *vR_cmd += add;
        }
    } else {
        stuck_ms = 0;
    }
}
///////////////////////////////////////////////////////////////////////////

//bien dung de cap nhat du lieu len locLCD
typedef struct {
  int32_t enc1, enc2;
  float   v1, v2;
  float   v1d, v2d;
} EncData_t;

typedef struct {
  float x, y, yaw;
} OdomData_t;

typedef struct {
  float pwm1, pwm2;
} MotorData_t;

// dữ liệu hiện tại
static EncData_t   g_encData;
static OdomData_t  g_odomData;
static MotorData_t g_motorData;

// sequence numbers (odd = đang ghi, even = xong)
static volatile uint32_t g_encSeq   = 0;
static volatile uint32_t g_odomSeq  = 0;
static volatile uint32_t g_motorSeq = 0;

// memory barrier (CMSIS)
#ifndef __DMB
#define __DMB() __asm volatile ("dmb 0xF":::"memory")
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static inline void EncData_Publish(int32_t enc1, int32_t enc2, float v1, float v2, float v1d, float v2d);
static inline void OdomData_Publish(float x, float y, float yaw);
static inline void MotorData_Publish(float pwm1, float pwm2);

static bool microros_odom_init(rcl_node_t * node);
static void microros_odom_fini(rcl_node_t *node);
static void microros_publish_odom(void);
static void cmdvel_cb(const void * msgin);
static void pid_cb(const void * msgin);

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#define RCCHECK(fn)  do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) return false; } while(0)
#define RCSOFTCHECK(fn) do { (void)(fn); } while(0)

//this is used to let it waits for connect
//static void wait_for_agent_forever(void)
//{
//  while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK)
//  {
//    osDelay(200);
//  }
//}

extern struct netif gnetif;

static bool network_is_ready(void)
{
  if (!netif_is_link_up(&gnetif)) return false;
  if (ip4_addr_isany_val(*netif_ip4_addr(&gnetif))) return false;
  return true;
}

static bool create_entities(
    rclc_support_t *support,
    rcl_node_t *node,
    rcl_subscription_t *sub_cmdvel,
    rclc_executor_t *executor,
    rcl_allocator_t *allocator,
    geometry_msgs__msg__Twist *msg_cmdvel)
{
    rcl_ret_t rc;

    // =====================================================
    // 1. Support / XRCE session
    // =====================================================
    rc = rclc_support_init(
        support,
        0,
        NULL,
        allocator
    );

    if (rc != RCL_RET_OK)
    {
        rcl_reset_error();
        return false;
    }

    g_support_initialized = true;

    // =====================================================
    // 2. Node
    // =====================================================
    rc = rclc_node_init_default(
        node,
        "stm32_odom_node",
        "",
        support
    );

    if (rc != RCL_RET_OK)
    {
        rcl_reset_error();
        return false;
    }

    g_node_initialized = true;

    // =====================================================
    // 3. Odom publisher
    // =====================================================
    if (!microros_odom_init(node))
    {
        rcl_reset_error();
        return false;
    }

    // =====================================================
    // 4. cmd_vel subscription
    // =====================================================
    rmw_qos_profile_t qos_cmdvel =
        rmw_qos_profile_default;

    qos_cmdvel.reliability =
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    qos_cmdvel.durability =
        RMW_QOS_POLICY_DURABILITY_VOLATILE;

    qos_cmdvel.history =
        RMW_QOS_POLICY_HISTORY_KEEP_LAST;

    qos_cmdvel.depth = 1;

    rc = rclc_subscription_init(
        sub_cmdvel,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            geometry_msgs,
            msg,
            Twist
        ),
        "cmd_vel",
        &qos_cmdvel
    );

    if (rc != RCL_RET_OK)
    {
        rcl_reset_error();
        return false;
    }

    g_sub_cmdvel_initialized = true;

    // =====================================================
    // 5. Executor
    // =====================================================
    rc = rclc_executor_init(
        executor,
        &support->context,
        1,
        allocator
    );

    if (rc != RCL_RET_OK)
    {
        rcl_reset_error();
        return false;
    }

    g_executor_initialized = true;

    // =====================================================
    // 6. Add subscription vào executor
    // =====================================================
    rc = rclc_executor_add_subscription(
        executor,
        sub_cmdvel,
        msg_cmdvel,
        &cmdvel_cb,
        ON_NEW_DATA
    );

    if (rc != RCL_RET_OK)
    {
        rcl_reset_error();
        return false;
    }

    return true;
}

static void destroy_entities(
    rclc_executor_t *executor,
    rcl_subscription_t *sub_cmdvel,
    rcl_node_t *node,
    rclc_support_t *support)
{
    // =====================================================
    // Agent có thể đã chết.
    // Không chờ ACK khi destroy XRCE entities/session.
    // =====================================================
    if (g_support_initialized)
    {
        rmw_context_t *rmw_context =
            rcl_context_get_rmw_context(
                &support->context
            );

        if (rmw_context != NULL)
        {
            (void)
            rmw_uros_set_context_entity_destroy_session_timeout(
                rmw_context,
                0
            );
        }
    }

    // =====================================================
    // Executor
    // =====================================================
    if (g_executor_initialized)
    {
        (void)rclc_executor_fini(
            executor
        );

        g_executor_initialized = false;
    }

    // =====================================================
    // cmd_vel subscription
    // =====================================================
    if (g_sub_cmdvel_initialized &&
        g_node_initialized)
    {
        (void)rcl_subscription_fini(
            sub_cmdvel,
            node
        );

        g_sub_cmdvel_initialized = false;
    }

    // =====================================================
    // odom publisher + odom message
    // =====================================================
    if (g_node_initialized)
    {
        microros_odom_fini(node);
    }

    // =====================================================
    // Node
    // =====================================================
    if (g_node_initialized)
    {
        (void)rcl_node_fini(
            node
        );

        g_node_initialized = false;
    }

    // =====================================================
    // Support / context / session
    // =====================================================
    if (g_support_initialized)
    {
        (void)rclc_support_fini(
            support
        );

        g_support_initialized = false;
    }

    // =====================================================
    // Quan trọng:
    // đưa toàn bộ handles về zero-state trước lần create mới.
    // =====================================================
    *support =
        (rclc_support_t){0};

    *node =
        (rcl_node_t){0};

    *sub_cmdvel =
        (rcl_subscription_t){0};

    *executor =
        (rclc_executor_t){0};

    g_msg_cmdvel =
        (geometry_msgs__msg__Twist){0};

    odom_pub_fail_streak = 0;

    rcl_reset_error();
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t encoderTaskHandle;
osThreadId_t motorTaskHandle;
osThreadId_t odometryTaskHandle;
osThreadId_t touchTaskHandle;
osThreadId_t PS2TaskHandle;
//osThreadId_t LCDTaskHandle;
osThreadId_t teleopTaskHandle;
//osThreadId_t motionTaskHandle;

const osThreadAttr_t encoderTask_attributes = {
  .name       = "EncoderTask",
  .priority   = (osPriority_t) osPriorityRealtime,
  .stack_size = 256 * 4
};
const osThreadAttr_t motorTask_attributes = {
  .name       = "MotorTask",
  .priority   = (osPriority_t) osPriorityRealtime,
  .stack_size = 256 * 4
};
const osThreadAttr_t odometryTask_attributes = {
  .name       = "OdometryTask",
  .priority   = (osPriority_t) osPriorityRealtime,
  .stack_size = 256 * 4
};
static const osThreadAttr_t touchTask_attributes = {
      .name = "TouchTask",
      .stack_size = 512 * 4,              // bytes
      .priority = (osPriority_t) osPriorityBelowNormal
};
const osThreadAttr_t PS2Task_attributes = {
  .name       = "PS2Task",
  .stack_size = 512 * 4,   // 2KB stack (đủ cho parse HID)
  .priority   = (osPriority_t) osPriorityNormal
};
const osThreadAttr_t teleopTask_attributes = {
  .name = "TeleopTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4   // bytes (512 words if you think in words; here it's bytes)
};
//const osThreadAttr_t lcdTask_attributes = {
//  .name = "lcdTask",
//  .stack_size = 1024,
//  .priority = (osPriority_t) osPriorityLow  // UI để Low là hợp lý
//};
//const osThreadAttr_t motionTask_attributes = {
//  .name = "motionTask",
//  .stack_size = 512 * 4,
//  .priority = (osPriority_t) osPriorityNormal  // UI để Low là hợp lý
//};

void StartUITask(void *argument);
void EncoderTask(void *argument);
void MotorControlTask(void *argument);
void OdometryTask(void *argument);
//void MotionTask(void *argument);
void TouchTask(void *argument);
void StartPS2Task(void *argument);
void StartLCDTask(void *argument);
void StartTeleopTask(void *argument);
void StartMicroRosSpinTask(void *argument);
void StartOdomPubTask(void *argument);

void StartMicroRosRxTask(void *argument);
void StartMicroRosOdomPubTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  pid_mutex = xSemaphoreCreateMutex();
  g_ps2_mutex = osMutexNew(NULL);


  encoderTaskHandle = osThreadNew(EncoderTask, NULL, &encoderTask_attributes);
  motorTaskHandle   = osThreadNew(MotorControlTask, NULL, &motorTask_attributes);
  odometryTaskHandle = osThreadNew(OdometryTask, NULL, &odometryTask_attributes);
  touchTaskHandle = osThreadNew(TouchTask_Start, NULL, &touchTask_attributes);
  PS2TaskHandle = osThreadNew(StartPS2Task, NULL, &PS2Task_attributes);
  teleopTaskHandle = osThreadNew(StartTeleopTask, NULL, &teleopTask_attributes);
  //LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &lcdTask_attributes);
//  motionTaskHandle = osThreadNew(MotionTask, NULL, &motionTask_attributes);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartDefaultTask(void *argument)
{
    (void)argument;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // 1) Bring up LwIP
    MX_LWIP_Init();
    osDelay(300);

    // 2) micro-ROS custom UDP transport
    static const char *agent_ip = "192.168.1.150";

    rmw_uros_set_custom_transport(
        false,
        (void *)agent_ip,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read
    );

    // 3) FreeRTOS allocator
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    rcutils_set_default_allocator(&freeRTOS_allocator);
    g_allocator = rcl_get_default_allocator();

    // Create mutex 1 lần
    if (g_uros_mutex == NULL)
    {
        g_uros_mutex = osMutexNew(NULL);
    }

    // Create RX task 1 lần
    if (g_rx_task_handle == NULL)
    {
        const osThreadAttr_t rxTask_attributes = {
            .name = "uROS_RX",
            .stack_size = 1024 * 4,
            .priority = (osPriority_t) osPriorityAboveNormal
        };
        g_rx_task_handle = osThreadNew(StartMicroRosRxTask, NULL, &rxTask_attributes);
    }

    // Create Odom task 1 lần
    if (g_odom_task_handle == NULL)
    {
        const osThreadAttr_t odomTask_attributes = {
            .name = "uROS_ODOM",
            .stack_size = 1024 * 4,
            .priority = (osPriority_t) osPriorityNormal
        };
        g_odom_task_handle = osThreadNew(StartMicroRosOdomPubTask, NULL, &odomTask_attributes);
    }

    uint8_t wait_agent_fail_cnt = 0U;
    bool eth_recovery_done_for_outage = false;

    for (;;)
    {
        // update touch nếu cần
        touch_x = g_touchState.x;
        touch_y = g_touchState.y;
        touch_pressed = g_touchState.pressed;

        // 0) network ready?
        if (!network_is_ready())
        {
            g_uros_connected = false;
            g_request_reconnect = false;

            /*
            * Physical link thực sự mất.
            * ethernet_link_thread sẽ tự xử lý
            * LINK DOWN -> LINK UP.
            */
            wait_agent_fail_cnt = 0U;
            eth_recovery_done_for_outage = false;

            osDelay(200);
            continue;
        }

        // 1) wait agent
        if (rmw_uros_ping_agent(100, 2) != RMW_RET_OK)
        {
            g_uros_connected = false;
            g_request_reconnect = false;

            /*
             * network_is_ready() vẫn TRUE nhưng Agent
             * không reachable.
             *
             * Đây chính là trường hợp có thể xảy ra khi:
             *
             * PC reboot
             * PHY vẫn báo LINK UP
             * nhưng ETH MAC/DMA/LwIP phía STM32 bị kẹt.
             */
            if (wait_agent_fail_cnt < 20U)
            {
                wait_agent_fail_cnt++;
            }

            /*
             * Force recovery đúng 1 lần trong đợt mất kết nối.
             *
             * Một vòng ở đây gồm ping timeout + delay,
             * nên 20 lần cho PC/card mạng có thời gian ổn định.
             */
            if ((wait_agent_fail_cnt >= 20U) &&
                (!eth_recovery_done_for_outage))
            {
                ethernet_request_recovery();

                eth_recovery_done_for_outage = true;
                wait_agent_fail_cnt = 0U;

                /*
                 * Cho ethernet_link_thread thời gian:
                 *
                 * Stop
                 * DeInit
                 * Init
                 * Start
                 */
                osDelay(1000);
            }
            else
            {
                osDelay(200);
            }

            continue;
        }

        /*
         * Ping Agent thành công.
         * Kết thúc đợt lỗi hiện tại và cho phép
         * recovery ở lần mất kết nối tiếp theo.
         */
        wait_agent_fail_cnt = 0U;
        eth_recovery_done_for_outage = false;

        // 2) create entities
        bool ok = false;

        if (osMutexAcquire(g_uros_mutex, 100) == osOK)
        {
            ok = create_entities(
                &g_support,
                &g_node,
                &g_sub_cmdvel,
                &g_executor,
                &g_allocator,
                &g_msg_cmdvel
            );

            /*
             * Nếu create fail giữa chừng:
             * cleanup ngay các object đã được tạo.
             */
            if (!ok)
            {
                destroy_entities(
                    &g_executor,
                    &g_sub_cmdvel,
                    &g_node,
                    &g_support
                );
            }

            osMutexRelease(g_uros_mutex);
        }

        if (!ok)
        {
            g_uros_connected = false;
            g_request_reconnect = false;

            osDelay(200);
            continue;
        }

        // 3) sync time mỗi lần connect / reconnect
        if (osMutexAcquire(g_uros_mutex, 100) == osOK)
        {
            (void)rmw_uros_sync_session(500);
            osMutexRelease(g_uros_mutex);
        }

        g_request_reconnect = false;
        g_uros_connected = true;

        uint32_t last_ping = osKernelGetTickCount();
        uint8_t ping_fail_cnt = 0;

        while (1)
        {
            touch_x = g_touchState.x;
            touch_y = g_touchState.y;
            touch_pressed = g_touchState.pressed;

            if (g_request_reconnect)
            {
                break;
            }

            uint32_t now_ms = osKernelGetTickCount();

            // ping agent chậm để tránh ảnh hưởng timing
            if ((now_ms - last_ping) >= 5000U)   // 5 giây
            {
                last_ping = now_ms;

                bool ping_ok = false;

                if (network_is_ready())
                {
                    if (osMutexAcquire(g_uros_mutex, 100) == osOK)
                    {
                        ping_ok =
                            (rmw_uros_ping_agent(20, 1) == RMW_RET_OK);

                        osMutexRelease(g_uros_mutex);
                    }
                }

                if (!ping_ok)
                {
                    ping_fail_cnt++;
                }
                else
                {
                    ping_fail_cnt = 0;
                }

                if (ping_fail_cnt >= 3U)
                {
                    break;
                }
            }

            osDelay(50);
        }

        // Prepare reconnect
        g_uros_connected = false;

        /*
         * Giữ reconnect=true trong lúc cleanup
         * để RX và ODOM worker chắc chắn không đụng micro-ROS.
         */
        g_request_reconnect = true;

        dbg_manager_reconnect_cnt++;

        osDelay(50);

        /*
         * Hai worker đã ngừng sử dụng micro-ROS.
         * Chờ cho tới khi lấy được mutex,
         * không được bỏ qua cleanup.
         */
        if (osMutexAcquire(
                g_uros_mutex,
                osWaitForever
            ) == osOK)
        {
            destroy_entities(
                &g_executor,
                &g_sub_cmdvel,
                &g_node,
                &g_support
            );

            osMutexRelease(
                g_uros_mutex
            );
        }

        /*
         * Cleanup hoàn tất.
         * Cho phép vòng manager thử tạo session mới.
         */
        g_request_reconnect = false;

        osDelay(100);
    }
}

//////////////////THIS PART IS MICROROS USING UDP TRANSPORT-testing with statemachine///////////////
//////////////////////////////////////////////////////////////////////////
//void StartDefaultTask(void *argument)
//{
//  (void)argument;
//
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//
//  // 1) Bring up LwIP
//  MX_LWIP_Init();
//  osDelay(300);
//
//  // 2) micro-ROS custom UDP transport
//  static const char * agent_ip = "192.168.1.150";
//
//  rmw_uros_set_custom_transport(
//    false,
//    (void*)agent_ip,
//    cubemx_transport_open,
//    cubemx_transport_close,
//    cubemx_transport_write,
//    cubemx_transport_read
//  );
//
//  // 3) FreeRTOS allocator
//  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
//  freeRTOS_allocator.allocate      = microros_allocate;
//  freeRTOS_allocator.deallocate    = microros_deallocate;
//  freeRTOS_allocator.reallocate    = microros_reallocate;
//  freeRTOS_allocator.zero_allocate = microros_zero_allocate;
//  rcutils_set_default_allocator(&freeRTOS_allocator);
//
//  rcl_allocator_t allocator = rcl_get_default_allocator();
//
//  // micro-ROS objects
//  rclc_support_t support = {0};
//  rcl_node_t node = {0};
//  rcl_subscription_t sub_cmdvel = {0};
//  rclc_executor_t executor = {0};
//  geometry_msgs__msg__Twist msg_cmdvel;
//
//  // State machine
//  microros_state_t state = MICROROS_WAIT_NET;
//
//  // Fast loop config
//  const uint64_t spin_timeout_ns = 2ULL * 1000000ULL;   // 2 ms
//  const TickType_t period = pdMS_TO_TICKS(10);          // 100 Hz loop
//
//  // Timing
//  TickType_t last_wake_time = xTaskGetTickCount();
//  uint32_t last_odom_ms = 0;
//  uint32_t last_watchdog_ms = 0;
//
//  // Watchdog / reconnect
//  uint8_t runtime_ping_fail_cnt = 0;
//
//  // Optional debug
//  // uint32_t last_debug_ms = 0;
//
//  for (;;)
//  {
//    switch (state)
//    {
//      case MICROROS_WAIT_NET:
//      {
//        if (network_is_ready())
//        {
//          state = MICROROS_WAIT_AGENT;
//        }
//        else
//        {
//          osDelay(200);
//        }
//        break;
//      }
//
//      case MICROROS_WAIT_AGENT:
//      {
//        // Chỉ ping ở state này, không ping trong fast loop
//        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK)
//        {
//          state = MICROROS_CREATE_SESSION;
//        }
//        else
//        {
//          osDelay(500);
//        }
//        break;
//      }
//
//      case MICROROS_CREATE_SESSION:
//      {
//        bool ok = create_entities(
//          &support,
//          &node,
//          &sub_cmdvel,
//          &executor,
//          &allocator,
//          &msg_cmdvel
//        );
//
//        if (!ok)
//        {
//          destroy_entities(&executor, &sub_cmdvel, &node, &support);
//          osDelay(200);
//          state = MICROROS_WAIT_AGENT;
//          break;
//        }
//
//        // Sync ROS time 1 lần sau khi connect / reconnect thành công
//        // Không cần ép buộc phải thành công mới RUN, nhưng nên thử
//        (void)rmw_uros_sync_session(500);
//
//        // Reset runtime timing
//        last_wake_time = xTaskGetTickCount();
//        last_odom_ms = osKernelGetTickCount();
//        last_watchdog_ms = osKernelGetTickCount();
//        runtime_ping_fail_cnt = 0;
//
//        state = MICROROS_RUN;
//        break;
//      }
//
//      case MICROROS_RUN:
//      {
//        // 1) đọc dữ liệu ROS2 / gọi callback như cmdvel_cb()
//        rcl_ret_t rc = rclc_executor_spin_some(&executor, spin_timeout_ns);
//
//        // Nếu executor lỗi thật thì reconnect
//        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT)
//        {
//          state = MICROROS_RECONNECT;
//          break;
//        }
//
//        // 2) phần xử lý local khác của bạn
//        touch_x = g_touchState.x;
//        touch_y = g_touchState.y;
//        touch_pressed = g_touchState.pressed;
//
//        uint32_t now_ms = osKernelGetTickCount();
//
//        microros_publish_odom(); //publish odom
//
//        // 4) watchdog nhẹ, rất thưa, để phát hiện agent reset
//        // Không check quá nhanh để tránh ảnh hưởng loop
//        if ((now_ms - last_watchdog_ms) >= 5000U)   // mỗi 5 giây
//        {
//          last_watchdog_ms = now_ms;
//
//          if (!network_is_ready() || rmw_uros_ping_agent(20, 1) != RMW_RET_OK)
//          {
//            runtime_ping_fail_cnt++;
//          }
//          else
//          {
//            runtime_ping_fail_cnt = 0;
//          }
//
//          // Fail liên tiếp thì coi như mất session -> reconnect
//          if (runtime_ping_fail_cnt >= 3U)
//          {
//            state = MICROROS_RECONNECT;
//            break;
//          }
//        }
//
//        // Optional debug mỗi 1 giây
//        /*
//        if ((now_ms - last_debug_ms) >= 1000U)
//        {
//          last_debug_ms = now_ms;
//          printf("[uROS] RUN ok, heap=%lu, min_heap=%lu\r\n",
//                 (unsigned long)xPortGetFreeHeapSize(),
//                 (unsigned long)xPortGetMinimumEverFreeHeapSize());
//        }
//        */
//
//        // 5) giữ loop 100 Hz
//        vTaskDelayUntil(&last_wake_time, period);
//        break;
//      }
//
//      case MICROROS_RECONNECT:
//      {
//        destroy_entities(&executor, &sub_cmdvel, &node, &support);
//
//        // Reset object handles về zero-state cho an toàn
//        support = (rclc_support_t){0};
//        node = (rcl_node_t){0};
//        sub_cmdvel = (rcl_subscription_t){0};
//        executor = (rclc_executor_t){0};
//
//        osDelay(200);
//        state = MICROROS_WAIT_AGENT;
//        break;
//      }
//
//      default:
//      {
//        state = MICROROS_WAIT_NET;
//        break;
//      }
//    }
//  }
//}


//void StartDefaultTask(void *argument)
//{
//    (void)argument;
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//
//    MX_LWIP_Init();
//    osDelay(300);
//
//    static const char * agent_ip = "192.168.1.150";
//
//    rmw_uros_set_custom_transport(
//        false,
//        (void*)agent_ip,
//        cubemx_transport_open,
//        cubemx_transport_close,
//        cubemx_transport_write,
//        cubemx_transport_read
//    );
//
//    // allocator như bạn đang làm
//    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
//    freeRTOS_allocator.allocate      = microros_allocate;
//    freeRTOS_allocator.deallocate    = microros_deallocate;
//    freeRTOS_allocator.reallocate    = microros_reallocate;
//    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
//    rcutils_set_default_allocator(&freeRTOS_allocator);
//
//    microros_mutex_init();
//
//    // tạo 2 task
//    osThreadNew(StartMicroRosSpinTask, NULL, &urosSpin_attr);
//    osThreadNew(StartOdomPubTask, NULL, &urosOdom_attr);
//
//    // DefaultTask không cần làm gì nữa
//    vTaskDelete(NULL);
//}
//////////////////THIS PART IS MICROROS USING UDP TRANSPORT///////////////
//////////////////////////////////////////////////////////////////////////


/////////////////THIS PART IS MICROROS USING UART//////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//void StartDefaultTask(void *argument)
//{
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//
//  rmw_uros_set_custom_transport(
//    true,
//    (void *) &huart3,
//    cubemx_transport_open,
//    cubemx_transport_close,
//    cubemx_transport_write,
//    cubemx_transport_read);
//
//  // Allocator FreeRTOS (your code)
//  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
//  freeRTOS_allocator.allocate      = microros_allocate;
//  freeRTOS_allocator.deallocate    = microros_deallocate;
//  freeRTOS_allocator.reallocate    = microros_reallocate;
//  freeRTOS_allocator.zero_allocate = microros_zero_allocate;
//  rcutils_set_default_allocator(&freeRTOS_allocator);
//
//  rcl_allocator_t allocator = rcl_get_default_allocator();
//
//  // micro-ROS objects (keep them outside so we can destroy/recreate)
//  rclc_support_t support;
//  rcl_node_t node;
//  rcl_subscription_t sub_cmdvel;
//  rcl_subscription_t sub_pid;
//  rclc_executor_t executor;
//
//  geometry_msgs__msg__Twist msg_cmdvel;
//  std_msgs__msg__Float64MultiArray msg_pid;
//
//  const uint32_t period_ms = 10;
//  const float dt = 0.010f;
//  const uint64_t spin_timeout_ns = 2ULL * 1000000ULL;
//
//  for (;;)
//  {
//    // 1) Wait for agent (no reset needed)
//    osDelay(300);
//    wait_for_agent_forever();
//
//    // 2) Create entities; if fails, retry
//    bool ok = create_entities(&support, &node, &sub_cmdvel, &sub_pid, &executor,
//                              &allocator, &msg_cmdvel, &msg_pid);
//
//    if (!ok)
//    {
//      destroy_entities(&executor, &sub_cmdvel, &sub_pid, &node, &support);
//      osDelay(500);
//      continue;
//    }
//
//    // 3) Normal running loop (until agent disappears)
////    float last_x = odom_x, last_y = odom_y, last_theta = odom_theta;
//
//    while (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
//    {
//      touch_x = g_touchState.x;
//      touch_y = g_touchState.y;
//      touch_pressed = g_touchState.pressed;
//
////      float curr_x = odom_x, curr_y = odom_y, curr_theta = odom_theta;
////
////      odom_vx  = (curr_x - last_x) / dt;
////      odom_vy  = (curr_y - last_y) / dt;
////      odom_vth = (curr_theta - last_theta) / dt;
////
////      last_x = curr_x; last_y = curr_y; last_theta = curr_theta;
//
//      microros_publish_odom();
//      rclc_executor_spin_some(&executor, spin_timeout_ns);
//
//      osDelay(period_ms);
//    }
//
//    // 4) Agent lost → cleanup and go back to waiting
//    destroy_entities(&executor, &sub_cmdvel, &sub_pid, &node, &support);
//    osDelay(500);
//  }
//}
/////////////////THIS PART IS MICROROS USING UART//////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void EncoderTask(void *argument)
{
	const TickType_t period = pdMS_TO_TICKS(ENCODER_PERIOD_MS);
	TickType_t lastWake = xTaskGetTickCount();

  for(;;)
  {
    // Đọc count mở rộng
	g_encCount1 = Encoder_GetCount(ENCODER_1);
    g_encCount2 = Encoder_GetCount(ENCODER_2);

    g_speed1_mps = Encoder_GetSpeed_mps(ENCODER_1,DT_TIME);
    g_speed2_mps = Encoder_GetSpeed_mps(ENCODER_2,DT_TIME);

    CurSpeed1 = g_speed1_mps;
    CurSpeed2 = g_speed2_mps;

    float vL_meas = CurSpeed2;
    float vR_meas = CurSpeed1;

    float theta_dot = (vR_meas - vL_meas) / WHEEL_BASE_M;
    g_theta = wrap_pi(g_theta + theta_dot * DT_TIME);

    EncData_Publish(g_encCount1, g_encCount2, g_speed1_mps, g_speed2_mps, DesiredSpeed1, DesiredSpeed2);

    vTaskDelayUntil(&lastWake, period);
  }
}

void MotorControlTask(void *argument)
{
	const TickType_t period = pdMS_TO_TICKS(MOTORCONTROL_PERIOD_MS);
	TickType_t lastWake = xTaskGetTickCount();

	for (;;)
	    {
			//update thông số PID nếu có thay đổi
			if (pid_update_pending)
			{
				if (pid_mutex) xSemaphoreTake(pid_mutex, pdMS_TO_TICKS(2));

				PID_SetTunings(&SpeedPID1, Kp1, Ki1, Kd1);
				PID_SetTunings(&SpeedPID2, Kp2, Ki2, Kd2);
				PID_SetTunings(&PosPID1, Kp1, Ki1, Kd1);
				PID_SetTunings(&PosPID2, Kp2, Ki2, Kd2);

				pid_update_pending = 0;

				if (pid_mutex) xSemaphoreGive(pid_mutex);
			}

			if (modCtrl == MP_MODE_POSITION) {
				PID_Compute(&PosPID1);
				PID_Compute(&PosPID2);
				MotorData_Publish(PosPIDOut1, PosPIDOut2);  //update PWM data for LCD display
				Motor_SetSpeed(MOTOR_1,PosPIDOut1);
				Motor_SetSpeed(MOTOR_2,PosPIDOut2);
			} else {
				//Decide whether to enable heading hold (straight mode detection with latch)
				float vL_ref_cmd = DesiredSpeed2;
				float vR_ref_cmd = DesiredSpeed1;

				float v_avg_cmd = 0.5f * (vL_ref_cmd + vR_ref_cmd);
				float dv_cmd    = (vR_ref_cmd - vL_ref_cmd);

				uint8_t want_heading_hold =
				    (fabsf(dv_cmd) < EPS_TURN_MPS) &&
				    (fabsf(v_avg_cmd) > EPS_MOVE_MPS);

				//Then detect rising edge to latch heading reference:
				if (want_heading_hold && !g_heading_hold_en) {
				    g_theta_ref = g_theta;        // latch current heading
				    g_e_theta_prev = 0.0f;
				}

				g_heading_hold_en = want_heading_hold;

				//Heading PD outer loop (run every 20 ms), then modify DesiredSpeed1/2
				float omega_corr = 0.0f;

				if (g_heading_hold_en) {

				    // Run heading controller at slower rate
				    g_heading_div_ctr++;
				    if (g_heading_div_ctr >= HEADING_DIV) {
				        g_heading_div_ctr = 0;

				        float e_theta = wrap_pi(g_theta_ref - g_theta);
				        float de      = (e_theta - g_e_theta_prev) / HEADING_DT;

				        omega_corr = HEADING_KP * e_theta + HEADING_KD * de;
				        omega_corr = clampf(omega_corr, -OMEGA_CORR_MAX, OMEGA_CORR_MAX);

				        g_e_theta_prev = e_theta;

				        // Convert omega correction -> wheel linear speed offsets
				        // vR = v + omega*L/2 ; vL = v - omega*L/2
				        float v_corr = omega_corr * (WHEEL_BASE_M * 0.5f);

				        // Keep same average command, just bias left/right
				        float v_avg = v_avg_cmd;

				        vR_ref_cmd = v_avg + v_corr;
				        vL_ref_cmd = v_avg - v_corr;
				    }
				    // else: keep last computed vL_ref_cmd/vR_ref_cmd from previous heading tick
				} else {
				    g_heading_div_ctr = 0;
				}

				DesiredSpeed2 = vL_ref_cmd;
				DesiredSpeed1 = vR_ref_cmd;
/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////end of correct heading///////////////////////////////////////////////////////////

				//Here apply kick start if needed////////////////////////////////////////////
				float vL_cmd = DesiredSpeed2;
				float vR_cmd = DesiredSpeed1;

				// Replace these with your real measured wheel speeds used by PID inputs:
				float vL_meas = CurSpeed2;
				float vR_meas = CurSpeed1;

				apply_stiction_kick(&vL_cmd, &vR_cmd, vL_meas, vR_meas);

				DesiredSpeed2 = vL_cmd;
				DesiredSpeed1 = vR_cmd;
				//////////////////End of applying kick////////////////////////////////////////

				PID_Compute(&SpeedPID1);
				PID_Compute(&SpeedPID2);
				MotorData_Publish(SpeedPIDOut1, SpeedPIDOut2);  //update PWM data for LCD display
				Motor_SetSpeed(MOTOR_1,SpeedPIDOut1);
				Motor_SetSpeed(MOTOR_2,SpeedPIDOut2);
			}

			vTaskDelayUntil(&lastWake, period);
	    }
}

void OdometryTask(void *argument)
{
    const TickType_t period = pdMS_TO_TICKS(ODOMETRY_PERIOD_MS);
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastTick = lastWake;

    for (;;)
    {
        TickType_t nowTick = xTaskGetTickCount();
        float dt = (float)(nowTick - lastTick) * (1.0f / configTICK_RATE_HZ);
        lastTick = nowTick;

        Odometry_Update(dt);
        OdomData_Publish(odom_x, odom_y, odom_theta);

        vTaskDelayUntil(&lastWake, period);
    }
}

void yaw_to_quaternion(float yaw, geometry_msgs__msg__Quaternion *q)
{
    q->x = 0.0;
    q->y = 0.0;
    q->z = sinf(yaw * 0.5f);
    q->w = cosf(yaw * 0.5f);
}

static bool microros_odom_init(rcl_node_t *node)
{
    odom_pub = rcl_get_zero_initialized_publisher();

    if (!nav_msgs__msg__Odometry__init(&odom_msg))
    {
        g_odom_msg_initialized = false;
        return false;
    }

    g_odom_msg_initialized = true;

    rcl_ret_t rc = rclc_publisher_init_default(
        &odom_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odomfromSTM32"
    );

    dbg_odom_init_rc = (int)rc;

    if (rc != RCL_RET_OK)
    {
        nav_msgs__msg__Odometry__fini(&odom_msg);

        g_odom_msg_initialized = false;
        g_odom_pub_initialized = false;

        return false;
    }

    g_odom_pub_initialized = true;

    rosidl_runtime_c__String__assign(
        &odom_msg.header.frame_id,
        "odom"
    );

    rosidl_runtime_c__String__assign(
        &odom_msg.child_frame_id,
        "base_link"
    );

    return true;
}

static void microros_odom_fini(rcl_node_t *node)
{
    if (g_odom_pub_initialized)
    {
        (void)rcl_publisher_fini(
            &odom_pub,
            node
        );

        g_odom_pub_initialized = false;
    }

    if (g_odom_msg_initialized)
    {
        nav_msgs__msg__Odometry__fini(
            &odom_msg
        );

        g_odom_msg_initialized = false;
    }

    odom_pub = rcl_get_zero_initialized_publisher();

    memset(
        &odom_msg,
        0,
        sizeof(odom_msg)
    );
}

static void microros_publish_odom(void)
{
    int64_t now_ns = rmw_uros_epoch_nanos();

    odom_msg.header.stamp.sec =
        (int32_t)(now_ns / 1000000000LL);

    odom_msg.header.stamp.nanosec =
        (uint32_t)(now_ns % 1000000000LL);

    // pose
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0f;

    yaw_to_quaternion(odom_theta, &odom_msg.pose.pose.orientation);

    // twist
    odom_msg.twist.twist.linear.x  = odom_vx;
    odom_msg.twist.twist.linear.y  = 0.0f;
    odom_msg.twist.twist.angular.z = odom_wz;

    rcl_ret_t rc = rcl_publish(&odom_pub, &odom_msg, NULL);

    if (rc == RCL_RET_OK) {
        odom_pub_fail_streak = 0;
    } else {
        odom_pub_fail_streak++;
        dbg_rcl_err_cnt++;
        //dbg_last_pub_rc = rc;
        if (odom_pub_fail_streak >= 5) {
            g_request_reconnect = true;
        }
        rcl_reset_error();
    }



//    if (rc != RCL_RET_OK) {
//        const rcl_error_string_t es = rcl_get_error_string();
//        // copy sang buffer để xem trong Debug Watch
//        size_t i = 0;
//        for (; i < sizeof(dbg_rcl_err)-1 && es.str[i] != '\0'; i++) {
//            dbg_rcl_err[i] = es.str[i];
//        }
//        dbg_rcl_err[i] = '\0';
//        dbg_rcl_err_cnt++;
//        rcl_reset_error();
//    }

//	if (rc != RCL_RET_OK) {
//		printf("[ODOM] rcl_publish failed rc=%d\n", (int)rc);
//	} else {
//		printf("[ODOM] ok\n");
//	}
}

static uint32_t millis_void(void)
{
	return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

// ===== Callback /cmd_vel: tính DesiredSpeed1/2 cho PID =====
static void cmdvel_cb(const void * msgin)
{
	dbg_cmdvel_cb_cnt++;

    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *) msgin;
    if (m == NULL) return;

    uint32_t now = osKernelGetTickCount();

	if (dbg_last_cmdvel_ms != 0U) {
		dbg_cmd_gap_ms = now - dbg_last_cmdvel_ms;
		if (dbg_cmd_gap_ms > dbg_cmd_gap_max_ms) {
			dbg_cmd_gap_max_ms = dbg_cmd_gap_ms;
		}
		if (dbg_cmd_gap_ms > 500U) {
			dbg_cmd_gap_over_500ms++;
		}
	}

	dbg_last_cmdvel_ms = now;
	dbg_cmd_cb_count++;
	dbg_last_cmd_v = m->linear.x;
	dbg_last_cmd_w = m->angular.z;

    float v = m->linear.x;   // m/s
    float w = m->angular.z;  // rad/s

    // --- tunables (start here) ---
    const float V_DEADBAND = 0.0005f;   // m/s  ignore tiny MPPI noise
    const float W_DEADBAND = 0.005f;    // rad/s

    const float V_MIN_EFF  = 0.05f;    // m/s  minimum that actually moves your robot
    const float W_MIN_EFF  = 0.25f;    // rad/s minimum rotate to break stiction

    // If MPPI asks to move/turn slightly, boost above stiction threshold
    v = apply_deadband_and_min(v, V_DEADBAND, V_MIN_EFF);
    w = apply_deadband_and_min(w, W_DEADBAND, W_MIN_EFF);

    // diff-drive: wheel linear speeds (m/s)
    float v_r = v + (w * WHEEL_BASE_M * 0.5f);
    float v_l = v - (w * WHEEL_BASE_M * 0.5f);

    Spd1_cmdfromROS = v_l;
    Spd2_cmdfromROS = v_r;

    last_cmdvel_ms  = millis_void();
    autonomous_MODE = 1;

}

//hàm callback khi nhận được data PID từ ROS
static void pid_cb(const void * msgin)
{
    const std_msgs__msg__Float64MultiArray *m =
        (const std_msgs__msg__Float64MultiArray *)msgin;

    // cần đủ 6 phần tử
    if (m->data.size < 6) return;

    if (pid_mutex) xSemaphoreTake(pid_mutex, pdMS_TO_TICKS(2));

    Kp1 = m->data.data[0];
    Ki1 = m->data.data[1];
    Kd1 = m->data.data[2];
    Kp2 = m->data.data[3];
    Ki2 = m->data.data[4];
    Kd2 = m->data.data[5];

    pid_update_pending = 1;

    if (pid_mutex) xSemaphoreGive(pid_mutex);
}

//void MotionTask(void *argument)
//{
//	const TickType_t period = pdMS_TO_TICKS(MOTORCONTROL_PERIOD_MS);
//	TickType_t lastWake = xTaskGetTickCount();
//
//	MP_Config_t mp_cfg1, mp_cfg2;
//	MP_State_t  mp_state1, mp_state2;
//
//    mp_cfg1.dt      = DT_TIME;     // 5 ms
//    mp_cfg1.vmax    = MAXSPEED;   // your units
//    mp_cfg1.amax    = MAXACC;
//    mp_cfg1.vel_eps = 0.01f * MAXSPEED; // tune
//    mp_cfg1.pos_eps = 0.001f;          // tune (depends on units)
//
//    mp_cfg2.dt      = DT_TIME;     // 5 ms
//    mp_cfg2.vmax    = MAXSPEED;   // your units
//    mp_cfg2.amax    = MAXACC;
//    mp_cfg2.vel_eps = 0.01f * MAXSPEED; // tune
//    mp_cfg2.pos_eps = 0.001f;          // tune (depends on units)
//
//    // Initialize using current measured state:
//    MP_Init(&mp_state1, CurPos1, CurSpeed1);
//    MP_Init(&mp_state2, CurPos2, CurSpeed2);
//
//    int   prevMode = modCtrl;
//
//    float prevSetPos1 = SetPos1, prevSetPos2 = SetPos2;
//    float prevSetSpd1 = SetSpeed1, prevSetSpd2 = SetSpeed2;
//
//    for (;;)
//    {
//    	 // ---- Snapshot inputs (avoid reading changing globals mid-loop) ----
//    	int   mode = modCtrl;
//        float curPos1 = CurPos1, curPos2 = CurPos2;
//        float curSpd1 = CurSpeed1, curSpd2 = CurSpeed2;
//
//        float setPos1 = SetPos1, setPos2 = SetPos2;
//        float setSpd1 = SetSpeed1, setSpd2 = SetSpeed2;
//
//        // ---- Determine whether to re-seed planner state (ONE-TIME) ----
//        uint8_t modeChanged = (mode != prevMode);
//
//        uint8_t posCmdJump =
//            (fabsf_fast(setPos1 - prevSetPos1) > POS_CMD_RESYNC_TH) ||
//            (fabsf_fast(setPos2 - prevSetPos2) > POS_CMD_RESYNC_TH);
//
//        uint8_t spdCmdJump =
//            (fabsf_fast(setSpd1 - prevSetSpd1) > SPD_CMD_RESYNC_TH) ||
//            (fabsf_fast(setSpd2 - prevSetSpd2) > SPD_CMD_RESYNC_TH);
//
//        // Choose resync conditions based on mode
//        uint8_t needResync = modeChanged;
//
//        if (mode == MP_MODE_POSITION) {
//            needResync = needResync || posCmdJump;
//        } else {
//            needResync = needResync || spdCmdJump;
//        }
//
//        if (needResync) {
//            // Re-seed once from measured state so profile starts from reality
//            mp_state1.pos_ref = curPos1;
//            mp_state1.vel_ref = curSpd1;   // set to measured if your speed is OK; otherwise set to mp_state1.vel_ref
//            mp_state2.pos_ref = curPos2;
//            mp_state2.vel_ref = curSpd2;
//        }
//
//        // ---- Step profile ----
//        MP_Output_t ref1, ref2;
//
//        if (modCtrl == MP_MODE_POSITION) {
//        	ref1 = MP_StepPosition(&mp_cfg1, &mp_state1, setPos1);
//        	ref2 = MP_StepPosition(&mp_cfg2, &mp_state2, setPos2);
//            DesiredPos1 = ref1.pos_next;
//            DesiredPos2 = ref2.pos_next;
//        } else {
//        	ref1 = MP_StepVelocity(&mp_cfg1, &mp_state1, setSpd1);
//        	ref2 = MP_StepVelocity(&mp_cfg2, &mp_state2, setSpd2);
//            DesiredSpeed1 = ref1.vel_next;
//            DesiredSpeed2 = ref2.vel_next;
//        }
//
//        // ---- Save previous command/mode for next loop ----
//		prevMode = mode;
//		prevSetPos1 = setPos1;  prevSetPos2 = setPos2;
//		prevSetSpd1 = setSpd1;  prevSetSpd2 = setSpd2;
//
//        // v_ref goes into your PID controller later
//        // PID_Update(v_ref, measured_speed);
//
//        vTaskDelayUntil(&lastWake, period);
//    }
//}

void StartPS2Task(void *argument)
{
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);

  (void)argument;

  // Init library (HAL-based, not RTOS-aware)
  PS2X_Init(&ps2, &hspi2);

  // Configure controller: analog + rumble enabled
  bool cfg_ok = PS2X_ConfigGamepad(&ps2, true, true);

  for (;;)
  {
    // Poll controller
    bool ok = cfg_ok && PS2X_ReadGamepad(&ps2, 0, 0);

    // Build snapshot
    PS2X_State st;
    st.tick   = osKernelGetTickCount();
    st.ok     = ok ? 1 : 0;
    st.id     = ps2.id;
    st.buttons= ps2.buttons;
    st.analog = ps2.analog_enabled ? 1 : 0;
    st.rx     = ps2.analog[PSS_RX];
    st.ry     = ps2.analog[PSS_RY];
    st.lx     = ps2.analog[PSS_LX];
    st.ly     = ps2.analog[PSS_LY];

    // Publish snapshot for other tasks
    if (osMutexAcquire(g_ps2_mutex, 2) == osOK) {
      g_ps2_state = st;
      osMutexRelease(g_ps2_mutex);
    }

    // Poll period
    vTaskDelayUntil(&last, period);   // 10ms is typical; adjust if you want
  }
}


void StartTeleopTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // ---- snapshot controller state ----
        PS2X_State st;
        bool got = false;
        if (osMutexAcquire(g_ps2_mutex, 2) == osOK) {
            st = g_ps2_state;
            osMutexRelease(g_ps2_mutex);
            got = true;
        }

        // Use the SAME clock as last_cmdvel_ms
        uint32_t now_ms = millis_void();

        // Default lock (manual path)
        float v_cmd = 0.0f;
        float w_cmd = 0.0f;
        bool enabled = false;

        // ---- ROS cmd watchdog: if no cmd_vel -> force manual ----
        if (autonomous_MODE) {
            uint32_t age_ros = now_ms - last_cmdvel_ms;
            age_ros_check = age_ros;
            if (age_ros > CMDVEL_TIMEOUT_MS) {
                autonomous_MODE = 0;
                num_timeout_ROScmd = num_timeout_ROScmd + 1;
                // optional: clear ROS commands to avoid stale reuse
                //Spd1_cmdfromROS = 0.0f;
                //Spd2_cmdfromROS = 0.0f;
            }
        }

        // ===== NEW: If autonomous active -> move immediately (no L1 needed) =====
        if (autonomous_MODE) {
            float v_l = Spd1_cmdfromROS;  // left
            float v_r = Spd2_cmdfromROS;  // right

            // Optional clamp per wheel
            v_l = clampf(v_l, -V_MAX, +V_MAX);
            v_r = clampf(v_r, -V_MAX, +V_MAX);

            // Keep your mapping (as in your autonomous block)
            DesiredSpeed1 = v_r;
            DesiredSpeed2 = v_l;

            osDelay(10);
            continue; // bypass manual logic below
        }

        // ---- decide enable (L1) and manual commands ----
        if (got && st.ok) {
            // timeout check for PS2
            uint32_t age = osKernelGetTickCount() - st.tick;
            if (age <= PS2_TIMEOUT_MS) {

                // L1 enable (manual only now)
                if (is_pressed(st.buttons, PSB_L1)) {
                    enabled = true;

                    // ===== Manual mode: keep your existing logic =====
                    int16_t rx = (int16_t)st.rx - 128;
                    int16_t ry = (int16_t)st.ry - 128;

                    if (abs(rx) < STICK_DEADBAND) rx = 0;
                    if (abs(ry) < STICK_DEADBAND) ry = 0;

                    bool analog_active = (rx != 0) || (ry != 0);

                    if (analog_active) {
                        float rx_n = (float)rx / 127.0f;      // -1..1
                        float ry_n = (float)ry / 127.0f;      // -1..1

                        v_cmd = (-ry_n) * V_MAX;
                        w_cmd = ( rx_n) * W_MAX;

                    } else {
                        bool up    = is_pressed(st.buttons, PSB_UP);
                        bool down  = is_pressed(st.buttons, PSB_DOWN);
                        bool left  = is_pressed(st.buttons, PSB_LEFT);
                        bool right = is_pressed(st.buttons, PSB_RIGHT);

                        if (up && !down && !left && !right) {
                            v_cmd = +V_STEP;  w_cmd = 0.0f;
                        } else if (down && !up && !left && !right) {
                            v_cmd = -V_STEP;  w_cmd = 0.0f;
                        } else if (left && !up && !down && !right) {
                            v_cmd = 0.0f;     w_cmd = -W_STEP;
                        } else if (right && !up && !down && !left) {
                            v_cmd = 0.0f;     w_cmd = +W_STEP;
                        } else {
                            v_cmd = 0.0f;     w_cmd = 0.0f;
                        }
                    }
                }
            }
        }

        // If not enabled or timeout => lock (0,0)
        if (!enabled) {
            v_cmd = 0.0f;
            w_cmd = 0.0f;
        }

        // ---- convert cmd_vel to wheel linear speeds (manual path only) ----
        float v_r = v_cmd + (w_cmd * WHEEL_BASE_M * 0.5f);
        float v_l = v_cmd - (w_cmd * WHEEL_BASE_M * 0.5f);

        v_r = clampf(v_r, -V_MAX, +V_MAX);
        v_l = clampf(v_l, -V_MAX, +V_MAX);

        DesiredSpeed1 = v_l;   // left
        DesiredSpeed2 = v_r;   // right

        osDelay(10);
    }
}

static inline void EncData_Publish(int32_t enc1, int32_t enc2, float v1, float v2, float v1d, float v2d)
{
  g_encSeq++; __DMB();
  g_encData.enc1 = enc1;
  g_encData.enc2 = enc2;
  g_encData.v1   = v1;
  g_encData.v2   = v2;
  g_encData.v1d = v1d;
  g_encData.v2d = v2d;
  __DMB(); g_encSeq++;
}

static inline void OdomData_Publish(float x, float y, float yaw)
{
  g_odomSeq++; __DMB();
  g_odomData.x   = x;
  g_odomData.y   = y;
  g_odomData.yaw = yaw;
  __DMB(); g_odomSeq++;
}

static inline void MotorData_Publish(float pwm1, float pwm2)
{
  g_motorSeq++; __DMB();
  g_motorData.pwm1 = pwm1;
  g_motorData.pwm2 = pwm2;
  __DMB(); g_motorSeq++;
}

static inline void EncData_Read(EncData_t* out)
{
  uint32_t s1, s2;
  do {
    s1 = g_encSeq; __DMB();
    *out = g_encData; __DMB();
    s2 = g_encSeq;
  } while ((s1 != s2) || (s1 & 1u));
}

static inline void OdomData_Read(OdomData_t* out)
{
  uint32_t s1, s2;
  do {
    s1 = g_odomSeq; __DMB();
    *out = g_odomData; __DMB();
    s2 = g_odomSeq;
  } while ((s1 != s2) || (s1 & 1u));
}

static inline void MotorData_Read(MotorData_t* out)
{
  uint32_t s1, s2;
  do {
    s1 = g_motorSeq; __DMB();
    *out = g_motorData; __DMB();
    s2 = g_motorSeq;
  } while ((s1 != s2) || (s1 & 1u));
}

void StartLCDTask(void *argument)
{
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);

  lcdInit();
  lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);

  UI_InitAndDrawOnce();
//  UI_LoadPIDFromFlashOnce();   // ✅ CHỈ LOAD 1 LẦN Ở ĐÂY (ngoài loop)

  static uint32_t t_ui = 0;

  for(;;)
  {
    uint16_t tx = g_touchState.x;
    uint16_t ty = g_touchState.y;
    uint8_t  p  = g_touchState.pressed ? 1 : 0;

    uint16_t x = (ty <= 319) ? (uint16_t)(319 - ty) : 0;
    uint16_t y = (tx <= 239) ? (uint16_t)(239 - tx) : 0;

    UI_OnTouch(x, y, p);

    if (HAL_GetTick() - t_ui > 100) {
      t_ui = HAL_GetTick();

      UI_UpdateEncoderPage(g_encData.enc1, g_encData.enc2,
                           g_encData.v1, g_encData.v2,
                           g_encData.v1d, g_encData.v2d,
                           g_motorData.pwm1, g_motorData.pwm2);

      if (UI_GetActivePage() == UI_PAGE_5) {
          Graph_Push(g_encData.v1, g_encData.v2, g_encData.v1d, g_encData.v2d);
          UI_VelGraph_UpdateCurvesOnly();
      }

      UI_UpdatePIDPage();   // ✅ GIỮ LẠI
      UI_UpdateOdomPage(g_odomData.x, g_odomData.y, g_odomData.yaw);

      }

    }

  vTaskDelayUntil(&last, period);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  printf("STACK OVERFLOW: %s\r\n", pcTaskName);
  taskDISABLE_INTERRUPTS();
  for(;;);
}

void StartMicroRosRxTask(void *argument)
{
    (void)argument;

    const uint64_t spin_timeout_ns = 1ULL * 1000000ULL; // 1 ms
    uint32_t last_tick = osKernelGetTickCount();
    const uint32_t period_ms = 10U; // 100 Hz

    for (;;)
    {
        if (!g_uros_connected || g_request_reconnect)
        {
            osDelay(20);
            last_tick = osKernelGetTickCount();
            continue;
        }

        if (osMutexAcquire(g_uros_mutex, 5) == osOK)
        {
            rcl_ret_t rc = rclc_executor_spin_some(&g_executor, spin_timeout_ns);
            osMutexRelease(g_uros_mutex);

            if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT)
            {
                dbg_rx_spin_err_cnt++;
                g_request_reconnect = true;
            }
        }

        uint32_t now = osKernelGetTickCount();
        uint32_t elapsed = now - last_tick;
        if (elapsed < period_ms)
        {
            osDelay(period_ms - elapsed);
        }
        last_tick = osKernelGetTickCount();
    }
}

void StartMicroRosOdomPubTask(void *argument)
{
    (void)argument;

    uint32_t last_tick = osKernelGetTickCount();
    const uint32_t period_ms = 10U;   // 100 Hz
    // const uint32_t period_ms = 20U; // 50 Hz
    // const uint32_t period_ms = 33U; // ~30 Hz

    for (;;)
    {
        if (!g_uros_connected || g_request_reconnect)
        {
            osDelay(20);
            last_tick = osKernelGetTickCount();
            continue;
        }

        if (osMutexAcquire(g_uros_mutex, 10) == osOK)
        {
            microros_publish_odom();
            dbg_pub_odom_cnt++;
            osMutexRelease(g_uros_mutex);
        }

        uint32_t now = osKernelGetTickCount();
        uint32_t elapsed = now - last_tick;
        if (elapsed < period_ms)
        {
            osDelay(period_ms - elapsed);
        }
        last_tick = osKernelGetTickCount();
    }
}
/* USER CODE END Application */

