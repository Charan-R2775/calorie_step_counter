/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (minimal, cleaned)
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "lsm303dlhc_acc.h"
#include "cmsis_os2.h"
#include "arm_math.h"   // CMSIS-DSP
#include <stdlib.h>     // atoi, strtof
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { STATE_IDLE = 0, STATE_ACTIVE = 1 } run_state_t;

typedef enum { BTN_EV_DOWN = 0, BTN_EV_UP = 1 } btn_ev_t;
typedef struct { btn_ev_t ev; uint32_t t_ms; } btn_msg_t;

typedef struct {
  char topic[64];
  char payload[256];
} pub_req_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Behavior when STATE == IDLE ---
#define COUNT_STEPS_WHEN_IDLE   0   // don't count steps in IDLE
#define ACCUMULATE_WHEN_IDLE    0   // don't add distance/kcal in IDLE

#define MIN_PEAK_ABS  0.10f   // ~0.10 g band-passed abs minimum


// Button timings (ms)
#define BTN_DEBOUNCE_MS      150u
#define BTN_LONG_MS         2000u
#define SNAPSHOT_MIN_SPAN    200u

// Step detector tuning (50 Hz)
#define ACC_ALPHA           0.90f
#define TH_HIGH_G           0.35f
#define TH_LOW_G            0.15f


// Defaults (can be switched later to remote-config values)
#define STRIDE_LENGTH_M     0.75f
#define KCAL_PER_STEP       0.05f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;


/* USER CODE BEGIN PV */
// RTOS objects
static osMessageQueueId_t qPub;
static osMessageQueueId_t qBtn;
static osSemaphoreId_t    semUart;  // UART TX lock

// App state
static volatile run_state_t g_state = STATE_IDLE;

static volatile uint32_t step_count = 0;
static volatile float    distance_m = 0.0f;
static volatile float    cal_kcal   = 0.0f;

// Button helpers
static volatile uint32_t g_btn_last_edge = 0;
static volatile uint8_t  g_btn_down_seen = 0;

// Step detector internals
static float    g_est = 1.0f;
static uint32_t last_step_ms = 0;

// ---- user-configurable params (defaults) ----
static volatile uint16_t g_height_cm = 170;    // default
static volatile uint16_t g_mass_kg   = 70;     // default
static volatile float    g_stride_m  = 0.75f;  // will be overridden/derived
static volatile uint8_t  g_stride_user_set = 0; // 0=derive from height

static volatile float    g_kcal_per_step = 0.05f;

// UART RX line collector (for CFG key=value coming from ESP32)
static uint8_t  rx_ch;
static char     rx_line[128];
static size_t   rx_len = 0;

// ===== DSP config (50 Hz sampling) =====
#define FS_HZ          50.0f
#define DT_SEC         (1.0f/FS_HZ)

// 0.5–3.0 Hz band using HPF(0.5Hz, Q=0.707) then LPF(3.0Hz, Q=0.707)
#define HPF_FC_HZ      0.50f
#define HPF_Q          0.707f
#define LPF_FC_HZ      3.00f
#define LPF_Q          0.707f

// Envelope and thresholds
#define ENV_ALPHA      0.98f
#define K_TH_HI        1.50f     // was 1.20f
#define K_TH_LO        0.55f      // was 0.50f
#define REFRACT_MS     420u      // was 300u

// Cadence/speed smoothing
#define CAD_EWMA       0.20f
#define SPD_EWMA       0.10f

// Envelope & kinematics
static float     env_abs = 0.05f;
static uint8_t   bp_armed = 0;
static uint32_t  last_step_ms_dsp = 0;

static volatile float cadence_spm = 0.0f;  // smoothed cadence (steps/min)
static volatile float speed_mps   = 0.0f;  // smoothed speed (m/s)
// Peak detection helpers (to avoid multiple counts per step)
static float   prev_y_bp  = 0.0f;
static uint8_t was_rising = 0;


// --- Simple filter state (bypassing CMSIS biquads for debug/robustness) ---
//static float mag_base = 1.0f;   // EMA baseline of |acc| ~ gravity
//static float y_lp     = 0.0f;   // one-pole LPF output

// Pre-tuned EMA gains for fs=50 Hz
#define HPF_ALPHA   0.061f   // ~0.5 Hz baseline tracker
#define LPF_ALPHA   0.314f   // ~3.0 Hz smoothing


// --- CMSIS-DSP filter state ---
static arm_biquad_casd_df1_inst_f32 iir_hpf;
static arm_biquad_casd_df1_inst_f32 iir_lpf;

// One stage each (2nd-order); CMSIS needs 5 coeffs/stage and 4 state values/stage
static float32_t hpf_coeffs[5];
static float32_t lpf_coeffs[5];
static float32_t hpf_state[4];
static float32_t lpf_state[4];


/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
// ===== Command-driven session helpers =====
// --- forward decls used by action_* ---
static void publish_snapshot(void);
static void publish_status(const char *state, bool include_summary);
static void dsp_init_filters(void);


static void action_start(void)
{
  if (g_state == STATE_ACTIVE) return;

  __disable_irq();
  step_count = 0;
  distance_m = 0.0f;
  cal_kcal   = 0.0f;
  g_est = 1.0f;
  last_step_ms = 0;
  // DSP state
  bp_armed         = 0;
  last_step_ms_dsp = 0;
  cadence_spm      = 0.0f;
  speed_mps        = 0.0f;
  env_abs          = 0.05f;
  __enable_irq();

  dsp_init_filters();

  g_state = STATE_ACTIVE;
  publish_status("ACTIVE", false);
}

static void action_stop(void)
{
  if (g_state == STATE_IDLE) return;
  g_state = STATE_IDLE;

  publish_status("IDLE", true);

}


static void action_snapshot(void)
{
  publish_snapshot();
}


static void cfg_recompute(void);
static void cfg_publish_applied(void);
static void handle_cfg_line(const char *line);
static void uart_rx_start_it(void);

static void handle_cmd_line(const char *line)
/* expects: "CMD <word>" where <word> is: start | stop | snapshot | snap | summary */
{
  const char *p = line;
  if (strncmp(p, "CMD ", 4) == 0) p += 4;

  char tok[24]={0};
  int i=0;
  while (*p && *p!='\r' && *p!='\n' && *p!=' ' && i<(int)sizeof(tok)-1) tok[i++]=*p++;
  tok[i]='\0';

  bool ok=false;
  if      (strcmp(tok,"start")==0)    { action_start();    ok=true; }
  else if (strcmp(tok,"stop")==0)     { action_stop();     ok=true; }
  else if (strcmp(tok,"snapshot")==0) { action_snapshot(); ok=true; }

//  // optional ACK
//  pub_req_t ack;
//  snprintf(ack.topic, sizeof(ack.topic), "suryasurya/feeds/stm32f4-01.cmd.ack");
//  snprintf(ack.payload, sizeof(ack.payload), "%s:%s", tok, ok?"ok":"unknown");
//  osMessageQueuePut(qPub, &ack, 0, 0);
}

// Tasks
static void Task_MqttPub(void *arg);
static void Task_AccelStep(void *arg);
static void Task_ButtonSM(void *arg);

// Helpers
static bool uart_send_line(const char *line);
static void publish_snapshot(void);
static void publish_status(const char *state, bool include_summary);

// Build RBJ biquad (Tustin) coefficients, normalized for CMSIS-DF1: {b0,b1,b2,a1,a2}
// type: 0=LPF, 1=HPF
static void rbj_make_biquad_lp_hp(float fs, float f0, float Q, int type,
                                  float32_t out[5])
{
  const float w0 = 2.0f * (float)M_PI * (f0 / fs);
  const float cw = arm_cos_f32(w0);
  const float sw = arm_sin_f32(w0);
  const float alpha = sw / (2.0f * Q);

  float b0,b1,b2,a0,a1,a2;

  if (type == 0) { // LPF
    b0 = (1.0f - cw) * 0.5f;
    b1 = (1.0f - cw);
    b2 = (1.0f - cw) * 0.5f;
    a0 = 1.0f + alpha;
    a1 = -2.0f * cw;
    a2 = 1.0f - alpha;
  } else {         // HPF
    b0 =  (1.0f + cw) * 0.5f;
    b1 = -(1.0f + cw);
    b2 =  (1.0f + cw) * 0.5f;
    a0 =  1.0f + alpha;
    a1 = -2.0f * cw;
    a2 =  1.0f - alpha;
  }

  // Normalize by a0; CMSIS expects feedback coeffs with NEGATED sign.
  out[0] = b0 / a0;      // b0
  out[1] = b1 / a0;      // b1
  out[2] = b2 / a0;      // b2
  out[3] = -(a1 / a0);   // a1  (NEGATED)
  out[4] = -(a2 / a0);   // a2  (NEGATED)
}

static void dsp_init_filters(void)
{
  rbj_make_biquad_lp_hp(FS_HZ, HPF_FC_HZ, HPF_Q, 1, hpf_coeffs); // HPF
  rbj_make_biquad_lp_hp(FS_HZ, LPF_FC_HZ, LPF_Q, 0, lpf_coeffs); // LPF

  memset(hpf_state, 0, sizeof(hpf_state));
  memset(lpf_state, 0, sizeof(lpf_state));


  arm_biquad_cascade_df1_init_f32(&iir_hpf, 1, hpf_coeffs, hpf_state);
  arm_biquad_cascade_df1_init_f32(&iir_lpf, 1, lpf_coeffs, lpf_state);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*        */
static void cfg_recompute(void)
{
  float stride = g_stride_user_set ? g_stride_m
                                   : (0.414f * ((float)g_height_cm / 100.0f));
  if (stride < 0.40f) stride = 0.40f;
  if (stride > 1.20f) stride = 1.20f;
  g_stride_m = stride;

  float scale = ((float)g_mass_kg) / 70.0f;
  if (scale < 0.4f) scale = 0.4f;
  if (scale > 2.0f) scale = 2.0f;
  g_kcal_per_step = 0.05f * scale;
}

static void cfg_publish_applied(void)
{
  char line[96];
  uint32_t stride_centi = (uint32_t)(g_stride_m * 100.0f + 0.5f);
  snprintf(line, sizeof(line),
    "height=%u,mass=%u,stride=%lu.%02lu%s",
    (unsigned)g_height_cm, (unsigned)g_mass_kg,
    stride_centi/100, stride_centi%100,
    g_stride_user_set ? "" : " (derived)");

  pub_req_t req;
  snprintf(req.topic, sizeof(req.topic),
           "suryasurya/feeds/stm32f4-01.cfg-dot-applied");
  snprintf(req.payload, sizeof(req.payload), "%s", line);
  osMessageQueuePut(qPub, &req, 0, 0);
}

static void handle_cfg_line(const char *line)
{
  // expect: "CFG key=value"
  if (strncmp(line, "CFG ", 4) != 0) return;
  const char *p = line + 4;

  char key[24]={0}, val[24]={0};
  int i=0;

  while (*p && *p!='=' && i<(int)sizeof(key)-1) key[i++]=*p++;
  key[i]='\0';
  if (*p != '=') return;
  p++;

  i=0;
  while (*p && *p!='\r' && *p!='\n' && i<(int)sizeof(val)-1) val[i++]=*p++;
  val[i]='\0';

  bool ok=false;
  if (strcmp(key,"height_cm")==0) {
    int v = atoi(val);
    if (v>=120 && v<=210) { g_height_cm=(uint16_t)v; ok=true; }
  } else if (strcmp(key,"mass_kg")==0) {
    int v = atoi(val);
    if (v>=30 && v<=160) { g_mass_kg=(uint16_t)v; ok=true; }
  } else if (strcmp(key,"stride_m")==0) {
    float f = strtof(val, NULL);
    if (f>=0.40f && f<=1.20f) { g_stride_m=f; g_stride_user_set=1; ok=true; }
  }

  if (ok) {
    __disable_irq(); cfg_recompute(); __enable_irq();
    cfg_publish_applied();
  } else {
    // optional rejection message
    pub_req_t req;
    snprintf(req.topic, sizeof(req.topic),
             "suryasurya/feeds/stm32f4-01.cfg-dot-applied");
    snprintf(req.payload, sizeof(req.payload),
             "rejected %s=%s", key, val);
    osMessageQueuePut(qPub, &req, 0, 0);
  }
}

// ---- UART RX interrupt line collector (USART2 from ESP32) ----
static void uart_rx_start_it(void)
{
  HAL_UART_Receive_IT(&huart2, &rx_ch, 1);
}

// HAL callback called on each received byte (IRQ context)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    char c = (char)rx_ch;
    if (c == '\r') {
      // ignore
    } else if (c == '\n') {
    	// terminate and process
    	if (rx_len < sizeof(rx_line)) rx_line[rx_len] = '\0';
    	if (rx_len > 0) {
    	  if (strncmp(rx_line, "CFG ", 4) == 0) {
    	    handle_cfg_line(rx_line);
    	  } else if (strncmp(rx_line, "CMD ", 4) == 0) {
    	    handle_cmd_line(rx_line);
    	  }
    	}
    	rx_len = 0;

    } else {
      if (rx_len < sizeof(rx_line)-1) {
        rx_line[rx_len++] = c;
      } else {
        // overflow -> reset
        rx_len = 0;
      }
    }
    // re-arm
    HAL_UART_Receive_IT(&huart2, &rx_ch, 1);
  }
}
/*       */


// UART helper: send a full line to ESP32 ("PUB <topic> <payload>\n")
static bool uart_send_line(const char *line)
{
  if (osSemaphoreAcquire(semUart, 200) != osOK) return false;
  HAL_StatusTypeDef st = HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)strlen(line), 200);
  osSemaphoreRelease(semUart);
  return (st == HAL_OK);
}

static void publish_snapshot(void)
{
  uint32_t steps; float dist, kcal;
  __disable_irq();
  steps = step_count; dist = distance_m; kcal = cal_kcal;
  __enable_irq();

  pub_req_t req;

  // steps
  snprintf(req.topic, sizeof(req.topic), "suryasurya/feeds/stm32f4-01.steps");
  snprintf(req.payload, sizeof(req.payload), "%lu", (unsigned long)steps);
  osMessageQueuePut(qPub, &req, 0, 0);

  // distance (2dp without float printf dependency)
  uint32_t dist_centi = (uint32_t)(dist * 100.0f + 0.5f);
  snprintf(req.topic, sizeof(req.topic), "suryasurya/feeds/stm32f4-01.distance");
  snprintf(req.payload, sizeof(req.payload), "%lu.%02lu", dist_centi/100, dist_centi%100);
  osMessageQueuePut(qPub, &req, 0, 0);

  // calories (2dp)
  uint32_t kcal_centi = (uint32_t)(kcal * 100.0f + 0.5f);
  snprintf(req.topic, sizeof(req.topic), "suryasurya/feeds/stm32f4-01.calories");
  snprintf(req.payload, sizeof(req.payload), "%lu.%02lu", kcal_centi/100, kcal_centi%100);
  osMessageQueuePut(qPub, &req, 0, 0);
}

static void publish_status(const char *state, bool include_summary)
{
  pub_req_t req;
  snprintf(req.topic, sizeof(req.topic), "suryasurya/feeds/stm32f4-01.state");
  snprintf(req.payload, sizeof(req.payload), "%s", state);
  osMessageQueuePut(qPub, &req, 0, 0);

  if (include_summary) publish_snapshot();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  dsp_init_filters();



  cfg_recompute();          // compute stride & kcal from defaults on boot
  uart_rx_start_it();       // begin receiving CFG lines from ESP32

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  semUart = osSemaphoreNew(1, 1, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  qPub = osMessageQueueNew(8, sizeof(pub_req_t), NULL);
  qBtn = osMessageQueueNew(8, sizeof(btn_msg_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t acc_attr = { .name="ACC",  .priority=osPriorityAboveNormal, .stack_size=512*4 };
  const osThreadAttr_t btn_attr = { .name="BTN",  .priority=osPriorityLow,         .stack_size=256*4 };
  const osThreadAttr_t pub_attr = { .name="MQTT", .priority=osPriorityNormal,      .stack_size=512*4 };

  (void)osThreadNew(Task_AccelStep, NULL, &acc_attr);
  (void)osThreadNew(Task_ButtonSM,  NULL, &btn_attr);
  (void)osThreadNew(Task_MqttPub,   NULL, &pub_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Called on button press (PA0 EXTI)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_0) return;

  uint32_t now = HAL_GetTick();
  if ((now - g_btn_last_edge) < BTN_DEBOUNCE_MS) return; // debounce
  g_btn_last_edge = now;

  // Read pin level to know DOWN/UP (adjust if pull-up)
  GPIO_PinState level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  btn_msg_t msg;

  if (level == GPIO_PIN_SET) {             // button DOWN
    msg.ev = BTN_EV_DOWN;
    msg.t_ms = now;
    g_btn_down_seen = 1;
    (void)osMessageQueuePut(qBtn, &msg, 0, 0);
  } else {                                 // button UP
    if (!g_btn_down_seen) return;          // ignore stray release
    msg.ev = BTN_EV_UP;
    msg.t_ms = now;
    g_btn_down_seen = 0;
    (void)osMessageQueuePut(qBtn, &msg, 0, 0);
  }
}

/* ================== TASK: MQTT Publisher ================== */
void Task_MqttPub(void *arg)
{
  (void)arg;
  pub_req_t req;
  char line[256];

  for(;;){
    if (osMessageQueueGet(qPub, &req, NULL, osWaitForever) != osOK) continue;
    // Bridge protocol to ESP32: "PUB <topic> <payload>\n"
    (void)snprintf(line, sizeof(line), "PUB %s %s\n", req.topic, req.payload);
    (void)uart_send_line(line);
  }
}

/* ================== TASK: Button State Machine ================== */
void Task_ButtonSM(void *arg)
{
  (void)arg;
  btn_msg_t msg;
  uint32_t last_down = 0;
  uint32_t last_snapshot_ms = 0;

  for(;;){
    if (osMessageQueueGet(qBtn, &msg, NULL, osWaitForever) != osOK) continue;

    if (msg.ev == BTN_EV_DOWN) {
      last_down = msg.t_ms;
      continue;
    }

    if (msg.ev == BTN_EV_UP) {
      if (last_down == 0) continue; // stray
      uint32_t press_ms = msg.t_ms - last_down;
      last_down = 0;

      if (press_ms >= BTN_LONG_MS) {
        if (g_state == STATE_IDLE) {
          // ===== IDLE -> ACTIVE: reset counters & DSP state =====
          __disable_irq();
          step_count = 0;
          distance_m = 0.0f;
          cal_kcal   = 0.0f;
          g_est = 1.0f;
          last_step_ms = 0;

          // Reset DSP-side detector so hysteresis/refractory don't carry over
          bp_armed         = 0;        // global
          last_step_ms_dsp = 0;        // global
          cadence_spm      = 0.0f;     // global
          speed_mps        = 0.0f;     // global
          env_abs          = 0.05f;    // global
          __enable_irq();

          dsp_init_filters();

          g_state = STATE_ACTIVE;
          publish_status("ACTIVE", false);  // start fresh
        } else {
          // ===== ACTIVE -> IDLE: keep totals; just stop counting =====
          g_state = STATE_IDLE;
          publish_status("IDLE", true);     // optional final snapshot
          // (Do NOT clear step_count/distance/cal_kcal here)
        }
      }
else {
        // SHORT PRESS: snapshot (rate limited)
        uint32_t now = HAL_GetTick();
        if ((now - last_snapshot_ms) >= SNAPSHOT_MIN_SPAN) {
          last_snapshot_ms = now;
          publish_snapshot();
        }
      }
    }
  }
}

/* ================== TASK: Accel Step Detector ================== */
void Task_AccelStep(void *arg)
{
  (void)arg;

  while (!lsm303_acc_init_50hz()) { osDelay(500); }

  uint32_t start_ms = HAL_GetTick();
  uint32_t last_step_t = 0;

  for (;;)
  {
    osDelay(20); // 50 Hz

    // === 1) sample ===
    float ax_mg, ay_mg, az_mg;
    if (!lsm303_acc_read_mg(&ax_mg, &ay_mg, &az_mg)) continue;
    float ax = ax_mg * 0.001f;
    float ay = ay_mg * 0.001f;
    float az = az_mg * 0.001f;

    // magnitude
    float mag = sqrtf(ax*ax + ay*ay + az*az);

    float y1, y_bp;
    arm_biquad_cascade_df1_f32(&iir_hpf, &mag, &y1,  1); // HPF -> y1
    arm_biquad_cascade_df1_f32(&iir_lpf, &y1,  &y_bp,1); // LPF -> y_bp




    // === Envelope from band-passed signal ===
    float ay_bp = fabsf(y_bp);
    env_abs = ENV_ALPHA * env_abs + (1.0f - ENV_ALPHA) * ay_bp;
    if (env_abs < 0.01f) env_abs = 0.01f;   // was 0.02f

    // === Peak-direction logic (count on local maxima only) ===
    float dy           = y_bp - prev_y_bp;
    uint8_t rising_now = (dy > 0.0f);
    uint8_t was_rising_prev = was_rising;   // <-- snapshot previous state

    uint32_t now = HAL_GetTick();
    if ((now - start_ms) < 800u) {
      // settle: update state then continue
      was_rising = rising_now;
      prev_y_bp  = y_bp;
      continue;
    }


    // What we are allowed to update in this tick
//    bool count_ok = (g_state == STATE_ACTIVE) || (COUNT_STEPS_WHEN_IDLE != 0);
    bool accum_ok = (g_state == STATE_ACTIVE) || (ACCUMULATE_WHEN_IDLE != 0);


    // Idle decay (keep processing even in IDLE)
    if (g_state != STATE_ACTIVE) {
      cadence_spm *= 0.98f;
      speed_mps   *= 0.98f;
    }



    // === 4) adaptive thresholds + hysteresis ===
    float th_hi = K_TH_HI * env_abs;
    float th_lo = K_TH_LO * env_abs;
    #define ARM_TIMEOUT_MS 1500u

    if (!bp_armed) {
      // Count only when we just turned from rising->falling (local max) and above th_hi,
      // and also respect the refractory time.
    	if (was_rising_prev && !rising_now &&
    	    (y_bp > th_hi) &&
    	    (y_bp > MIN_PEAK_ABS) &&
    	    (now - last_step_ms_dsp) > REFRACT_MS) {

        last_step_ms_dsp = now;

        uint32_t dt_ms = (last_step_t == 0) ? 0 : (now - last_step_t);
        last_step_t = now;

        __disable_irq();
        if ((g_state == STATE_ACTIVE) || (COUNT_STEPS_WHEN_IDLE != 0)) {
          step_count++;
        }
        __enable_irq();

        if (dt_ms > 200 && dt_ms < 2000) {
          float cad_inst = 60000.0f / (float)dt_ms;
          cadence_spm = cadence_spm + CAD_EWMA * (cad_inst - cadence_spm);
        }

        // dynamic stride (unchanged)
        float stride = g_stride_user_set ? g_stride_m
                                         : (0.414f * ((float)g_height_cm / 100.0f));
        if (!g_stride_user_set) {
          float gain = 1.0f + 0.30f * ((cadence_spm - 100.0f) / 100.0f);
          if (gain < 0.8f) gain = 0.8f;
          if (gain > 1.3f) gain = 1.3f;
          stride *= gain;
          if (stride < 0.40f) stride = 0.40f;
          if (stride > 1.30f) stride = 1.30f;

          if ((g_state == STATE_ACTIVE) || (ACCUMULATE_WHEN_IDLE != 0)) {
            g_stride_m = stride;
          }
        }

        if ((g_state == STATE_ACTIVE) || (ACCUMULATE_WHEN_IDLE != 0)) {
          __disable_irq();
          distance_m += g_stride_m;
          __enable_irq();
        }

        bp_armed = 1;   // wait for low crossing or timeout
      }
    } else {
    	 const float REARM_MARGIN = 0.02f;  // small absolute margin (~0.02 g)
    	  if (y_bp < (th_lo - REARM_MARGIN)) {
    	    bp_armed = 0;
    	  } else if ((now - last_step_ms_dsp) > ARM_TIMEOUT_MS) {
    	    bp_armed = 0;
      }
    }



    // === 5) speed estimate & calories (MET) ===
    float v_est = (g_stride_m * cadence_spm) / 120.0f;  // m/s
    if (v_est < 0.0f) v_est = 0.0f;
    if (v_est > 4.5f) v_est = 4.5f;
    speed_mps = speed_mps + SPD_EWMA * (v_est - speed_mps);

    float v = speed_mps;
    float MET;
    if (v <= 1.0f)       MET = 2.0f + 1.5f * (v / 1.0f);
    else if (v <= 1.8f)  MET = 3.5f + 3.0f * (v - 1.0f);
    else if (v <= 3.0f)  MET = 6.0f + 2.0f * (v - 1.8f);
    else                 MET = 9.0f + 1.5f * (v - 3.0f);

    float kcal_per_min = MET * 3.5f * (float)g_mass_kg / 200.0f;
    float kcal_tick    = (kcal_per_min / 60.0f) * DT_SEC;

    // only add calories if allowed (ACTIVE) AND actually moving (>5 steps/min)
    if (accum_ok && cadence_spm > 5.0f) {
      __disable_irq();
      cal_kcal += kcal_tick;
      __enable_irq();
    }
    // update edge state for next sample
    was_rising = rising_now;
    prev_y_bp  = y_bp;

  }

}


/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
