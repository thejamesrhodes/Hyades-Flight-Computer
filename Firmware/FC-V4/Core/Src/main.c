/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Hyades flight computer (FC-V4 board, STM32F411CEU @ 96 MHz).
  *          QMI8658A IMU -> invariant EKF (attitude + gyro bias) ->
  *          geometric SO(3) + INDI attitude control -> 4 fin servos.
  *          ATGM336H GNSS on USART1 (receive-only, status/logging only).
  *          Single 100 Hz cyclic-executive frame paced by TIM5.
  *
  *   Boot:  clocks -> peripherals -> GPS line check -> IMU init/self-test
  *          -> gyro calibrate (still) -> seed from accel -> control self-check -> settle
  *   Frame: IMU read -> GPS poll -> IEKF -> control -> fins -> telemetry
  *          -> LEDs -> watchdog
  *
  * Project rules (see FC-V6 main.c header): determinism over convenience,
  * safety over performance, explicit over implicit. Estimator and controller
  * are pure (no I/O); this file owns all I/O and sequencing. No float printf
  * (Log_Fx), no blocking calls in the frame, fins centred on any fault.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"
#include "sched.h"
#include "imu.h"
#include "iekf.h"
#include "control.h"
#include "pwm.h"
#include "telemetry.h"
#include "usb_log.h"
#include "gps.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static IEKF_State  iekf;
static Controller  ctrl;
static GyroCal     gcal;
static AccelCal    acal;
static MagCal      mcal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static void MX_IWDG_Init(void) __attribute__((unused));  /* used only if CFG_ENABLE_IWDG */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- Status LEDs -----------------------------------------------------------
 *  LED_SYS = PA1 (D8)                      LED_GPS = PA0 (D6)
 *    solid ......... booting/calibrating     solid ......... position fix
 *    4 Hz blink .... settling                1 Hz blip ..... NMEA, no fix
 *    1 Hz blip ..... running, healthy        off ........... no NMEA
 *    10 Hz flicker . IMU fail / overrun (held 1 s)
 *  Both alternating = fault_halt (fins centred). Both solid = CPU fault.
 * -------------------------------------------------------------------------- */
#define LED_SYS_PORT  LED_2_GPIO_Port
#define LED_SYS_PIN   LED_2_Pin
#define LED_GPS_PORT  GPIOA
#define LED_GPS_PIN   GPIO_PIN_0

typedef enum { VS_BOOT = 0, VS_SETTLE, VS_RUN, VS_DEGRADED } VehState;

static void led_set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void leds_update(VehState vs, uint32_t t_ms)
{
    bool sys;
    switch (vs) {
        case VS_BOOT:     sys = true;                       break;
        case VS_SETTLE:   sys = ((t_ms / 125u) & 1u) != 0u; break;
        case VS_RUN:      sys = (t_ms % 1000u) < 100u;      break;
        case VS_DEGRADED:
        default:          sys = ((t_ms / 50u) & 1u) != 0u;  break;
    }
    led_set(LED_SYS_PORT, LED_SYS_PIN, sys);

    bool g = false;
#if CFG_ENABLE_GPS
    if (GPS_HasFix(GPS_TIMEOUT_MS))     { g = true; }
    else if (GPS_Alive(GPS_TIMEOUT_MS)) { g = (t_ms % 1000u) < 80u; }
#endif
    led_set(LED_GPS_PORT, LED_GPS_PIN, g);
}

/* Unrecoverable boot fault: fins centred, LEDs alternate. The IWDG (if
 * enabled) is not kicked here, so it resets the MCU. */
static void fault_halt(const char *why)
{
    PWM_CentreFins();
    USB_Log("[FATAL] %s - halting, fins centred\r\n", why);
    led_set(LED_SYS_PORT, LED_SYS_PIN, true);
    led_set(LED_GPS_PORT, LED_GPS_PIN, false);
    for (;;) {
        HAL_GPIO_TogglePin(LED_SYS_PORT, LED_SYS_PIN);
        HAL_GPIO_TogglePin(LED_GPS_PORT, LED_GPS_PIN);
        HAL_Delay(120);
    }
}

/* Average n accel (+mag) samples for the attitude seed. Airframe still. */
static void seed_sample(float accel_avg[3], float mag_avg[3], uint32_t n)
{
    uint32_t na = 0u, nm = 0u;
    for (uint32_t k = 0u; k < 3u; k++) { accel_avg[k] = 0.0f; mag_avg[k] = 0.0f; }
    for (uint32_t i = 0u; i < n; i++) {
        ImuSample s;
        IMU_ReadBody(&hi2c1, &gcal, &acal, &mcal, &s);
        for (uint32_t k = 0u; k < 3u; k++) {
            if (s.accel_ok) { accel_avg[k] += s.accel[k]; }
            if (s.mag_ok)   { mag_avg[k]   += s.mag[k]; }
        }
        if (s.accel_ok) { na++; }
        if (s.mag_ok)   { nm++; }
        HAL_Delay(5);
    }
    for (uint32_t k = 0u; k < 3u; k++) {
        if (na > 0u) { accel_avg[k] /= (float)na; }
        if (nm > 0u) { mag_avg[k]   /= (float)nm; }
    }
}

/* Log four fin commands as "+0.123 -0.456 ..." with a prefix. */
static void log_fins(const char *prefix, const float fin[4])
{
    char b[4][LOG_FX_LEN];
    USB_Log("%s %s %s %s %s\r\n", prefix, Log_Fx(b[0], fin[0], 3u), Log_Fx(b[1], fin[1], 3u),
            Log_Fx(b[2], fin[2], 3u), Log_Fx(b[3], fin[3], 3u));
}

#if CFG_ENABLE_BOOTVERIFY
/* Run the full control path (SO(3) error -> INDI -> allocation) on synthetic
 * attitudes and log the fin commands, to check fin signs/order. Uses a scratch
 * copy of the controller; no servo motion. Expected for a +error (restoring):
 *   +pitch(X): top -, bottom +   +yaw(Y): right -, left +   +roll(Z): all -
 * With CTRL_ENABLE_PITCHYAW = 0 the pitch/yaw cases must read all zero. */
static void boot_verify(void)
{
    USB_Log("[BOOT] Control-path check, fin order 0=top 1=right 2=bottom 3=left\r\n");
    const float d = 0.20f;                         /* ~11.5 deg synthetic error */
    const float c = cosf(0.5f * d), sn = sinf(0.5f * d);
    const float w0[3] = { 0.0f, 0.0f, 0.0f };
    const struct { const char *name; float q[4]; } cases[3] = {
        { "[BOOT]   +pitch(X) fins:", { c, sn, 0.0f, 0.0f } },
        { "[BOOT]   +yaw(Y)   fins:", { c, 0.0f, sn, 0.0f } },
        { "[BOOT]   +roll(Z)  fins:", { c, 0.0f, 0.0f, sn } },
    };
    for (uint32_t k = 0u; k < 3u; k++) {
        Controller scratch;
        Ctrl_Init(&scratch);                       /* fresh filters and integrator */
        scratch.active = true;
        float fin[4];
        Ctrl_Update(&scratch, cases[k].q, w0, FRAME_DT_S, fin);
        log_fins(cases[k].name, fin);
    }
}
#endif

#if CFG_ENABLE_SERVO_SWEEP
/* Boot servo sweep: each axis to +/- full deflection, then each fin alone.
 * Checks fin direction, travel and centring (NOT the IMU remap). Airframe
 * RESTRAINED, fin arc CLEAR. Boot only; blocking delays are acceptable. */
static void servo_sweep(void)
{
    static const float pat[3][4] = {
        { +1.0f,  0.0f, -1.0f,  0.0f },            /* +pitch(X): top +, bottom - */
        {  0.0f, +1.0f,  0.0f, -1.0f },            /* +yaw(Y):   right +, left - */
        { +1.0f, +1.0f, +1.0f, +1.0f },            /* +roll(Z):  all +           */
    };
    static const char *const name[3] = { "pitch", "yaw", "roll" };

    USB_Log("[SWEEP] Servo sweep: RESTRAIN airframe, keep fin arc clear\r\n");
    PWM_CentreFins();
    HAL_Delay(SWEEP_STEP_MS);
    for (uint32_t a = 0u; a < 3u; a++) {
        for (int32_t sgn = 1; sgn >= -1; sgn -= 2) {
            float fin[4];
            for (uint8_t i = 0u; i < 4u; i++) {
                fin[i] = (float)sgn * pat[a][i];
                PWM_SetFin(i, fin[i]);
            }
            USB_Log("[SWEEP] %s %s\r\n", name[a], (sgn > 0) ? "+max" : "-max");
            HAL_Delay(SWEEP_HOLD_MS);
            PWM_CentreFins();
            HAL_Delay(SWEEP_STEP_MS);
        }
    }
    for (uint8_t k = 0u; k < 4u; k++) {            /* one fin at a time */
        USB_Log("[SWEEP] fin %u alone +max\r\n", (unsigned)k);
        PWM_SetFin(k, 1.0f);
        HAL_Delay(SWEEP_HOLD_MS);
        PWM_CentreFins();
        HAL_Delay(SWEEP_STEP_MS);
    }
    USB_Log("[SWEEP] Done, fins centred\r\n");
}
#endif

#if CFG_ENABLE_GPS
/* Boot GPS check: line activity first (wiring), then whatever NMEA arrived. */
static void gps_boot_check(void)
{
    uint32_t e10 = 0u, e15 = 0u;
    GPS_LineActivity(1100u, &e10, &e15);
    GPS_Poll(4096u);
    USB_Log("[BOOT] GPS line edges: PA10(RX)=%lu PA15=%lu | rx %lu B, %lu NMEA, %lu bad\r\n",
            (unsigned long)e10, (unsigned long)e15, (unsigned long)gps.bytes,
            (unsigned long)gps.sentences, (unsigned long)gps.cksum_err);
    if (e10 == 0u && e15 > 0u) {
        USB_Log("[BOOT] GPS: data on PA15 only -> module TX/RX crossed, swap the two wires\r\n");
    } else if (e10 == 0u) {
        USB_Log("[BOOT] GPS: no activity on either pin -> check module TX wire and GND\r\n");
    } else if (gps.sentences == 0u) {
        USB_Log("[BOOT] GPS: edges but no valid NMEA -> baud mismatch (expect 9600)?\r\n");
    }
}
#endif
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
    Sched_Init();          /* TIM5 1 MHz free-running timebase              */
    PWM_Init();            /* servos configured, all fins centred           */
    leds_update(VS_BOOT, HAL_GetTick());
#if CFG_ENABLE_GPS
    GPS_Init(&huart1);     /* NMEA RX: USART1 IRQ -> ring buffer            */
#endif

    HAL_Delay(1500);       /* USB CDC enumeration before logging            */
    USB_Log("\r\n=== Hyades FC-V4 (F411) boot === SYSCLK %lu Hz\r\n",
            (unsigned long)HAL_RCC_GetSysClockFreq());

#if CFG_ENABLE_GPS
    gps_boot_check();
#endif

    /* ---- IMU self-test ---- */
    if (!IMU_Init(&hi2c1)) {
        fault_halt("IMU init/self-test failed");
    }
    USB_Log("[BOOT] IMU OK (QMI8658A, no magnetometer)\r\n");

    /* ---- gyro calibration (airframe STILL; attitude is seeded from accel) ---- */
    USB_Log("[BOOT] Calibrating gyro - keep still\r\n");
    IMU_Calibrate(&hi2c1, &gcal, &acal);
    {
        char b[6][LOG_FX_LEN];
        USB_Log("[BOOT] gyro bias dps %s %s %s | accel bias m/s2 %s %s %s\r\n",
                Log_Fx(b[0], gcal.bias[0], 3u), Log_Fx(b[1], gcal.bias[1], 3u),
                Log_Fx(b[2], gcal.bias[2], 3u), Log_Fx(b[3], acal.bias[0], 3u),
                Log_Fx(b[4], acal.bias[1], 3u), Log_Fx(b[5], acal.bias[2], 3u));
    }
#if CFG_ENABLE_MAG_CAL
    USB_Log("[BOOT] Mag cal - rotate airframe through all orientations\r\n");
    IMU_MagCalibrate(&hi2c1, &mcal);
#else
    MagCal_Identity(&mcal);
#endif

    /* ---- estimator seed ---- */
    IEKF_Init(&iekf);
    {
        float a0[3], m0[3], r, p, y;
        seed_sample(a0, m0, 32u);
        IEKF_Seed(&iekf, a0, m0);
        IEKF_GetEuler(&iekf, &r, &p, &y);
        char b[3][LOG_FX_LEN];
        USB_Log("[BOOT] seed RPY deg %s %s %s\r\n", Log_Fx(b[0], r, 1u),
                Log_Fx(b[1], p, 1u), Log_Fx(b[2], y, 1u));
    }

    /* ---- controller ---- */
    Ctrl_Init(&ctrl);
#if CFG_ENABLE_BOOTVERIFY
    boot_verify();
#endif
#if CFG_ENABLE_SERVO_SWEEP
    servo_sweep();
#endif
#if CFG_ENABLE_IWDG
    MX_IWDG_Init();
#endif

    USB_Log("[BOOT] Settling %lu ms (control off, fins centred)\r\n",
            (unsigned long)RUN_SETTLE_MS);
#if CFG_TELEM_BINARY
    USB_Log("=== TELEMETRY BINARY START ===\r\n");   /* PC: switch to frame parse */
#else
    USB_Log("[BOOT] Status %lu Hz: t(s) state RPY(deg) w(dps) |a|(g) fins imu(fails) "
            "loop(us) ovr gps(fix sats alt rxB nmea bad) drop\r\n",
            (unsigned long)(FRAME_HZ / TEXT_STATUS_DECIMATE));
#endif

    /* ---- cyclic executive state ---- */
    const uint32_t settle_frames = (RUN_SETTLE_MS * FRAME_HZ) / 1000u;
    uint32_t frame    = 0u;
    uint32_t deadline = Sched_MicrosNow() + FRAME_PERIOD_US;
    uint32_t prev_t   = Sched_MicrosNow();
    uint32_t imu_fails = 0u, overruns = 0u, degraded_until_ms = 0u;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* Late at the top of the frame = previous frame overran (detect, never mask). */
        const bool overran = ((int32_t)(Sched_MicrosNow() - deadline) > 0);
        Sched_WaitUntil(deadline);
        deadline += FRAME_PERIOD_US;               /* fixed cadence, never re-sync */

        const uint32_t t0 = Sched_MicrosNow();
        float dt = (float)(t0 - prev_t) * 1e-6f;
        prev_t = t0;
        if (dt <= 0.0f || dt > 0.1f) { dt = FRAME_DT_S; }

        /* ---- sensing ---- */
        ImuSample s;
        IMU_ReadBody(&hi2c1, &gcal, &acal, &mcal, &s);
        if (!s.gyro_ok) { imu_fails++; }
        if (overran)    { overruns++; }
#if CFG_ENABLE_GPS
        GPS_Poll(256u);                            /* 9600 bps = ~10 B per frame */
#endif

        /* ---- estimation ---- */
        if (s.gyro_ok) { IEKF_Predict(&iekf, s.gyro, dt); }
        const bool acc_applied = s.accel_ok && IEKF_UpdateAccel(&iekf, s.accel);
        const bool mag_applied = s.mag_ok   && IEKF_UpdateMag(&iekf, s.mag);

        /* ---- control: active after the settle period ---- */
        if (frame == settle_frames) { ctrl.active = true; }
        float fin[4];
        Ctrl_Update(&ctrl, iekf.q, iekf.omega, dt, fin);
        for (uint8_t i = 0u; i < 4u; i++) { PWM_SetFin(i, fin[i]); }

        /* ---- telemetry (non-blocking; dropped and counted if USB busy) ---- */
#if CFG_TELEM_BINARY
        {
            TelemState st;
            st.t_us  = t0;
            st.frame = frame;
            IEKF_GetEuler(&iekf, &st.euler[0], &st.euler[1], &st.euler[2]);
            for (uint32_t k = 0u; k < 3u; k++) {
                st.omega[k] = s.gyro[k];           /* dps, body */
                st.accel[k] = s.accel[k];
                st.mag[k]   = s.mag[k];
                st.bias[k]  = iekf.bias[k];
            }
            for (uint32_t k = 0u; k < 4u; k++) { st.q[k] = iekf.q[k]; st.fin[k] = fin[k]; }
            st.health = 0u;
            if (s.accel_ok)            { st.health |= TELEM_H_ACCEL_OK; }
            if (s.gyro_ok)             { st.health |= TELEM_H_GYRO_OK; }
            if (s.mag_ok)              { st.health |= TELEM_H_MAG_OK; }
            if (acc_applied)           { st.health |= TELEM_H_ACCEL_APPLIED; }
            if (mag_applied)           { st.health |= TELEM_H_MAG_APPLIED; }
            if (ctrl.active)           { st.health |= TELEM_H_CTRL_ACTIVE; }
            if (overran)               { st.health |= TELEM_H_OVERRUN; }
            if (frame < settle_frames) { st.health |= TELEM_H_SETTLING; }
            st.loop_us = (uint16_t)(Sched_MicrosNow() - t0);
            (void)Telem_SendState(&st);
        }
#else
        (void)acc_applied; (void)mag_applied;
        if ((frame % TEXT_STATUS_DECIMATE) == 0u) {
            const float r2d = 57.29578f;
            float r, p, y;
            IEKF_GetEuler(&iekf, &r, &p, &y);
            const float an = sqrtf(s.accel[0] * s.accel[0] + s.accel[1] * s.accel[1]
                                   + s.accel[2] * s.accel[2]) / BOARD_G_MPS2;
            const char *state = (frame < settle_frames) ? "SETTLE" : "RUN";
#if CFG_ENABLE_GPS
            const char *gst = GPS_HasFix(GPS_TIMEOUT_MS) ? "FIX"
                            : (GPS_Alive(GPS_TIMEOUT_MS) ? "nofix" : "none");
            const float alt = gps.alt_msl_m;
#else
            const char *gst = "off";
            const float alt = 0.0f;
#endif
            char b[12][LOG_FX_LEN];
            char line[256];
            const int n = snprintf(line, sizeof line,
                "t=%lu.%02lu %s RPY %s %s %s w %s %s %s |a|=%s fin %s %s %s %s "
                "imu=%s(%lu) loop=%lu ovr=%lu gps=%s s%u alt=%s rx=%lu nmea=%lu bad=%lu drop=%lu\r\n",
                (unsigned long)(t0 / 1000000u), (unsigned long)((t0 / 10000u) % 100u), state,
                Log_Fx(b[0], r, 1u), Log_Fx(b[1], p, 1u), Log_Fx(b[2], y, 1u),
                Log_Fx(b[3], iekf.omega[0] * r2d, 1u), Log_Fx(b[4], iekf.omega[1] * r2d, 1u),
                Log_Fx(b[5], iekf.omega[2] * r2d, 1u), Log_Fx(b[6], an, 2u),
                Log_Fx(b[7], fin[0], 2u), Log_Fx(b[8], fin[1], 2u),
                Log_Fx(b[9], fin[2], 2u), Log_Fx(b[10], fin[3], 2u),
                s.gyro_ok ? "OK" : "FAIL", (unsigned long)imu_fails,
                (unsigned long)(Sched_MicrosNow() - t0), (unsigned long)overruns,
                gst, (unsigned)gps.sats, Log_Fx(b[11], alt, 1u),
                (unsigned long)gps.bytes, (unsigned long)gps.sentences,
                (unsigned long)gps.cksum_err, (unsigned long)Telem_Dropped());
            if (n > 0) {
                (void)Telem_SendText(line, (n < (int)sizeof line) ? (uint32_t)n : (uint32_t)sizeof line - 1u);
            }
        }
#endif

        /* ---- LEDs: degraded indication held 1 s after any IMU fail/overrun ---- */
        const uint32_t now_ms = HAL_GetTick();
        if (!s.gyro_ok || overran) { degraded_until_ms = now_ms + 1000u; }
        const VehState vs = (frame < settle_frames) ? VS_SETTLE
                          : (((int32_t)(degraded_until_ms - now_ms) > 0) ? VS_DEGRADED : VS_RUN);
        leds_update(vs, now_ms);

#if CFG_ENABLE_IWDG
        HAL_IWDG_Refresh(&hiwdg);                  /* only after a completed frame */
#endif
        frame++;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
  __HAL_DBGMCU_FREEZE_IWDG();   /* watchdog paused while the core is halted by the debugger */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 800;      /* LSI 32 kHz /4 -> ~100 ms (68-188 ms over LSI spread) = 10 frames */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 LED_2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
