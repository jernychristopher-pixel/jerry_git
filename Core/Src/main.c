/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "app_config.h"
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "motor.h"
#include "vofa.h"
#include "kinematics.h"
#include "protocol.h"
#include "imu.h"
#include "yaw_hold.h"
#include "ps2.h"
#include "adc.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_PutString(0, 0, "L:", OLED_COLOR_WHITE);
  OLED_PutString(0, 8, "R:", OLED_COLOR_WHITE);
  OLED_Refresh();

  /* 上电静止1秒校准陀螺零偏, 期间保持车不动 */
  OLED_PutString(0, 16, "IMU CAL...", OLED_COLOR_WHITE);
  OLED_Refresh();
  /* MPU6050 DMP: 初始化失败不影响行车, OLED会显示YAW:-- */
  IMU_Init();

  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);

  PID_Init(&SpeedPID_L);
  PID_Init(&SpeedPID_R);
  SpeedPID_L.Kp = 1.5f;  SpeedPID_L.Ki = 0.4f;  SpeedPID_L.Kd = 0.1f;
  SpeedPID_L.OutMax = 100.0f;  SpeedPID_L.OutMin = -100.0f;
  SpeedPID_R.Kp = 1.5f;  SpeedPID_R.Ki = 0.3f;  SpeedPID_R.Kd = 0.1f;
  SpeedPID_R.OutMax = 100.0f;  SpeedPID_R.OutMin = -100.0f;
  Vofa_Start_Receive_IT();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim8, 0);
  HAL_TIM_Base_Start_IT(&htim1);

  PS2_Init();
  PS2_EnableAnalog();
  Battery_ADC_Init();
  // TEST: 注释掉下面这行看电机转不转
  // Inverse_Kinematics(0.2f, 0.0f);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    PS2_ReadData();
    Vofa_Rx_Reattach();   /* 串口出错后兜底重挂接收, 防止 RX 永久静默 */
    Vofa_Service();       /* #Q 参数回读: 主循环发出, 不挤占串口中断 */
    IMU_Read();   /* 轮询DMP FIFO, 更新偏航角 */
    /* 车轮静止时慢速跟踪陀螺零偏, 抑制温漂引起的残余漂移 */
    if (robot_state.raw_pulse_L < 2.0f && robot_state.raw_pulse_L > -2.0f &&
        robot_state.raw_pulse_R < 2.0f && robot_state.raw_pulse_R > -2.0f) {
        IMU_NotifyStill();
    }

    static uint32_t last_gear_ms = 0;
    static uint32_t last_reinit_ms = 0;
    static uint8_t  bad_frames = 0;
    static int16_t  lx0 = 128, ly0 = 128;   /* 上电标定的摇杆中点 */
    static uint8_t  calib_done = 0;
    static uint8_t  ctrl_mode = 0;      /* 0=按键模式(默认) 1=摇杆模式 */
    static const float gear_gain[3] = {0.5f, 1.0f, 1.5f};

    /* 帧判定与旧版一致(0x5A),另加保护: 数字模式/掉线时四个摇杆字节全0视为无效,
       连续无效约1秒才重新配置手柄,正常行驶中不干扰手柄 */
    uint8_t frame_ok = (PS2_Data[2] == 0x5A) &&
                       !(PS2_Data[5] == 0 && PS2_Data[6] == 0 &&
                         PS2_Data[7] == 0 && PS2_Data[8] == 0);
    if (frame_ok) {
        bad_frames = 0;
    } else if (++bad_frames > 40 && (HAL_GetTick() - last_reinit_ms >= 1000)) {
        PS2_SetInit();
        last_reinit_ms = HAL_GetTick();
        bad_frames = 0;
    }

    int lx = 128, ly = 128;
    uint16_t key = 0;

    if (frame_ok) {
        lx = PS2_Data[PSS_LX];
        ly = PS2_Data[PSS_LY];

        /* 上电后采样20帧"稳定且接近中间"的数据标定中点(上电时摇杆必须松开,
           有人拨杆导致抖动则放弃这批数据重新采样) */
        if (!calib_done) {
            static int32_t acc_x = 0, acc_y = 0;
            static int16_t min_x = 255, max_x = 0, min_y = 255, max_y = 0;
            static uint8_t  calib_cnt = 0;
            if (lx > 108 && lx < 148 && ly > 108 && ly < 148) {
                acc_x += lx;
                acc_y += ly;
                if (lx < min_x) min_x = (int16_t)lx;
                if (lx > max_x) max_x = (int16_t)lx;
                if (ly < min_y) min_y = (int16_t)ly;
                if (ly > max_y) max_y = (int16_t)ly;
                if (++calib_cnt >= 20) {
                    if ((max_x - min_x) <= 4 && (max_y - min_y) <= 4) {
                        lx0 = (int16_t)(acc_x / 20);
                        ly0 = (int16_t)(acc_y / 20);
                        calib_done = 1;
                    } else {
                        acc_x = 0; acc_y = 0;
                        min_x = 255; max_x = 0; min_y = 255; max_y = 0;
                        calib_cnt = 0;
                    }
                }
            }
        }

        /* 中点慢速跟踪: 摇杆长时间稳定停在中间附近时, 每次只修正1格,
           补偿兼容手柄随温度/使用时间的中点漂移; 大角度转向不受影响 */
        if (calib_done) {
            static int16_t  last_lx = 0, last_ly = 0;
            static uint16_t steady_cnt = 0;
            int16_t ax = (int16_t)lx - lx0;
            int16_t ay = (int16_t)ly - ly0;
            if (ax >= -10 && ax <= 10 && ay >= -12 && ay <= 12) {
                int16_t dx = (int16_t)lx - last_lx;
                int16_t dy = (int16_t)ly - last_ly;
                if (dx >= -2 && dx <= 2 && dy >= -2 && dy <= 2) {
                    if (++steady_cnt >= 50) {
                        if (ax > 0) lx0++;
                        else if (ax < 0) lx0--;
                        if (ay > 0) ly0++;
                        else if (ay < 0) ly0--;
                        steady_cnt = 0;
                    }
                } else {
                    steady_cnt = 0;
                }
            } else {
                steady_cnt = 0;
            }
            last_lx = (int16_t)lx;
            last_ly = (int16_t)ly;
        }

        for (uint8_t i = 0; i < 8; i++) {
            if (PS2_Data[3] & (1 << i)) key |= (uint16_t)(1 << i);
            if (PS2_Data[4] & (1 << i)) key |= (uint16_t)(1 << (i + 8));
        }

        static uint16_t last_key = 0;
        uint16_t pressed = key & ~last_key;
        last_key = key;

        if (pressed & 0x1000) {          /* TRIANGLE: 切换 按键/摇杆 模式 */
            ctrl_mode = !ctrl_mode;
        }

        if (pressed & 0x0008) {          /* START: 自动模式交还手动控制 */
            Chassis_Set_Mode(CHASSIS_MODE_MANUAL);
        }

        if ((key & 0x0001) && (key & 0x0400) &&
            (pressed & 0x0401) && HAL_GetTick() - last_gear_ms >= 500) {
            gear = (gear % 3) + 1;
            last_gear_ms = HAL_GetTick();
        }
    }

    int dlx = lx - lx0;
    int dly = ly - ly0;
    if (dlx > -2 && dlx < 2) dlx = 0;   /* 转向死区,抑制中点抖动 */
    if (dly > -8 && dly < 8) dly = 0;

    float gain = gear_gain[gear - 1];
    float V, W;

    if (ctrl_mode == 0) {
        /* 按键模式(默认): 方向键四键, 排除摇杆漂移的人为因素 */
        float bv = 0.0f, bw = 0.0f;
        if (key & 0x0040) bv += 1.0f;   /* 前进 */
        if (key & 0x0010) bv -= 1.0f;   /* 后退 */
        if (key & 0x0020) bw += 1.0f;   /* 左转 */
        if (key & 0x0080) bw -= 1.0f;   /* 右转 */
        V = bv * 0.512f * gain;
        /* 原地拐弯: 左右轮等速反向, 每轮固定0.10m/s (W=2*0.10/0.115=1.739rad/s) */
        W = bw * 1.739f;
    } else {
        /* 摇杆模式 */
        V = -dly * 0.004f * gain;
        W = -dlx * 0.050f * gain;
    }

    if (V < 0.0f) V *= back_scale;   /* 后退目标按系数缩放, 与前进速度对称 */

    /* 手动命令统一经 Chassis_Set_Cmd 下发; 斜坡/死区/逆解算已移到 TIM1 控制环,
       自动模式(SLAM)下手动命令会被拒绝, 按手柄START键可交还手动控制 */
    Chassis_Set_Cmd(V, W, CHASSIS_SRC_MANUAL);

#if APP_USE_OLED
    static uint8_t tick = 0;
    if ((tick++ % 3) == 0) {
        char buf[16], buf2[32];
        Ftoa(buf, robot_state.actual_pulse_L * 0.00561f * CHASSIS_V_SIGN, 2);
        snprintf(buf2, sizeof(buf2), "L:%s m/s", buf);
        OLED_PutString(0, 0, buf2, OLED_COLOR_WHITE);
        Ftoa(buf, robot_state.actual_pulse_R * 0.00561f * CHASSIS_V_SIGN, 2);
        snprintf(buf2, sizeof(buf2), "R:%s m/s", buf);
        OLED_PutString(0, 8, buf2, OLED_COLOR_WHITE);

        if (IMU_IsOk()) {
            Ftoa(buf, IMU_GetYaw(), 1);
            char buf3[16];
            Ftoa(buf3, YawHold_GetError(), 1);
            snprintf(buf2, sizeof(buf2), "YAW:%s E:%s", buf, buf3);
        } else {
            snprintf(buf2, sizeof(buf2), "YAW:-- E:%d", IMU_GetInitErr());
        }
        OLED_PutString(0, 16, buf2, OLED_COLOR_WHITE);

        if (Chassis_Get_Mode() == CHASSIS_MODE_AUTO) {
            snprintf(buf, sizeof(buf), "AUTO");
        } else {
            const char *vdir = (V > 0.02f) ? "FWD" : ((V < -0.02f) ? "BWD" : "");
            const char *wdir = (W > 0.04f) ? "LEFT" : ((W < -0.04f) ? "RIGHT" : "");
            if (vdir[0] != 0 && wdir[0] != 0) {
                snprintf(buf, sizeof(buf), "%s %s", vdir, wdir);
            } else if (vdir[0] != 0) {
                snprintf(buf, sizeof(buf), "%s", vdir);
            } else if (wdir[0] != 0) {
                snprintf(buf, sizeof(buf), "%s", wdir);
            } else {
                snprintf(buf, sizeof(buf), "STOP");
            }
            size_t used = strlen(buf);
            snprintf(buf + used, sizeof(buf) - used, " MAN");
        }
        OLED_PutString(0, 24, buf, OLED_COLOR_WHITE);

        snprintf(buf2, sizeof(buf2), "%s ID:%02X E:%u", (ctrl_mode ? "JOY" : "BTN"),
                 PS2_Data[1], (unsigned)Vofa_GetRxErr());
        OLED_PutString(0, 32, buf2, OLED_COLOR_WHITE);

        snprintf(buf2, sizeof(buf2), "GEAR %d", gear);
        OLED_PutString(0, 40, buf2, OLED_COLOR_WHITE);

        Ftoa(buf, IMU_GetYawRate() * 57.29578f, 1);
        char buf4[16];
        Ftoa(buf4, YawHold_GetLastOut(), 2);
        snprintf(buf2, sizeof(buf2), "Ry:%s Wc:%s", buf, buf4);
        OLED_PutString(0, 48, buf2, OLED_COLOR_WHITE);

        OLED_Refresh();
    }
#endif /* APP_USE_OLED */

    /* USART1: 树莓派 37字节反馈帧 (布局见 Lib/Inc/protocol.h) */
    Protocol_Feedback_t fb;
    fb.flag_stop  = 0x00;                        /* 0=电机使能 */
    fb.vx_mm_s    = (int16_t)Chassis_GetVx_MmS();
    fb.vy_mm_s    = 0;
    fb.wz_mrad_s  = (int16_t)Chassis_GetWz_MradS();
    fb.acc_x_raw  = IMU_GetAccelXRaw();
    fb.acc_y_raw  = IMU_GetAccelYRaw();
    fb.acc_z_raw  = IMU_GetAccelZRaw();
    fb.gyro_x_raw = IMU_GetGyroXRaw();
    fb.gyro_y_raw = IMU_GetGyroYRaw();
    fb.gyro_z_raw = IMU_GetGyroZRaw();
    fb.battery_mv = Battery_Get_Mv();
    /* 原来显示在 OLED 上的 IMU / 航向环信息, OLED 拆掉后改由串口上行 */
    fb.yaw_001deg          = Protocol_ClampI16(IMU_GetYaw() * 100.0f);
    fb.yaw_rate_mrad_s     = Protocol_ClampI16(IMU_GetYawRate() * 1000.0f);
    fb.yaw_hold_err_001deg = Protocol_ClampI16(YawHold_GetError() * 100.0f);
    fb.yaw_hold_out_mrad_s = Protocol_ClampI16(YawHold_GetLastOut() * 1000.0f);
    fb.status              = (uint8_t)((IMU_IsOk() ? 0x01u : 0x00u) |
                                       (Chassis_Get_Mode() == CHASSIS_MODE_AUTO ? 0x02u : 0x00u) |
                                       (uint8_t)(((gear >= 1 && gear <= 3 ? gear : 1) - 1) << 2));
    {
        float wl_mm_s, wr_mm_s;
        Chassis_GetWheels_MmS(&wl_mm_s, &wr_mm_s);
        fb.vl_mm_s = Protocol_ClampI16(wl_mm_s);
        fb.vr_mm_s = Protocol_ClampI16(wr_mm_s);
    }
    Protocol_Send_Feedback(&fb);

    /* USART3: VOFA 文本波形 */
    Vofa_Send(robot_state.target_pulse_L, robot_state.raw_pulse_L,
              robot_state.target_pulse_R, robot_state.raw_pulse_R,
              YawHold_GetYaw(), YawHold_GetError(),
              IMU_GetYawRate() * 57.29578f, YawHold_GetLastOut());

    HAL_Delay(20);
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


