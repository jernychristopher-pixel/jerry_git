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
#include "pid.h"
#include "motor.h"
#include "vofa.h"
#include "kinematics.h"
#include "protocol.h"
#include "imu.h"
#include "yaw_hold.h"
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
  SpeedPID_L.OutMax = 100.0f;  SpeedPID_L.OutMin = -100.0f;
  SpeedPID_R.OutMax = 100.0f;  SpeedPID_R.OutMin = -100.0f;
    /* M4-3: 默认 PID 组, 现场用 VOFA #S 0/1/2 切换 P/PI/PID */
    PID_Apply_Preset(&SpeedPID_L, 2U);
    PID_Apply_Preset(&SpeedPID_R, 2U);
  Vofa_Start_Receive_IT();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim8, 0);
  HAL_TIM_Base_Start_IT(&htim1);

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
    /* 固定 20ms 节拍: 先等到本周期起始时刻再干活, 避免"干活+延时"把采样率拖到 40Hz 左右 */
    {
      static uint32_t next_tick = 0;
      uint32_t now_tick = HAL_GetTick();
      if ((int32_t)(now_tick - next_tick) < 0)
      {
        HAL_Delay(1);
        continue;
      }
      next_tick = now_tick + 20;
    }

    Vofa_Rx_Reattach();   /* 串口出错后兜底重挂接收, 防止 RX 永久静默 */
    Vofa_Service();       /* #Q 参数回读: 主循环发出, 不挤占串口中断 */
    IMU_Read();   /* 轮询DMP FIFO, 更新偏航角 */
    /* 车轮静止时慢速跟踪陀螺零偏, 抑制温漂引起的残余漂移 */
    if (robot_state.actual_rpm_L < 4.0f && robot_state.actual_rpm_L > -4.0f &&
        robot_state.actual_rpm_R < 4.0f && robot_state.actual_rpm_R > -4.0f) {
        IMU_NotifyStill();
    }

#if APP_USE_OLED
    static uint8_t tick = 0;
    if ((tick++ % 3) == 0) {
        char buf[16], buf2[32];
        float vl_mm_s, vr_mm_s;
        Chassis_GetWheels_MmS(&vl_mm_s, &vr_mm_s);
        Ftoa(buf, vl_mm_s / 1000.0f, 2);
        snprintf(buf2, sizeof(buf2), "L:%s m/s", buf);
        OLED_PutString(0, 0, buf2, OLED_COLOR_WHITE);
        Ftoa(buf, vr_mm_s / 1000.0f, 2);
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

        snprintf(buf, sizeof(buf), "HOST");
        OLED_PutString(0, 24, buf, OLED_COLOR_WHITE);

        snprintf(buf2, sizeof(buf2), "HOST E:%u", (unsigned)Vofa_GetRxErr());
        OLED_PutString(0, 32, buf2, OLED_COLOR_WHITE);

        Ftoa(buf, IMU_GetYawRate() * 57.29578f, 1);
        char buf4[16];
        Ftoa(buf4, YawHold_GetLastOut(), 2);
        snprintf(buf2, sizeof(buf2), "Ry:%s Wc:%s", buf, buf4);
        OLED_PutString(0, 48, buf2, OLED_COLOR_WHITE);

        OLED_Refresh();
    }
#endif /* APP_USE_OLED */

    /* USART1: 树莓派 37字节反馈帧 (布局见 Lib/comm/protocol.h) */
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
    /* bit0=IMU可用, bit1=上位机控制中(纯上位机版本恒为1); bit2-3 固定填 0 */
    fb.status              = (uint8_t)((IMU_IsOk() ? 0x01u : 0x00u) | 0x02u);
    {
        float wl_mm_s, wr_mm_s;
        Chassis_GetWheels_MmS(&wl_mm_s, &wr_mm_s);
        fb.vl_mm_s = Protocol_ClampI16(wl_mm_s);
        fb.vr_mm_s = Protocol_ClampI16(wr_mm_s);
    }
    if (Vofa_Csv_Enabled()) {
        /* M4-3 采集模式: USART1 改发 CSV, 一行 50Hz(主循环 20ms) */
        Vofa_Send_Csv(HAL_GetTick() / 1000.0f,
                      robot_state.target_rpm_L, robot_state.actual_rpm_L, SpeedPID_L.Out,
                      robot_state.target_rpm_R, robot_state.actual_rpm_R, SpeedPID_R.Out);
    } else {
        Protocol_Send_Feedback(&fb);

        /* USART3: VOFA 文本波形(仅非 CSV 模式发送, 采集时让出带宽和 CPU 时间) */
        Vofa_Send(robot_state.target_rpm_L, robot_state.actual_rpm_L,
                  robot_state.target_rpm_R, robot_state.actual_rpm_R,
                  YawHold_GetYaw(), YawHold_GetError(),
                  IMU_GetYawRate() * 57.29578f, YawHold_GetLastOut());
    }
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


