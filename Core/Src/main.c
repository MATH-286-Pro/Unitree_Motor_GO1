/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention      : 小米电机测试
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MI_motor_drive.h"
#include "can_test.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "bsp_rc.h"
#include "OLED.h"
#include "motor_A1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 宏定义 不要加";" 要不然会报错
#define PI 3.1415926535f
#define DGR2RAD PI/180
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const RC_ctrl_t* DT7_pram; //遥控器控制结构体

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // MX_CAN1_Init() 功能：配置CAN1参数 + 打开NVIC + 打开GPIO

  //自定义 初始化 开始 ----------------------------------------------------------------
  HAL_Delay(1000);       // 延时1s 防止电机没上电先初始化现象
  int i = 0;
  OLED_init();           // OLED初始化
  OLED_clear();          OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // OLED清屏
  remote_control_init(); OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // 遥控器初始化
  CAN_Init(&hcan1);      OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // 初始化CAN1

  // TIM4 已经设置 50% 占空比
  // PSC=0 Reload=21000-1 => f=4KHz
  HAL_TIM_Base_Start(&htim4);               // 开启TIM4
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  // 开启TIM4 PWM 蜂鸣器
  __HAL_TIM_PRESCALER(&htim4, 8);           // 设置TIM4 预分频 调整音色
  HAL_Delay(100);                           // 延时0.1s
  HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);   // 关闭TIM4 PWM

  MI_Motor_s MI_Motor_ID1;                  // 定义小米电机结构体1
  MI_Motor_s MI_Motor_ID2;                  // 定义小米电机结构体2
  MI_motor_Init(&MI_Motor_ID1,&MI_CAN_1,1); // 将MI_CAN_1，1传入小米结构体 
  MI_motor_Init(&MI_Motor_ID2,&MI_CAN_1,2); // 将MI_CAN_2，2传入小米结构体 
  MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
  MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能
  OLED_clear();
  //自定义 初始化 结束 ----------------------------------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      OLED_show_string(0,0,"S1 = ");   OLED_show_string(0,10,"S0 = "); 
      OLED_show_string(2,0,"CH2= ");   OLED_show_string(2,10,"CH1= ");
      OLED_show_string(3,0,"CH3= ");   OLED_show_string(3,10,"CH0= ");

  while (1)
  {
    DT7_pram = get_remote_control_point(); // 获取遥控器控制结构体
    uint8_t STOP = DT7_pram->rc.s[1]/2;    // 跟踪遥控器开关 S[1]左 S[0]右 状态  // 上1 中3 下2
                                           

    // 跟踪遥控器4个通道参数
    OLED_show_num(0,5,(uint8_t) DT7_pram->rc.s[1]/2,1);  OLED_show_num(0,15,(uint8_t) DT7_pram->rc.s[0]/2,1);
    OLED_show_signednum(2,5,DT7_pram->rc.ch[2],3);       OLED_show_signednum(2,15,DT7_pram->rc.ch[0],3);
    OLED_show_signednum(3,5,DT7_pram->rc.ch[3],3);       OLED_show_signednum(3,15,DT7_pram->rc.ch[1],3);
    OLED_refresh_gram();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 宇树A1电机控制
    // A1_Motor_Speed_Control(0,(float) DT7_pram->rc.ch[0]/660*10); // 该函数使用 UART1 发送
    // HAL_Delay(2); 
    // A1_Motor_Speed_Control(1,(float) DT7_pram->rc.ch[2]/660*10); // 该函数使用 UART1 发送
    // A1_Motor_Speed_Control(1,(float) 5.0f); // 该函数使用 UART1 发送
    // HAL_Delay(2); 

    // 宇树A1电机 位置模式
    // 输入(1)*360*DGR2RAD*9.1f = 减速后 360°
    A1_Motor_Position_Control(0,(float) STOP*DT7_pram->rc.ch[2]/660*360*DGR2RAD*9.1f); // 该函数使用 UART1 发送
    HAL_Delay(1);

    // 宇树A1电机 力矩模式
    // 官方建议开始值 0.05f
    // 发热非常明显
    // A1_Motor_Torque_Control(1,(float) STOP*DT7_pram->rc.ch[2]/660*10); // 该函数使用 UART1 发送
    // A1_Motor_Torque_Control(1,(float) 0.05f); // 该函数使用 UART1 发送
    // HAL_Delay(1);

    // 宇树A1电机 0力矩模式
    // A1_Motor_0Torque_Control(0xBB); 
    // HAL_Delay(1);

    // 小米电机控制
    MI_motor_SpeedControl(&MI_Motor_ID1,(float) STOP*DT7_pram->rc.ch[1]/33,1); // 使用 (float) 强制转换
    MI_motor_SpeedControl(&MI_Motor_ID2,(float) STOP*DT7_pram->rc.ch[3]/33,1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
