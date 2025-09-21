/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file : main.c
  * @brief : Main program body
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
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#ifdef MAVPACKED
#undef MAVPACKED
#endif
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  #define MAVPACKED(__Declaration__) __attribute__((__packed__)) __Declaration__
#else
  #define MAVPACKED(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#endif
#include "mavlink/common/mavlink.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PX4_MAIN_MODE_MANUAL 1
#define PX4_MAIN_MODE_ALTCTL 2
#define PX4_MAIN_MODE_POSCTL 3
#define PX4_MAIN_MODE_AUTO 4
#define PX4_MAIN_MODE_ACRO 5
#define PX4_MAIN_MODE_OFFBOARD 6
#define PX4_MAIN_MODE_STABILIZED 7
#define PX4_MAIN_MODE_RATTITUDE 8
#define PX4_MAIN_MODE_SIMPLE 9
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE BEGIN PV */
uint8_t ibus_buf[32] = {0};
uint8_t framebuf[32] = {0};
volatile bool ibus_frame_ready = false;
uint16_t roll, pitch, yaw, throttle, AUX1, AUX2;
uint16_t channels[14];
int16_t manual_x = 0; // Roll
int16_t manual_y = 0; // Pitch
int16_t manual_z = 500; // Throttle (0..1000)
int16_t manual_r = 0; // Yaw
uint8_t target_sysid = 1;   // Default PX4 system ID
uint8_t target_compid = 1;  // Default PX4 component ID
uint8_t my_sysid = 255;     // GCS-like system ID
uint8_t my_compid = 0;    // GCS-like component ID
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t scale_to_manual(float val, uint8_t is_throttle) {
    val = fmaxf(fminf(val, 1.0f), -1.0f);
    if (is_throttle) {
        return (int16_t)((val + 1.0f) * 500.0f);
    } else {
        return (int16_t)(val * 1000.0f);
    }
}

static inline uint32_t px4_custom_mode(uint8_t main_mode, uint8_t sub_mode)
{
    return ((uint32_t)main_mode << 16) | ((uint32_t)sub_mode << 24);
}

void send_heartbeat(void) {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;
    mavlink_msg_heartbeat_pack(
        my_sysid, my_compid, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0
    );
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    HAL_UART_Transmit(&huart2, buffer, len, HAL_MAX_DELAY);
}

void send_command_long(uint16_t command, float param1) {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;
    mavlink_msg_command_long_pack(
        my_sysid, my_compid, &msg,
        target_sysid, target_compid, command, 0,
        param1, 0, 0, 0, 0, 0, 0
    );
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    HAL_UART_Transmit(&huart2, buffer, len, HAL_MAX_DELAY);
}

void send_manual_control(void) {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;
    mavlink_msg_manual_control_pack(
        my_sysid, my_compid, &msg,
        target_sysid,
        manual_x, manual_y, manual_z, manual_r,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    );
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    HAL_UART_Transmit(&huart2, buffer, len, HAL_MAX_DELAY);
}

void arm(void) {
    send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void disarm(void) {
    send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void send_set_mode(uint8_t base_mode, uint8_t main_mode, uint8_t sub_mode)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;
    uint32_t custom_mode = px4_custom_mode(main_mode, sub_mode);
    mavlink_msg_set_mode_pack(
        my_sysid, my_compid, &msg,
        target_sysid, base_mode, custom_mode
    );
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    HAL_UART_Transmit(&huart2, buffer, len, HAL_MAX_DELAY);
}
uint8_t wait_heartbeat(void) {
    uint8_t rx_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_status_t status;

    uint32_t timeout = HAL_GetTick() + 3000; // 3 seconds timeout
    while (HAL_GetTick() < timeout) {
        uint8_t byte;
        if (HAL_UART_Receive(&huart2, &byte, 1, 10) == HAL_OK) {
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    target_sysid = msg.sysid;
                    target_compid = msg.compid;
                    return 1;
                }
            }
        }
    }
    return 0;
}

void receive_mavlink(void) {
    uint8_t rx_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_status_t status;
    if (HAL_UART_Receive(&huart2, rx_buf, sizeof(rx_buf), 10) == HAL_OK) {
        for (uint16_t i = 0; i < sizeof(rx_buf); i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    // Handle COMMAND_ACK silently
                }
            }
        }
    }
}

bool check_ibus(uint8_t* buf) {
    if (buf[0] != 0x20 || buf[1] != 0x40) {
        return false;
    }
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) {
        sum += buf[i];
    }
    uint16_t checksum = buf[30] | (buf[31] << 8);
    return (checksum == (uint16_t)(0xFFFF - sum));
}

void parse_ibus(uint8_t* buf) {
    for (int i = 0; i < 14; i++) {
        channels[i] = buf[i * 2 + 2] | (buf[i * 2 + 3] << 8);
    }
    roll = channels[0];
    pitch = channels[1];
    throttle = channels[2];
    yaw = channels[3];
    AUX1 = channels[4];
		AUX2 = channels[5];
    // Scale to [-1, 1]
		float roll_f = (float)(roll - 1500) / 500.0f;
    float pitch_f = (float)(pitch - 1500) / 500.0f;
    float throttle_f = (float)(throttle - 1500) / 500.0f;
    float yaw_f = (float)(yaw - 1500) / 500.0f;
    // Scale to manual control values
		roll_f = fmaxf(fminf(roll_f, 1.0f), -1.0f);
    pitch_f = fmaxf(fminf(pitch_f, 1.0f), -1.0f);
    throttle_f = fmaxf(fminf(throttle_f, 1.0f), -1.0f);
    yaw_f = fmaxf(fminf(yaw_f, 1.0f), -1.0f);
		
    manual_x = scale_to_manual(pitch_f, 0);
    manual_y = scale_to_manual(roll_f, 0);
    manual_z = scale_to_manual(throttle_f, 1);
    manual_r = scale_to_manual(yaw_f, 0);

    if (throttle_f <= -0.9f && roll_f < -0.5f && yaw_f > 0.5f && pitch_f <= 0.9f) {
        arm(); 
    }
    else if (throttle_f <= -0.9f && roll_f > 0.5f && yaw_f < -0.5f && pitch_f <= 0.9f) {
        disarm();
    }
		if (AUX1 > 1800) {
						send_command_long(MAV_CMD_NAV_LAND, 0.0f);
				} else {
						if (AUX2 < 1400) {
								send_set_mode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_MODE_POSCTL, 0);
						} else if (AUX2 >= 1400 && AUX2 < 1600) {
								send_set_mode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_MODE_OFFBOARD, 0);
						} else if (AUX2 >= 1600) {
								send_set_mode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_MODE_MANUAL, 0);
						}
				}
}

void USART1_IdleHandler(void) {
    HAL_UART_IRQHandler(&huart1);
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
        memcpy(framebuf, ibus_buf, 32);
        ibus_frame_ready = true;
        HAL_UART_Receive_DMA(&huart1, ibus_buf, 32);
    }
}
/* USER CODE END 0 */
/**
  * @brief The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */
    /* MCU Configuration--------------------------------------------------------*/
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
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_DMA(&huart1, ibus_buf, 32);
		
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t last_10hz = 0;
    uint32_t last_20hz = 0;

    while (1) {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        uint32_t current_time = HAL_GetTick();
				receive_mavlink();

        if (ibus_frame_ready) {
            ibus_frame_ready = false;
            if (check_ibus(framebuf)) {
                parse_ibus(framebuf);
							
            }
        }

        if (current_time - last_10hz >= 100) {
            send_heartbeat();
            last_10hz = current_time;
        }

        if (current_time - last_20hz >= 50) {
            send_manual_control();
            last_20hz = current_time;
        }
        /* USER CODE END 3 */
    }
    /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}
/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
  * @brief This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}
#ifdef USE_FULL_ASSERT
/**
  * @brief Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */