/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "bdma.h"
#include "dcmi.h"
#include "dma.h"
#include "fatfs.h"
#include "openamp.h"
#include "stm32h7xx_hal_i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "defines.h"
#include "ipc_messages.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "i2c.h"
#include "ov2640.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
message_notif_t message_notif;

static uint32_t message;
static volatile uint8_t message_received;
static volatile message_notif_t received_data;

static volatile uint8_t text_received;
static volatile uint8_t acoustic_received;

char recv_text_buf[TEXT_MSG_SIZE];
strength_t recv_acoustic_buf[ACOUSTIC_SIZE];

strength_t power_buf[ACOUSTIC_HORIZ][ACOUSTIC_VERT];
strength_t freq_buf[ACOUSTIC_HORIZ][ACOUSTIC_VERT];

static struct rpmsg_endpoint rp_endpoint;

uint8_t TxBuffer[128];
uint8_t TxBufferLen = sizeof(TxBuffer);
uint8_t send_update = 0;
uint8_t power_header[] = {0xFD, 0xFD,0xFD,0xFD};
uint8_t freq_header[] = {0xFE,0xFE,0xFE,0xFE};

own_list_t own_list = {ANY, ANY, ANY, ANY, ANY, ANY};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
void write_log(char *str, uint16_t size);

strength_t *get_open_acoustic(void);
char *get_open_text(void);
uint8_t *get_open_ctrl(void);

uint8_t buf_take(void *buffer);
uint8_t buf_release(void *buffer);

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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_DCMI_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_FMC_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* Inilitize the mailbox use notify the other core on new message */
  MAILBOX_Init();

  /* Inilitize OpenAmp and libmetal libraries */
  if (MX_OPENAMP_Init(RPMSG_REMOTE, NULL)!= HAL_OK)
    Error_Handler();

  uint8_t status = OPENAMP_create_endpoint(&rp_endpoint, RPMSG_CHAN_NAME, RPMSG_ADDR_ANY,
                                    rpmsg_recv_callback, NULL);
  if (status < 0)
  {
    Error_Handler();
  }

  while (HAL_HSEM_FastTake(HSEM_I2C4) != HAL_OK) {}
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
  MX_I2C4_Init();
  // OV2640_Init(&hi2c4, &hdcmi);
  HAL_HSEM_Release(HSEM_I2C4, 0);
  /* create a endpoint for rmpsg communication */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OPENAMP_check_for_message();
    if (send_update) {
      uint8_t max_x = 0;
      uint8_t max_y = 0;

      for (int x = 0; x < ACOUSTIC_HORIZ; x++) {
        for (int y = 0; y < ACOUSTIC_VERT; y++) {
          if (power_buf[x][y] > power_buf[max_x][max_y]) {
            max_x = x;
            max_y = y;
          }
        }
      }

      // float horiz_step = HORIZ_FOV / ACOUSTIC_HORIZ;
      // float vert_step = VERTICAL_FOV / ACOUSTIC_VERT;

      // float theta_x = (-HORIZ_FOV / 2 + horiz_step * (max_x + 0.5)) * 180 / 3.14159265;
      // float theta_y = (-VERTICAL_FOV / 2 + vert_step * (max_y + 0.5)) * 180 / 3.14159265;

      // printf("Freq: %f\tPower: %f\ttheta_x: %f\ttheta_y: %f\n", freq_buf[max_x][max_y], power_buf[max_x][max_y], theta_x, theta_y);
      while (CDC_Transmit_FS(power_header, 4) == USBD_BUSY) {}
      while (CDC_Transmit_FS((uint8_t *) power_buf, sizeof(power_buf)) == USBD_BUSY) {}
      while (CDC_Transmit_FS(freq_header, 4) == USBD_BUSY) {}
      while (CDC_Transmit_FS((uint8_t *) freq_buf, sizeof(freq_buf)) == USBD_BUSY) {}
      send_update = 0;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 98;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 2458;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
               size_t len, uint32_t src, void *priv)
{
  received_data = *((message_notif_t *) data);
  own_t* ptr = get_own_flag(received_data.address, &own_list);
  if (received_data.type == MSG_TAKE) {
    *ptr = M7;
  } else if (received_data.type == MSG_RELEASE) {
    *ptr = ANY;
  } else {
    *ptr = M4;
  }
  if (received_data.type == MSG_TEXT) {
    memcpy(recv_text_buf, (const char *) received_data.address, sizeof(recv_text_buf));
    buf_release(received_data.address);
    
    #ifdef DEBUG
    printf("CM7: %s\n", recv_text_buf);
    #endif
  }
  else if (received_data.type == MSG_ACOUSTIC) {
    memcpy(power_buf, acoustic_power, sizeof(power_buf));
    memcpy(freq_buf, acoustic_freq, sizeof(freq_buf));
    buf_release(acoustic_power);
    buf_release(acoustic_freq);
    send_update = 1;
  }

  message_received=1;

  return 0;
}

void write_log(char *str, uint16_t size) {
  //TODO: add formatting and whatnot

}

/**
 * @brief Retargets the C library printf function to the USART.
 * None
 * @retval None
 */
int _write(int file, char *ptr, int len)
{
  /* Send to ITM for debugging */
  for (int i = 0; i < len; i++) {
    ITM_SendChar(ptr[i]);
  }
  
  /* Send the entire buffer at once over USB CDC */
  /* Note: If you call printf in rapid succession, you may still need 
     a while(CDC_Transmit_FS(...) == USBD_BUSY) loop, but use caution 
     as it can hang the MCU if the USB cord is unplugged. */
  CDC_Transmit_FS((uint8_t*)ptr, len);
  
  //TODO: also log to SD card here
  
  return len;
}

char *get_open_text(void) {
  if (own_list.text_ping != M7)
    return text_ping;
  else if (own_list.text_pong != M7)
    return text_pong;
  else
    return NULL;
}

uint8_t *get_open_ctrl(void){
  if (own_list.control_ping != M7)
    return ctrl_ping;
  else if (own_list.control_pong != M7)
    return ctrl_pong;
  else
    return NULL;
}

uint8_t buf_take(void *buffer) {
  *get_own_flag(buffer, &own_list) = M4;
  message_notif.address = buffer;
  message_notif.length = 0;
  message_notif.type = MSG_TAKE;
  OPENAMP_send(&rp_endpoint, &message_notif, sizeof(message_notif));
  return 1;
}

uint8_t buf_release(void *buffer) {
  *get_own_flag(buffer, &own_list) = ANY;
  message_notif.address = buffer;
  message_notif.length = 0;
  message_notif.type = MSG_RELEASE;
  OPENAMP_send(&rp_endpoint, &message_notif, sizeof(message_notif));
  return 1;
}

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
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
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
