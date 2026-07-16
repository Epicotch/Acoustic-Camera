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
#include "arm_math_types.h"
#include "bdma.h"
#include "dma.h"
#include "dsp/fast_math_functions.h"
#include "dsp/svm_functions.h"
#include "i2c.h"
#include "openamp.h"
#include "sai.h"
#include "sdmmc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include "arm_math.h"
#include "ipc_messages.h"
#include "defines.h"
#include "stm32h7xx_hal_sai.h"
#include "tlv320adc5140.h"
#include "stm32h747xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_i2c.h"

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

#define ADC1_ADDR 0x4C << 1
#define ADC2_ADDR 0x4D << 1

#define HALF_SAMPLES  (NUM_CHANNELS * SAMPLES_PER_CH)
#define HALF_BYTES    (HALF_SAMPLES * sizeof(int16_t))

#define LUT_BITS      10
#define LUT_SIZE      (1u << LUT_BITS)
#define PHASE_SHIFT   (32 - LUT_BITS)
#define PHASE_ROUND   (1u << (PHASE_SHIFT - 1))
#define PHASE_SCALE   (4294967296.0f / (float32_t)SAMPLES_PER_CH)

#define MAX_INDEX     SAMPLES_PER_CH / 2 // up to change
#define MIN_INDEX     40
#define NBINS         (MAX_INDEX - MIN_INDEX)

typedef struct { float32_t re, im; } cf32_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

message_notif_t message_notif;
static volatile int message_received;
static volatile int service_created;
volatile unsigned int received_data_str;
static struct rpmsg_endpoint rp_endpoint;

static volatile message_notif_t received_data;

__attribute__((section(".axi_sram"), aligned(32), used)) static float32_t fft_outputs[NUM_CHANNELS][SAMPLES_PER_CH * 2];
static arm_rfft_fast_instance_f32 fft_handler;
__attribute__((aligned(32))) static float32_t fft_mag[SAMPLES_PER_CH];

__attribute__((section(".axi_sram"), aligned(32), used)) int16_t axi_rx_buffer[NUM_CHANNELS * SAMPLES_PER_CH * 2];
__attribute__((section(".dtcm"), aligned(32), used)) int16_t dtcm_rx_buffer[NUM_CHANNELS * SAMPLES_PER_CH];

__attribute__((section(".dtcm"), used, aligned(4))) static float32_t scratch[SAMPLES_PER_CH];

__attribute__((section(".dtcm"), aligned(8))) static cf32_t   lut[LUT_SIZE];
__attribute__((section(".dtcm"), aligned(8))) static float32_t mic_s[16][2];
__attribute__((section(".dtcm"), aligned(8))) static cf32_t   acc[NBINS];
__attribute__((section(".dtcm"), aligned(32), used)) float32_t pix_k[ACOUSTIC_HORIZ][ACOUSTIC_VERT][2];
__attribute__((section(".axi_sram"), aligned(32), used)) float32_t power[ACOUSTIC_HORIZ][ACOUSTIC_VERT];
__attribute__((section(".axi_sram"), aligned(32), used)) float32_t max_freq[ACOUSTIC_HORIZ][ACOUSTIC_VERT];

static const float32_t MIC_POS[16][2] = {
  {38.075e-3, 18.160e-3}, 
  {68.283e-3, 29.155e-3}, 
  {11.973e-3, 8.659e-3},
  {21.294e-3, 12.054e-3},
  {-1.256e-3, 49.581e-3},
  {-0.555e-3, 29.508e-3},
  {-14.756e-3, 8.744e-3},
  {-0.165e-3, 18.357e-3},
  {-26.82e-3, 12.203e-3},
  {-48.537e-3, 18.43e-3},
  {-17.434e-3, -21.906e-3},
  {-31.647e-3, -42.975e-3},
  {12.538e-3, -11.216e-3},
  {-9.544e-3, -10.204e-3},
  {39.909e-3, -46.250e-3},
  {22.313e-3, -23.728e-3}
};
  

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
void service_destroy_cb(struct rpmsg_endpoint *ept);
void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest);

int send_text(const char *format, ...);
void send_power(void);

uint8_t buf_take(void *buffer);
uint8_t buf_release(void *buffer);

volatile uint8_t process_fft_1 = 0;
volatile uint8_t process_fft_2 = 0;

char *get_open_text(void);
uint8_t *get_open_ctrl(void);

void adc_setup(uint8_t addr, uint8_t master);

float32_t arm_tan(float32_t theta);
void calculate_k(void);

void beamform_init(void);
void beamform_frame(void);

own_list_t own_list = {ANY, ANY, ANY, ANY, ANY};
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */

  MAILBOX_Init();
  rpmsg_init_ept(&rp_endpoint, RPMSG_CHAN_NAME, RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);
  if (MX_OPENAMP_Init(RPMSG_MASTER, new_service_cb) != HAL_OK)
 	{
 		Error_Handler();
 	}

  OPENAMP_Wait_EndPointready(&rp_endpoint);

  // initialize both ADCs
  HAL_GPIO_WritePin(SHDNZ_GPIO_Port, SHDNZ_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SHDNZ_GPIO_Port, SHDNZ_Pin, GPIO_PIN_SET);

  calculate_k();

  HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)axi_rx_buffer, NUM_CHANNELS * SAMPLES_PER_CH * 2);

  HAL_Delay(100);

  adc_setup(ADC1_ADDR, 1);
  adc_setup(ADC2_ADDR, 0);

  uint8_t write_byte = 0x60;
  HAL_I2C_Mem_Write(&hi2c4, ADC1_ADDR, ADC5140_PWR_CFG, I2C_MEMADD_SIZE_8BIT, &write_byte, 1, 100);
  HAL_I2C_Mem_Write(&hi2c4, ADC2_ADDR, ADC5140_PWR_CFG, I2C_MEMADD_SIZE_8BIT, &write_byte, 1, 100);

  arm_rfft_fast_init_f32(&fft_handler, SAMPLES_PER_CH);
  beamform_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OPENAMP_check_for_message(); // ALWAYS KEEP THIS HERE. TRY TO AVOID BLOCKING.

    if (process_fft_1 || process_fft_2) {
      send_text("%lu", SystemCoreClock);
      uint8_t use_ping = process_fft_1;
      for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int n = 0; n < SAMPLES_PER_CH; n++) {
          scratch[n] = dtcm_rx_buffer[n * NUM_CHANNELS + ch];
        }
        arm_rfft_fast_f32(&fft_handler, scratch, fft_outputs[ch], 0);
      }
      if (use_ping) process_fft_1 = 0;
      else process_fft_2 = 0;

      beamform_frame();

      uint8_t max_x = 0;
      uint8_t max_y = 0;

      for (int x = 0; x < ACOUSTIC_HORIZ; x++) {
        for (int y = 0; y < ACOUSTIC_VERT; y++) {
          if (power[x][y] > power[max_x][max_y]) {
            max_x = x;
            max_y = y;
          }
        }
      }

      float32_t horiz_step = HORIZ_FOV / ACOUSTIC_HORIZ;
      float32_t vert_step = VERTICAL_FOV / ACOUSTIC_VERT;

      float32_t theta_x = (-HORIZ_FOV / 2 + horiz_step * (max_x + 0.5)) * 180 / PI;
      float32_t theta_y = (-VERTICAL_FOV / 2 + vert_step * (max_y + 0.5)) * 180 / PI;

      send_text("Freq: %f\tPower: %f\ttheta_x: %f\ttheta_y: %f", max_freq[max_x][max_y], power[max_x][max_y], theta_x, theta_y);

    }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_SMPS_1V8_SUPPLIES_LDO);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  // TODO: update this to parse the message and actually do stuff with it
  received_data = *((message_notif_t *) data);

  own_t* ptr = get_own_flag(received_data.address, &own_list);
  if (received_data.type == MSG_TAKE) {
    *ptr = M4;
  } else if (received_data.type == MSG_RELEASE) {
    *ptr = ANY;
  } else {
    *ptr = M7;
  }
  message_received=1;

  return 0;
}

void service_destroy_cb(struct rpmsg_endpoint *ept)
{
  /* this function is called while remote endpoint as been destroyed, the
   * service is no more available
   */
  service_created = 0;
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
  /* create a endpoint for rmpsg communication */
  OPENAMP_create_endpoint(&rp_endpoint, name, dest, rpmsg_recv_callback,
                          service_destroy_cb);
  service_created = 1;
}

char *get_open_text(void) {
  if (own_list.text_ping != M4)
    return text_ping;
  else if (own_list.text_pong != M4)
    return text_pong;
  else
    return NULL;
}

uint8_t *get_open_ctrl(void){
  if (own_list.control_ping != M4)
    return ctrl_ping;
  else if (own_list.control_pong != M4)
    return ctrl_pong;
  else
    return NULL;
}

int send_text(const char *format, ...) {
  OPENAMP_check_for_message();
  va_list args;
  int result;

  va_start(args, format);
  char *buffer = get_open_text();
  if (buffer == NULL)
    return 0;
  result = vsprintf(buffer, format, args);

  message_notif.address = buffer;
  message_notif.length = sizeof(buffer);
  message_notif.type = MSG_TEXT;
  OPENAMP_send(&rp_endpoint, &message_notif, sizeof(message_notif));

  *get_own_flag(buffer, &own_list) = M4;

  va_end(args);
  return result;
}

void send_power() {
  OPENAMP_check_for_message();
  memcpy(acoustic_power, power, sizeof(acoustic_power));
  memcpy(acoustic_freq, max_freq, sizeof(acoustic_power));
  
  message_notif.address = acoustic_power;
  message_notif.length = sizeof(acoustic_power);
  message_notif.type = MSG_ACOUSTIC;
  OPENAMP_send(&rp_endpoint, &message_notif, sizeof(message_notif));
  *get_own_flag(acoustic_power, &own_list) = M4;
  *get_own_flag(acoustic_freq, &own_list) = M4;
}

uint8_t buf_take(void *buffer) {
  *get_own_flag(buffer, &own_list) = M7;
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

void adc_setup(uint8_t addr, uint8_t master) {
  uint8_t write_byte = 0x81;
  uint8_t *write_ptr = &write_byte;

  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_SLEEP_CFG, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
  HAL_Delay(10);
  write_byte = 0x01;
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_ASI_CFG0, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);

  write_byte = 0x41;
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_GPO1_CFG, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_GPO2_CFG, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_GPO3_CFG, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_GPO4_CFG, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
  write_byte = 0x45;
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_GPI_CFG0, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
  write_byte = 0x67;
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_GPI_CFG1, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);

  uint8_t mic_setup[] = {0x40, 0x00, 0xFA, 0x80, 0x00};
  for (uint8_t mem_addr = ADC5140_CH1_CFG0; mem_addr <= ADC5140_CH3_CFG0; mem_addr += 5) {
    HAL_I2C_Mem_Write(&hi2c4, addr, mem_addr, I2C_MEMADD_SIZE_8BIT, mic_setup, 5, 100);
  }

  for (uint8_t mem_addr = ADC5140_CH4_CFG0; mem_addr <= ADC5140_CH8_CFG0; mem_addr += 5) {
    HAL_I2C_Mem_Write(&hi2c4, addr, mem_addr+2, I2C_MEMADD_SIZE_8BIT, mic_setup+2, 3, 100);
  }

  if (master) {
    write_byte = 0xA0;
    HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_ASI_CFG1, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
    write_byte = 0xB0;
    HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_GPIO1_CFG, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
    write_byte = 0x80;
    HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_ASI_CFG2, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
    write_byte = 0;
    for (uint8_t mem_addr = ADC5140_ASI_CH1; mem_addr <= ADC5140_ASI_CH8; mem_addr++) {
      HAL_I2C_Mem_Write(&hi2c4, addr, mem_addr, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
      write_byte++;
    }
  }
  else {
    write_byte = 0x80;
    HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_ASI_CFG1, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
    write_byte = 0x00;
    HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_ASI_CFG2, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);

    write_byte = 0; // TODO: check if this actually works.
    for (uint8_t mem_addr = ADC5140_ASI_CH1; mem_addr <= ADC5140_ASI_CH8; mem_addr++) {
      HAL_I2C_Mem_Write(&hi2c4, addr, mem_addr, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
      write_byte++;
    }
  }

  write_byte = 0xFF;
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_IN_CH_EN, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);
  HAL_I2C_Mem_Write(&hi2c4, addr, ADC5140_ASI_OUT_CH_EN, I2C_MEMADD_SIZE_8BIT, write_ptr, 1, 100);

}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
  send_text("SAI error!");
}
// Triggered when the first half of the AXI buffer is full
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if(hsai->Instance == hsai_BlockA1.Instance) // Verify it's the correct SAI block
  {
    // #ifdef DEBUG
    // if (process_fft_1 == 1)
    //   send_text("SAI overrun first half");
    // #endif
    if (!(process_fft_1 || process_fft_2)) {
      process_fft_1 = 1;
      uint32_t base = 0 ;
      SCB_InvalidateDCache_by_Addr((uint32_t*)&axi_rx_buffer[base],
                              NUM_CHANNELS * SAMPLES_PER_CH * sizeof(int16_t));
      memcpy(dtcm_rx_buffer, &axi_rx_buffer[base], sizeof(dtcm_rx_buffer));
    }
  }
}

// Triggered when the second half of the AXI buffer is full
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if(hsai->Instance == hsai_BlockA1.Instance)
  {
    // #ifdef DEBUG
    // if (process_fft_2 == 1)
    //   send_text("SAI overrun second half");
    // #endif
    if (!(process_fft_1 || process_fft_2)) {
      process_fft_2 = 1;
      uint32_t base = NUM_CHANNELS * SAMPLES_PER_CH;
      SCB_InvalidateDCache_by_Addr((uint32_t*)&axi_rx_buffer[base],
                              NUM_CHANNELS * SAMPLES_PER_CH * sizeof(int16_t));
      memcpy(dtcm_rx_buffer, &axi_rx_buffer[base], sizeof(dtcm_rx_buffer));
    }
  }
}

float32_t arm_tan(float32_t theta) {
  return arm_sin_f32(theta) / arm_cos_f32(theta);
}

void calculate_k() {
  float32_t horiz_step = HORIZ_FOV / ACOUSTIC_HORIZ;
  float32_t vert_step = VERTICAL_FOV / ACOUSTIC_VERT;

  for (int x = 0; x < ACOUSTIC_HORIZ; x++) {
    float theta_x = -HORIZ_FOV / 2 + horiz_step * (x + 0.5); 
    for (int y = 0; y < ACOUSTIC_VERT; y++) {
      float theta_y = -VERTICAL_FOV / 2 + vert_step * (y + 0.5);
      float32_t temp_k[] = {arm_tan(theta_x), arm_tan(theta_y), 1.0}; 
      float32_t mag;
      arm_sqrt_f32(arm_exponent_f32(temp_k[0], 2) + arm_exponent_f32(temp_k[1], 2) + arm_exponent_f32(temp_k[2], 2), &mag);
      pix_k[x][y][0] = temp_k[0] / (mag * 343);
      pix_k[x][y][1] = temp_k[1] / (mag * 343);
    }
  }
}

void beamform_init(void)
{
    for (uint32_t i = 0; i < LUT_SIZE; i++) {
        float32_t th = 2.0f * PI * (float32_t)i / (float32_t)LUT_SIZE;
        lut[i].re = cosf(th);
        lut[i].im = sinf(th);
    }
    for (int m = 0; m < 16; m++) {
        mic_s[m][0] = -MIC_POS[m][0] * SAMPLE_RATE;
        mic_s[m][1] = -MIC_POS[m][1] * SAMPLE_RATE;
    }
}

void beamform_frame(void)
{
  for (int x = 0; x < ACOUSTIC_HORIZ; x++) {
    for (int y = 0; y < ACOUSTIC_VERT; y++) {

      const float32_t *k = pix_k[x][y];

      /* ---- mic pair 0/1: '=' stores, so no acc-zeroing pass ---- */
      {
        float32_t d0 = mic_s[0][0]*k[0] + mic_s[0][1]*k[1];
        float32_t d1 = mic_s[1][0]*k[0] + mic_s[1][1]*k[1];
        uint32_t  s0 = (uint32_t)(int32_t)(d0 * PHASE_SCALE);
        uint32_t  s1 = (uint32_t)(int32_t)(d1 * PHASE_SCALE);
        uint32_t  p0 = s0 * (uint32_t)MIN_INDEX + PHASE_ROUND;
        uint32_t  p1 = s1 * (uint32_t)MIN_INDEX + PHASE_ROUND;

        const float32_t *X0 = &fft_outputs[0][2*MIN_INDEX];
        const float32_t *X1 = &fft_outputs[1][2*MIN_INDEX];

        for (int b = 0; b < NBINS; b++) {
          cf32_t w0 = lut[p0 >> PHASE_SHIFT];
          cf32_t w1 = lut[p1 >> PHASE_SHIFT];
          float32_t x0r = X0[2*b], x0i = X0[2*b+1];
          float32_t x1r = X1[2*b], x1i = X1[2*b+1];
          acc[b].re = (x0r*w0.re - x0i*w0.im) + (x1r*w1.re - x1i*w1.im);
          acc[b].im = (x0r*w0.im + x0i*w0.re) + (x1r*w1.im + x1i*w1.re);
          p0 += s0;
          p1 += s1;
        }
      }

      /* ---- mic pairs 2..15: accumulate ---- */
      for (int mm = 2; mm < 16; mm += 2) {
        float32_t d0 = mic_s[mm  ][0]*k[0] + mic_s[mm  ][1]*k[1];
        float32_t d1 = mic_s[mm+1][0]*k[0] + mic_s[mm+1][1]*k[1];
        uint32_t  s0 = (uint32_t)(int32_t)(d0 * PHASE_SCALE);
        uint32_t  s1 = (uint32_t)(int32_t)(d1 * PHASE_SCALE);
        uint32_t  p0 = s0 * (uint32_t)MIN_INDEX + PHASE_ROUND;
        uint32_t  p1 = s1 * (uint32_t)MIN_INDEX + PHASE_ROUND;

        const float32_t *X0 = &fft_outputs[mm  ][2*MIN_INDEX];
        const float32_t *X1 = &fft_outputs[mm+1][2*MIN_INDEX];

        for (int b = 0; b < NBINS; b++) {
          cf32_t w0 = lut[p0 >> PHASE_SHIFT];
          cf32_t w1 = lut[p1 >> PHASE_SHIFT];
          float32_t ar  = acc[b].re, ai  = acc[b].im;
          float32_t x0r = X0[2*b],   x0i = X0[2*b+1];
          float32_t x1r = X1[2*b],   x1i = X1[2*b+1];
          ar += x0r*w0.re - x0i*w0.im;
          ai += x0r*w0.im + x0i*w0.re;
          ar += x1r*w1.re - x1i*w1.im;
          ai += x1r*w1.im + x1i*w1.re;
          acc[b].re = ar;
          acc[b].im = ai;
          p0 += s0;
          p1 += s1;
        }
      }

      /* ---- power + per-pixel peak bin ---- */
      float32_t total = 0.0f, best = -1.0f;
      uint32_t  best_b = 0;
      for (int b = 0; b < NBINS; b++) {
        float32_t magsq = acc[b].re*acc[b].re + acc[b].im*acc[b].im;
        if (magsq > best) { best = magsq; best_b = b; }
        total += magsq;
      }
      power[x][y]    = total;
      max_freq[x][y] = (float32_t)(best_b + MIN_INDEX) * SAMPLE_RATE / SAMPLES_PER_CH;
    }
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
