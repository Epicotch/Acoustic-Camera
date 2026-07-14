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
#include "bdma.h"
#include "dma.h"
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

__attribute__((section(".dtcm"), aligned(32), used)) static q15_t fft_outputs[NUM_CHANNELS][SAMPLES_PER_CH * 2];
static arm_rfft_instance_q15 fft_handler;
__attribute__((aligned(32))) static q15_t fft_mag[SAMPLES_PER_CH];

__attribute__((section(".axi_sram"), aligned(32), used)) int16_t axi_rx_buffer[NUM_CHANNELS * SAMPLES_PER_CH * 2];

__attribute__((section(".dtcm"), used, aligned(4))) static q15_t scratch[SAMPLES_PER_CH];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
void service_destroy_cb(struct rpmsg_endpoint *ept);
void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest);

int send_text(const char *format, ...);

uint8_t buf_take(void *buffer);
uint8_t buf_release(void *buffer);

uint8_t process_fft_1 = 0;
uint8_t process_fft_2 = 0;

strength_t *get_open_acoustic(void);
char *get_open_text(void);
uint8_t *get_open_ctrl(void);

void adc_setup(uint8_t addr, uint8_t master);

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

  uint32_t count = 0;

  // initialize both ADCs
  HAL_GPIO_WritePin(SHDNZ_GPIO_Port, SHDNZ_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SHDNZ_GPIO_Port, SHDNZ_Pin, GPIO_PIN_SET);

  HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)axi_rx_buffer, NUM_CHANNELS * SAMPLES_PER_CH * 2);

  HAL_Delay(100);

  adc_setup(ADC1_ADDR, 1);
  adc_setup(ADC2_ADDR, 0);

  uint8_t write_byte = 0x60;
  HAL_I2C_Mem_Write(&hi2c4, ADC1_ADDR, ADC5140_PWR_CFG, I2C_MEMADD_SIZE_8BIT, &write_byte, 1, 100);
  HAL_I2C_Mem_Write(&hi2c4, ADC2_ADDR, ADC5140_PWR_CFG, I2C_MEMADD_SIZE_8BIT, &write_byte, 1, 100);

  arm_rfft_init_q15(&fft_handler, SAMPLES_PER_CH, 0, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OPENAMP_check_for_message(); // ALWAYS KEEP THIS HERE. TRY TO AVOID BLOCKING. 

    if (process_fft_1 || process_fft_2) {
      uint8_t use_ping = process_fft_1;
      uint32_t base = use_ping ? 0 : NUM_CHANNELS * SAMPLES_PER_CH;
      SCB_InvalidateDCache_by_Addr((uint32_t*)&axi_rx_buffer[use_ping ? 0 : NUM_CHANNELS*SAMPLES_PER_CH],
                             NUM_CHANNELS * SAMPLES_PER_CH * sizeof(int16_t));
      for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int n = 0; n < SAMPLES_PER_CH; n++) {
          scratch[n] = axi_rx_buffer[n * NUM_CHANNELS + ch + base];
        }
        arm_rfft_q15(&fft_handler, scratch, fft_outputs[ch]);
        if (process_fft_1 == 1) {
          process_fft_1 = 0;
        }
        else {
          process_fft_2 = 0;
        }
      }
      arm_cmplx_mag_q15(fft_outputs[0], fft_mag, SAMPLES_PER_CH / 2); // TODO: figure out why no channels work except the first :sob:
      uint32_t max_ind = 1;
      uint32_t prev_max = max_ind;
      for (int i = max_ind; i < SAMPLES_PER_CH / 2; i++) {
        if (fft_mag[i] > fft_mag[max_ind]) {
          prev_max = max_ind;
          max_ind = i;
        }
      }
      // max_ind = 64;
      float freq = max_ind * 48000.0f / SAMPLES_PER_CH;
      send_text("Freq: %f, Mag: %d", freq, fft_mag[max_ind]);


      // uint8_t read_status[5];
      // HAL_I2C_Mem_Read(&hi2c4, ADC1_ADDR, 0x76, I2C_MEMADD_SIZE_8BIT, read_status, 2, 100);
      // HAL_I2C_Mem_Read(&hi2c4, ADC1_ADDR, 0x15, I2C_MEMADD_SIZE_8BIT, read_status+2, 1, 100);
      // HAL_I2C_Mem_Read(&hi2c4, ADC1_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, read_status+3, 1, 100);
      // HAL_I2C_Mem_Read(&hi2c4, ADC1_ADDR, ADC5140_GPIO1_CFG, I2C_MEMADD_SIZE_8BIT, read_status+4, 1, 100);
      // send_text("ADC 1 DEV_STS0: %#x STS2: %#x: ASI_STS: %#x INT_LTCH0: %#x GPIO: %#x", read_status[0], read_status[1], read_status[2], read_status[3], read_status[4]);
      // HAL_I2C_Mem_Read(&hi2c4, ADC2_ADDR, 0x76, I2C_MEMADD_SIZE_8BIT, read_status, 2, 100);
      // HAL_I2C_Mem_Read(&hi2c4, ADC2_ADDR, 0x15, I2C_MEMADD_SIZE_8BIT, read_status+2, 1, 100);
      // HAL_I2C_Mem_Read(&hi2c4, ADC2_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, read_status+3, 1, 100);
      // HAL_I2C_Mem_Read(&hi2c4, ADC2_ADDR, ADC5140_GPIO1_CFG, I2C_MEMADD_SIZE_8BIT, read_status+4, 1, 100);
      // send_text("ADC 2 DEV_STS0: %#x STS2: %#x: ASI_STS: %#x INT_LTCH0: %#x GPIO: %#x", read_status[0], read_status[1], read_status[2], read_status[3], read_status[4]);
    }

    // send_text("Count: %u", count);
    // count++;
    // HAL_Delay(1000);

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

strength_t *get_open_acoustic(void) {
  if (own_list.acoustic_ping != M4)
    return acoustic_ping;
  else if (own_list.acoustic_pong != M4)
    return acoustic_pong;
  else
    return NULL;
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
    #ifdef DEBUG
    if (process_fft_1 == 1)
      send_text("SAI overrun first half");
    #endif

    process_fft_1 = 1;
  }
}

// Triggered when the second half of the AXI buffer is full
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if(hsai->Instance == hsai_BlockA1.Instance)
  {
    #ifdef DEBUG
    if (process_fft_2 == 1)
      send_text("SAI overrun second half");
    #endif

    process_fft_2 = 1;
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
