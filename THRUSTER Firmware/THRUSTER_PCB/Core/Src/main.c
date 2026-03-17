/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_ESP_CMD      0x203
#define CAN_ID_THRUST_TX    0x102
#define CAN_ID_HEARTBEAT_TX 0x103

#define CMD_THRUST_0   'D'
#define CMD_THRUST_50  'E'
#define CMD_THRUST_100 'F'
#define CMD_LED1_ON    'G'
#define CMD_LED2_ON    'H'
#define CMD_LED3_ON    'I'
#define CMD_BLINK_ALL  'J'

#define LED_BLINK_MS          100
#define HEARTBEAT_MS          200
#define PWM_ARR               4200
#define START_DELAY_MS        8
#define END_DELAY_MS          2
#define RAMP_TIME_MS          5000
#define RUN_TIME_MS           6000
#define FAULT_CLEAR_TIMEOUT_MS 500
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
static CAN_RxHeaderTypeDef rxHeader;
static uint8_t rxData[8];

static CAN_TxHeaderTypeDef txHeader;
static uint8_t txData[8];
static uint32_t txMailbox;

static uint32_t lastLed1Off  = 0;
static uint32_t lastHeartbeat = 0;
static uint8_t  stmStatus    = 0x01;

static uint16_t          adc_buffer[4];
static uint8_t           step            = 0;
static uint32_t          motor_start_time = 0;
static uint32_t          last_step_time   = 0;
static uint8_t           motor_stopped    = 1;
static uint32_t          current_duty     = 0;
static volatile uint8_t  pending_esp_cmd  = 0;
static volatile uint8_t  pending_esp_seq  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef configCANFilter(void);
static HAL_StatusTypeDef sendHeartbeat(void);
static HAL_StatusTypeDef sendThrustTelemetry(uint8_t percentage, uint16_t ch3);
static HAL_StatusTypeDef sendAck302(uint8_t sequence, uint8_t cmd_echo,
                                        uint8_t status);
static void handleESPCommand(uint8_t command);

void commutateMotor(uint8_t step_idx, uint32_t duty);
void stopMotor(uint8_t fault);
static void startMotor(uint32_t duty);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void commutateMotor(uint8_t step_idx, uint32_t duty) {
#define ENABLE_PHASE_A()                                                       \
  do {                                                                         \
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);                            \
  } while (0)
#define DISABLE_PHASE_A()                                                      \
  do {                                                                         \
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);                           \
  } while (0)
#define ENABLE_PHASE_B()                                                       \
  do {                                                                         \
    TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);                            \
  } while (0)
#define DISABLE_PHASE_B()                                                      \
  do {                                                                         \
    TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);                           \
  } while (0)
#define ENABLE_PHASE_C()                                                       \
  do {                                                                         \
    TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);                            \
  } while (0)
#define DISABLE_PHASE_C()                                                      \
  do {                                                                         \
    TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);                           \
  } while (0)

  switch (step_idx) {
  case 0:
    ENABLE_PHASE_A();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    ENABLE_PHASE_B();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    DISABLE_PHASE_C();
    break;
  case 1:
    ENABLE_PHASE_A();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    DISABLE_PHASE_B();
    ENABLE_PHASE_C();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    break;
  case 2:
    DISABLE_PHASE_A();
    ENABLE_PHASE_B();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
    ENABLE_PHASE_C();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    break;
  case 3:
    ENABLE_PHASE_A();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    ENABLE_PHASE_B();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
    DISABLE_PHASE_C();
    break;
  case 4:
    ENABLE_PHASE_A();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    DISABLE_PHASE_B();
    ENABLE_PHASE_C();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);
    break;
  case 5:
    DISABLE_PHASE_A();
    ENABLE_PHASE_B();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    ENABLE_PHASE_C();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);
    break;
  default:
    DISABLE_PHASE_A();
    DISABLE_PHASE_B();
    DISABLE_PHASE_C();
    break;
  }
}

void stopMotor(uint8_t fault) {
  __HAL_TIM_MOE_DISABLE(&htim1);
  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                  TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
  HAL_GPIO_WritePin(GPIOB, EN_GATE_OUT_Pin, GPIO_PIN_RESET);
  motor_stopped = 1;
  if (fault) {
    stmStatus = 0xFF;
    HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
  } else {
    stmStatus = 0x01;
    HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
  }
}

static void startMotor(uint32_t duty) {
  if (motor_stopped) {
    HAL_GPIO_WritePin(GPIOB, EN_GATE_OUT_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    __HAL_TIM_MOE_ENABLE(&htim1);
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                   TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
    HAL_Delay(5);
    motor_start_time = HAL_GetTick();
    last_step_time   = motor_start_time;
    step             = 0;
    motor_stopped    = 0;
    stmStatus        = 0x02;
  }
  current_duty = duty;
}

static HAL_StatusTypeDef configCANFilter(void) {
  CAN_FilterTypeDef f = {0};
  f.FilterBank          = 0;
  f.FilterMode          = CAN_FILTERMODE_IDMASK;
  f.FilterScale         = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh        = (CAN_ID_ESP_CMD << 5) & 0xFFFF;
  f.FilterIdLow         = 0x0000;
  f.FilterMaskIdHigh    = (0x7FF << 5) & 0xFFFF;
  f.FilterMaskIdLow     = 0x0000;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation    = ENABLE;
  return HAL_CAN_ConfigFilter(&hcan1, &f);
}

static HAL_StatusTypeDef Safe_CAN_Tx(CAN_TxHeaderTypeDef *pHeader,
                                     uint8_t aData[]) {
  uint32_t t_start = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    if (HAL_GetTick() - t_start > 50)
      return HAL_TIMEOUT;
  }
  return HAL_CAN_AddTxMessage(&hcan1, pHeader, aData, &txMailbox);
}

static HAL_StatusTypeDef sendHeartbeat(void) {
  uint32_t uptime = HAL_GetTick();
  txHeader.StdId              = CAN_ID_HEARTBEAT_TX;
  txHeader.ExtId              = 0;
  txHeader.RTR                = CAN_RTR_DATA;
  txHeader.IDE                = CAN_ID_STD;
  txHeader.DLC                = 5;
  txHeader.TransmitGlobalTime = DISABLE;
  txData[0] = stmStatus;
  txData[1] = uptime >> 24;
  txData[2] = uptime >> 16;
  txData[3] = uptime >> 8;
  txData[4] = uptime;
  return Safe_CAN_Tx(&txHeader, txData);
}

static HAL_StatusTypeDef sendThrustTelemetry(uint8_t percentage, uint16_t ch3) {
  txHeader.StdId              = CAN_ID_THRUST_TX;
  txHeader.ExtId              = 0;
  txHeader.RTR                = CAN_RTR_DATA;
  txHeader.IDE                = CAN_ID_STD;
  txHeader.DLC                = 3;
  txHeader.TransmitGlobalTime = DISABLE;
  txData[0] = percentage;
  txData[1] = ch3 >> 8;
  txData[2] = ch3;
  return Safe_CAN_Tx(&txHeader, txData);
}



static HAL_StatusTypeDef sendAck302(uint8_t sequence, uint8_t cmd_echo,
                                        uint8_t status) {
  CAN_TxHeaderTypeDef ackHdr;
  memset(&ackHdr, 0, sizeof(ackHdr));
  ackHdr.StdId              = 0x302;
  ackHdr.ExtId              = 0;
  ackHdr.RTR                = CAN_RTR_DATA;
  ackHdr.IDE                = CAN_ID_STD;
  ackHdr.DLC                = 4;
  ackHdr.TransmitGlobalTime = DISABLE;
  uint8_t ackData[4] = {sequence, cmd_echo, status, 0};
  return Safe_CAN_Tx(&ackHdr, ackData);
}

static void handleESPCommand(uint8_t command) {
  switch (command) {
  case CMD_THRUST_0:
    stopMotor(0);
    sendThrustTelemetry(0, 0);
    sendAck302(pending_esp_seq, command, 0);
    break;
  case CMD_THRUST_50:
    startMotor(420);
    sendThrustTelemetry(50, 420);
    sendAck302(pending_esp_seq, command, 0);
    break;
  case CMD_THRUST_100:
    startMotor(600);
    sendThrustTelemetry(100, 600);
    sendAck302(pending_esp_seq, command, 0);
    break;

  case CMD_LED1_ON:
    HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
    sendAck302(pending_esp_seq, command, 0);
    break;
  case CMD_LED2_ON:
    HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
    sendAck302(pending_esp_seq, command, 0);
    break;
  case CMD_LED3_ON:
    HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
    sendAck302(pending_esp_seq, command, 0);
    break;
  case CMD_BLINK_ALL:
    for (int i = 0; i < 3; i++) {
      HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
    }
    HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
    sendAck302(pending_esp_seq, command, 0);
    break;

  default:
    HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_RESET);
    break;
  }
}

/* USER CODE END 0 */

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 4) != HAL_OK) {
    stopMotor(1);
  }

  HAL_GPIO_WritePin(GPIOB, DC_CAL_Out_Pin, GPIO_PIN_RESET);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                  TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  HAL_GPIO_WritePin(GPIOB, EN_GATE_OUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, EN_GATE_OUT_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  uint32_t t_startup = HAL_GetTick();
  while (HAL_GPIO_ReadPin(GPIOB, FAULT_Pin) == GPIO_PIN_RESET) {
    if ((HAL_GetTick() - t_startup) > FAULT_CLEAR_TIMEOUT_MS) {
      stopMotor(1);
    }
    HAL_Delay(10);
  }
  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);

  if (configCANFilter() != HAL_OK) {
    while (1) {
      HAL_GPIO_TogglePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin);
      HAL_Delay(200);
    }
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    while (1) {
      HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
      HAL_Delay(100);
    }
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  while (1) {
    /* USER CODE BEGIN 3 */
    if (pending_esp_cmd != 0) {
      handleESPCommand(pending_esp_cmd);
      pending_esp_cmd = 0;
    }

    uint32_t now = HAL_GetTick();

    if (lastLed1Off != 0 && (now - lastLed1Off) >= LED_BLINK_MS) {
      HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
      lastLed1Off = 0;
    }

    if ((now - lastHeartbeat) >= HEARTBEAT_MS) {
      lastHeartbeat = now;
      sendHeartbeat();
    }

    if (!motor_stopped) {
      uint32_t t = now - motor_start_time;
      if (t >= RUN_TIME_MS) {
        stopMotor(0);
        sendThrustTelemetry(0, 0);
      } else {
        if (HAL_GPIO_ReadPin(GPIOB, FAULT_Pin) == GPIO_PIN_RESET) {
          stopMotor(1);
        } else if (HAL_GPIO_ReadPin(GPIOB, OCTW_Pin) == GPIO_PIN_RESET) {
          stopMotor(1);
        } else {
          uint32_t t_clamped;
          if (t < RAMP_TIME_MS) {
            t_clamped = t;
          } else {
            t_clamped = RAMP_TIME_MS;
          }
          uint32_t current_delay =
              START_DELAY_MS -
              ((START_DELAY_MS - END_DELAY_MS) * t_clamped) / RAMP_TIME_MS;
          if (now - last_step_time >= current_delay) {
            last_step_time = now;
            commutateMotor(step, current_duty);
            step++;
            if (step >= 6)
              step = 0;
          }
        }
      }
    }
    /* USER CODE END 3 */
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 336;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance                = ADC1;
  hadc1.Init.ClockPrescaler     = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution         = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode       = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode   = DISABLE;
  hadc1.Init.ExternalTrigConvEdge    = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv        = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign               = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion         = 4;
  hadc1.Init.DMAContinuousRequests   = ENABLE;
  hadc1.Init.EOCSelection            = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel      = ADC_CHANNEL_2;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank    = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel      = ADC_CHANNEL_0;
  sConfig.Rank         = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank    = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_CAN1_Init(void) {
  hcan1.Instance                  = CAN1;
  hcan1.Init.Prescaler            = 21;
  hcan1.Init.Mode                 = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1             = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode    = DISABLE;
  hcan1.Init.AutoBusOff           = DISABLE;
  hcan1.Init.AutoWakeUp           = DISABLE;
  hcan1.Init.AutoRetransmission   = ENABLE;
  hcan1.Init.ReceiveFifoLocked    = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef       sClockSourceConfig  = {0};
  TIM_MasterConfigTypeDef      sMasterConfig       = {0};
  TIM_OC_InitTypeDef           sConfigOC           = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period            = 4200;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 50;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, EN_GATE_OUT_Pin | DC_CAL_Out_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = LED1_Pin | LED2_Pin | LED3_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = EN_GATE_OUT_Pin | DC_CAL_Out_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = FAULT_Pin | OCTW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
      HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
      lastLed1Off = HAL_GetTick();
      if (rxHeader.StdId == CAN_ID_ESP_CMD && rxHeader.DLC >= 2) {
        pending_esp_seq = rxData[0];
        pending_esp_cmd = rxData[1];
      }
    }
  }
}

/* USER CODE END 4 */

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
