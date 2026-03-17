/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAN_ID_ESP_PRESET 0x201
#define CAN_ID_SERVO_COMMAND  0x202
#define CAN_ID_SERVO_FEEDBACK   0x104
#define CAN_ID_SERVO_HEARTBEAT   0x105
#define CAN_ID_SERVO_ACK 0x301

#define DIVE      'P'
#define SURFACE   'Q'
#define ROLL_PORT 'R'
#define ROLL_STBD 'S'
#define CENTER    'N'

#define SERVO_CLOSED    500
#define SERVO_OPEN      2400
#define SERVO_NEUTRAL   1450

#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180

#define DOWN_LEFT        45
#define DOWN_RIGHT       135
#define UP_LEFT          135
#define UP_RIGHT         45
#define ROLL_OFFSET      45

#define TELEMETRY_INTERVAL    100
#define HEARTBEAT_INTERVAL    200
#define CAN_STARTUP_DELAY     3000
#define INA226_READ_INTERVAL  250

#define INA226_ADDR        0x40

#define INA_REG_CONFIG      0x00
#define INA_REG_SHUNT_VOLT  0x01
#define INA_REG_BUS_VOLT    0x02
#define INA_REG_POWER       0x03
#define INA_REG_CURRENT     0x04
#define INA_REG_CALIB       0x05
#define INA_REG_MASK_EN     0x06
#define INA_REG_DEVICE_ID   0xFF

#define INA_CONFIG_DEFAULT    0x4527
#define INA_CALIB_DEFAULT     25600
#define INA_CURRENT_LSB_UA    100

#define CAN_ID_POWER_FEEDBACK   0x106
#define CAN_ID_SHUNT_DIAGNOSTIC 0x107
#define CAN_ID_REG_DIAGNOSTIC   0x108

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

static CAN_TxHeaderTypeDef txHeader;
static uint32_t txMailbox;
static uint8_t feedbackBuffer[8];
static uint8_t heartbeatBuffer[8];

static volatile uint8_t  canMsgReceived = 0;
static volatile uint8_t  canRxBuffer[8];
static volatile uint8_t  canRxLength = 0;
static volatile uint16_t canRxId = 0;

static uint8_t  servoAngle[4] = {90, 90, 90, 90};
static uint16_t servoPosition[4] = {SERVO_NEUTRAL, SERVO_NEUTRAL,
                                  SERVO_NEUTRAL, SERVO_NEUTRAL};

static const uint32_t servoChannel[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2,
                                          TIM_CHANNEL_3, TIM_CHANNEL_4};

static uint32_t lastTelemetryTick = 0;
static uint32_t lastHeartbeatTick = 0;
static uint32_t lastInaReadTick   = 0;
static uint8_t  stmStatus = 0x01;

static uint16_t busVoltage      = 0;
static int16_t  current_mA      = 0;
static uint16_t power_mW        = 0;
static uint8_t  inaReady        = 0;
static int16_t  shuntRaw        = 0;
static uint16_t peakShuntVoltage = 0;

static int16_t  lastCurrentReading = 0;
static uint16_t lastPowerReading   = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static uint16_t setPWM(uint8_t angle);
static void setServoAngle(uint8_t ch, uint8_t angle);
static void processCANCommand(void);
static void sendACK(uint8_t seq, uint8_t cmd_echo, uint8_t status);
static void sendServoFeedback(void);
static void sendHeartbeat(void);
static void sendPowerTelemetry(void);
static void sendShuntDiagnostic(void);
static void MX_I2C1_Init(void);
static HAL_StatusTypeDef INA226_WriteReg(uint8_t reg, uint16_t value);
static HAL_StatusTypeDef INA226_ReadReg(uint8_t reg, uint16_t *value);
static uint8_t INA226_Init(void);
static void INA226_Read(void);
static void sendRegDiagnostic(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static uint16_t setPWM(uint8_t angle) {
  if (angle > SERVO_ANGLE_MAX)
    angle = SERVO_ANGLE_MAX;
  return SERVO_CLOSED +
         ((uint32_t)angle * (SERVO_OPEN - SERVO_CLOSED)) /
             SERVO_ANGLE_MAX;
}

static void setServoAngle(uint8_t ch, uint8_t angle) {
  if (ch > 3)
    return;
  if (angle > SERVO_ANGLE_MAX)
    angle = SERVO_ANGLE_MAX;

  servoAngle[ch] = angle;

  uint8_t physAngle = angle;
  if (ch == 3) {
    physAngle = SERVO_ANGLE_MAX - angle;
  }

  servoPosition[ch] = setPWM(physAngle);
  __HAL_TIM_SET_COMPARE(&htim3, servoChannel[ch], servoPosition[ch]);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef tempHeader;
  uint8_t tempData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &tempHeader, tempData) == HAL_OK) {
    for (uint8_t i = 0; i < tempHeader.DLC && i < 8; i++) {
      canRxBuffer[i] = tempData[i];
    }
    for (uint8_t i = tempHeader.DLC; i < 8; i++) {
      canRxBuffer[i] = 0;
    }
    canRxLength = tempHeader.DLC;
    canRxId  = tempHeader.StdId;
    canMsgReceived = 1;
  }
}

static void sendACK(uint8_t seq, uint8_t cmd_echo, uint8_t status) {
  CAN_TxHeaderTypeDef ackHdr;
  uint8_t ackData[4];
  uint32_t ackMailbox;

  ackHdr.StdId = CAN_ID_SERVO_ACK;
  ackHdr.ExtId = 0;
  ackHdr.RTR   = CAN_RTR_DATA;
  ackHdr.IDE   = CAN_ID_STD;
  ackHdr.DLC   = 4;
  ackHdr.TransmitGlobalTime = DISABLE;

  ackData[0] = seq;
  ackData[1] = cmd_echo;
  ackData[2] = status;
  ackData[3] = 0;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1) {
    HAL_CAN_AddTxMessage(&hcan1, &ackHdr, ackData, &ackMailbox);
  }
}

static void processCANCommand(void) {
  uint16_t id = canRxId;

  if (id == CAN_ID_ESP_PRESET) {
    if (canRxLength < 2)
      return;
    uint8_t sequence = canRxBuffer[0];
    uint8_t command  = canRxBuffer[1];
    uint8_t ackStatus = 0;

    switch (command) {
    case DIVE:
      setServoAngle(0, DOWN_LEFT);
      setServoAngle(1, DOWN_RIGHT);
      break;

    case SURFACE:
      setServoAngle(0, UP_LEFT);
      setServoAngle(1, UP_RIGHT);
      break;

    case ROLL_PORT:
      setServoAngle(2, 90 - ROLL_OFFSET);
      setServoAngle(3, 90 + ROLL_OFFSET);
      break;

    case ROLL_STBD:
      setServoAngle(2, 90 + ROLL_OFFSET);
      setServoAngle(3, 90 - ROLL_OFFSET);
      break;

    case CENTER:
      setServoAngle(0, 90);
      setServoAngle(1, 90);
      setServoAngle(2, 90);
      setServoAngle(3, 90);
      break;

    default:
      ackStatus = 1;
      break;
    }
    sendACK(sequence, command, ackStatus);

  } else if (id == CAN_ID_SERVO_COMMAND) {
    if (canRxLength < 3)
      return;
    uint8_t sequence = canRxBuffer[0];
    uint8_t ch    = canRxBuffer[1];
    uint8_t angle = canRxBuffer[2];
    if (ch < 4) {
      setServoAngle(ch, angle);
      sendACK(sequence, 'S', 0);
    } else {
      sendACK(sequence, 'S', 2);
    }
  }
}

static void sendServoFeedback(void) {
  memset(feedbackBuffer, 0, sizeof(feedbackBuffer));

  for (uint8_t i = 0; i < 4; i++) {
    uint16_t tenths = servoAngle[i] * 10;
    feedbackBuffer[i * 2]     = tenths >> 8;
    feedbackBuffer[i * 2 + 1] = tenths & 0xFF;
  }

  txHeader.StdId = CAN_ID_SERVO_FEEDBACK;
  txHeader.DLC   = 8;

  HAL_CAN_AddTxMessage(&hcan1, &txHeader, feedbackBuffer, &txMailbox);
}

static void sendHeartbeat(void) {
  uint32_t uptime = HAL_GetTick();

  memset(heartbeatBuffer, 0, sizeof(heartbeatBuffer));

  heartbeatBuffer[0] = stmStatus;
  heartbeatBuffer[1] = (uptime >> 24) & 0xFF;
  heartbeatBuffer[2] = (uptime >> 16) & 0xFF;
  heartbeatBuffer[3] = (uptime >> 8)  & 0xFF;
  heartbeatBuffer[4] = uptime & 0xFF;

  txHeader.StdId = CAN_ID_SERVO_HEARTBEAT;
  txHeader.DLC   = 5;

  HAL_CAN_AddTxMessage(&hcan1, &txHeader, heartbeatBuffer, &txMailbox);
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

static HAL_StatusTypeDef INA226_WriteReg(uint8_t reg, uint16_t value) {
  uint8_t buf[3];
  buf[0] = reg;
  buf[1] = value >> 8;
  buf[2] = value & 0xFF;
  return HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR << 1, buf, 3, 10);
}

static HAL_StatusTypeDef INA226_ReadReg(uint8_t reg, uint16_t *value) {
  uint8_t buf[2];
  if (HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR << 1, &reg, 1, 10) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_I2C_Master_Receive(&hi2c1, INA226_ADDR << 1, buf, 2, 10) != HAL_OK) {
    return HAL_ERROR;
  }
  *value = ((uint16_t)buf[0] << 8) | buf[1];
  return HAL_OK;
}

static uint8_t INA226_Init(void) {
  uint16_t id;
  if (INA226_ReadReg(INA_REG_DEVICE_ID, &id) != HAL_OK)
    return 0;
  if (id != 0x2260)
    return 0;

  INA226_WriteReg(INA_REG_CONFIG, 0x8000);
  HAL_Delay(1);

  INA226_WriteReg(INA_REG_CONFIG, INA_CONFIG_DEFAULT);

  uint16_t configCheck;
  if (INA226_ReadReg(INA_REG_CONFIG, &configCheck) != HAL_OK ||
      configCheck != INA_CONFIG_DEFAULT) {
    return 0;
  }

  INA226_WriteReg(INA_REG_CALIB, INA_CALIB_DEFAULT);

  uint16_t calibrationCheck;
  if (INA226_ReadReg(INA_REG_CALIB, &calibrationCheck) != HAL_OK ||
      calibrationCheck != INA_CALIB_DEFAULT) {
    return 0;
  }

  return 1;
}

static void INA226_Read(void) {
  uint16_t raw;

  if (INA226_ReadReg(INA_REG_BUS_VOLT, &raw) == HAL_OK) {
    busVoltage = (uint16_t)(((uint32_t)raw * 5) / 4);
  }

  if (INA226_ReadReg(INA_REG_SHUNT_VOLT, &raw) != HAL_OK) {
    return;
  }

  int16_t shunt = (int16_t)raw;
  shuntRaw = shunt;

  uint16_t absShunt;
  if (shunt < 0) {
    absShunt = -shunt;
  } else {
    absShunt = shunt;
  }

  static uint8_t regDumpSent = 0;
  if (absShunt >= 0x7FFF && !regDumpSent) {
    regDumpSent = 1;
    sendRegDiagnostic();
  } else if (absShunt < 0x4000) {
    regDumpSent = 0;
  }

  if (absShunt >= 0x7FFF) {
    uint16_t raw2;
    if (INA226_ReadReg(INA_REG_SHUNT_VOLT, &raw2) == HAL_OK) {
      int16_t  shunt2 = (int16_t)raw2;
      uint16_t abs2;
      if (shunt2 < 0) {
        abs2 = -shunt2;
      } else {
        abs2 = shunt2;
      }
      if (abs2 < 0x7FFF) {
        shunt    = shunt2;
        absShunt = abs2;
      } else {
        if (absShunt > peakShuntVoltage) {
          peakShuntVoltage = absShunt;
        }
        current_mA = lastCurrentReading;
        power_mW   = lastPowerReading;
        return;
      }
    } else {
      current_mA = lastCurrentReading;
      power_mW   = lastPowerReading;
      return;
    }
  }

  if (absShunt > peakShuntVoltage) {
    peakShuntVoltage = absShunt;
  }

  int32_t temp = ((int32_t)shunt * 5) / 4;
  if (temp >  32767) temp =  32767;
  if (temp < -32767) temp = -32767;
  current_mA = -temp;

  uint16_t absoluteCurrent;
  if (current_mA < 0) {
    absoluteCurrent = -current_mA;
  } else {
    absoluteCurrent = current_mA;
  }

  uint32_t power = ((uint32_t)absoluteCurrent * busVoltage) / 1000;
  if (power > 65535) {
    power_mW = 65535;
  } else {
    power_mW = power;
  }

  lastCurrentReading = current_mA;
  lastPowerReading   = power_mW;
}

static void sendPowerTelemetry(void) {
  uint8_t data[7];

  data[0] = busVoltage >> 8;
  data[1] = busVoltage & 0xFF;
  data[2] = (uint16_t)current_mA >> 8;
  data[3] = (uint16_t)current_mA & 0xFF;
  data[4] = power_mW >> 8;
  data[5] = power_mW & 0xFF;
  if (inaReady) {
    data[6] = 0x01;
  } else {
    data[6] = 0x00;
  }

  txHeader.StdId = CAN_ID_POWER_FEEDBACK;
  txHeader.DLC   = 7;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1) {
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
  }
}

static void sendShuntDiagnostic(void) {
  uint8_t  data[4];
  uint16_t shuntWord = shuntRaw;

  data[0] = shuntWord >> 8;
  data[1] = shuntWord & 0xFF;
  data[2] = peakShuntVoltage >> 8;
  data[3] = peakShuntVoltage & 0xFF;

  peakShuntVoltage = 0;

  txHeader.StdId = CAN_ID_SHUNT_DIAGNOSTIC;
  txHeader.DLC   = 4;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1) {
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
  }
}

static void sendRegDiagnostic(void) {
  uint16_t regShunt = 0, regBus = 0, regConfig = 0, regCalib = 0;
  INA226_ReadReg(INA_REG_SHUNT_VOLT, &regShunt);
  INA226_ReadReg(INA_REG_BUS_VOLT,   &regBus);
  INA226_ReadReg(INA_REG_CONFIG,     &regConfig);
  INA226_ReadReg(INA_REG_CALIB,      &regCalib);

  uint8_t data[8];
  data[0] = regShunt >> 8;
  data[1] = regShunt & 0xFF;
  data[2] = regBus >> 8;
  data[3] = regBus & 0xFF;
  data[4] = regConfig >> 8;
  data[5] = regConfig & 0xFF;
  data[6] = regCalib >> 8;
  data[7] = regCalib & 0xFF;

  txHeader.StdId = CAN_ID_REG_DIAGNOSTIC;
  txHeader.DLC   = 8;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1) {
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
  }
}

/* USER CODE END 0 */

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  setServoAngle(0, 90);
  setServoAngle(1, 90);
  setServoAngle(2, 90);
  setServoAngle(3, 90);

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    while (1) {}
  }

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  txHeader.ExtId = 0;
  txHeader.IDE   = CAN_ID_STD;
  txHeader.RTR   = CAN_RTR_DATA;
  txHeader.TransmitGlobalTime = DISABLE;

  HAL_Delay(CAN_STARTUP_DELAY);

  inaReady = INA226_Init();

  lastTelemetryTick = HAL_GetTick();
  lastHeartbeatTick = HAL_GetTick();
  lastInaReadTick   = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1) {
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if (canMsgReceived) {
      canMsgReceived = 0;
      processCANCommand();
    }

    if ((now - lastTelemetryTick) >= TELEMETRY_INTERVAL) {
      lastTelemetryTick = now;

      if (HAL_CAN_GetError(&hcan1) != HAL_CAN_ERROR_NONE) {
        HAL_CAN_Stop(&hcan1);
        HAL_Delay(10);
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
      } else if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 2) {
        sendServoFeedback();
      }
    }

    if ((now - lastHeartbeatTick) >= HEARTBEAT_INTERVAL) {
      lastHeartbeatTick = now;

      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1) {
        sendHeartbeat();
      }
    }

    if ((now - lastInaReadTick) >= INA226_READ_INTERVAL) {
      lastInaReadTick = now;

      if (inaReady) {
        INA226_Read();
      }
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1) {
        sendPowerTelemetry();
      }
      if (inaReady && HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 1) {
        sendShuntDiagnostic();
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

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
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
  hcan1.Init.AutoBusOff           = ENABLE;
  hcan1.Init.AutoWakeUp           = DISABLE;
  hcan1.Init.AutoRetransmission   = DISABLE;
  hcan1.Init.ReceiveFifoLocked    = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  CAN_FilterTypeDef f = {0};
  f.FilterBank          = 0;
  f.FilterMode          = CAN_FILTERMODE_IDMASK;
  f.FilterScale         = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh        = (0x200 << 5) & 0xFFFF;
  f.FilterIdLow         = 0x0000;
  f.FilterMaskIdHigh    = (0x7FC << 5) & 0xFFFF;
  f.FilterMaskIdLow     = 0x0000;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation    = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &f);

  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

static void MX_TIM3_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 83;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 19999;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = SERVO_CLOSED;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
