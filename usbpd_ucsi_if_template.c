/**
  ******************************************************************************
  * @file  : usbpd_ucsi_if_template.c
  * @brief : Source file for interfacing the usbpd_ucsi to hardware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "usbpd_ucsi.h"
#include "ppm.h"
#include "usbpd_ucsi_if_template.h"

#include "stm32g0xx.h"
#if defined(_RTOS)
#include "cmsis_os.h"
#endif /* _RTOS */
#if defined(_TRACE)
#include "usbpd_trace.h"
#include "string.h"
#include "stdio.h"
#endif /* _TRACE */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief UCSI states
  */
typedef enum
{
  UCSI_STATE_INIT,
  UCSI_STATE_WAIT_REGID,
  UCSI_STATE_CHANGE_FOR_READ,
  UCSI_STATE_READ_DATAEND,
  UCSI_STATE_NACK,
  UCSI_STATE_WRITE_RXCOUNT,

  /* Errors */
  UCSI_ERROR_SEQ_RECEIVE,
  UCSI_ERROR_SEQ_TRANSMIT,
  UCSI_ERROR_SEQ_UNKOWN,
  UCSI_ERROR_SEQ_TXCOUNT,
} UCSI_enum;

/**
  * @brief UCSI user commands
  */
typedef enum
{
  USER_CMD_RETRIEVE_FW_VERSION    = 0xFF00U,  /* MPU -> MCU: request MCU to send its FW version */
  USER_CMD_RETRIEVE_USBPD_VERSION = 0xFE01U,  /* MPU -> MCU: Get USBPD library version          */
  USER_CMD_GET_PARAMETERS_VERSION = 0xFD02U,  /* MPU -> MCU: Get parameters version             */
  USER_CMD_RESET_TO_BL            = 0xDE21U,  /* MPU -> MCU: Go to bootloader mode              */
  USER_CMD_UNKNOWN                = (USER_CMD_RESET_TO_BL + 1U)
} USER_REG;

/**
  * @brief UCSI interface handle
  */
typedef struct
{
  void (*ReceiveCommandCb)(uint8_t, uint8_t *);      /*!< function used to return the UCSI command received by OPM */
} UCSI_IF_HandleTypeDef;

/**
  * @brief UCSI user handle
  */
typedef struct
{
  UCSI_enum       Error;
  uint8_t         Direction;
  UCSI_REG_t      Register;
  USER_REG        RegisterUser;
  UCSI_enum       State;
  uint8_t        *PtrBuffer;
  uint8_t         DataSize;
} UCSI_USER_HandleTypeDef;

/* Private define ------------------------------------------------------------*/
/*****************************************************************************
   Alert PIN
  ****************************************************************************/
#define ALERT_GPIO_CLK_ENABLE   __HAL_RCC_GPIOB_CLK_ENABLE
#define ALERT_GPIO_PORT         STM32G0_INTN_GPIO_Port
#define ALERT_GPIO_PIN          STM32G0_INTN_Pin

/*****************************************************************************
   I2C configuration
  ****************************************************************************/
#define I2C_ADDRESS_USER                0xA2U  /* Use by the bootloader @0x51  */
#define I2C_ADDRESS_UCSI                0xA6U  /* Use for UCSI command  @0x53  */
#define I2C_ADDRESS_INVALID             0xFFU /* Invalid I2C address          */

/******************************************************************************/

#if defined(_TRACE)
/* Definition of debug levels */
#define UCSI_DEBUG_NONE     (0U)
#define UCSI_DEBUG_LEVEL_0  (1U << 0)
#define UCSI_DEBUG_LEVEL_1  (1U << 1)
#define UCSI_DEBUG_LEVEL_2  (1U << 2)
#endif /* _TRACE */


/* Private macro -------------------------------------------------------------*/
/* Wait and check the I2C operation complete */
#if defined(_RTOS)
#define CHECK_I2C_STATUS_LISTEN()                                \
  do                                                             \
  {                                                              \
    if (osSemaphoreWait(sem_i2c_cplt_id, osWaitForever) == osOK) \
    {                                                            \
      switch(HAL_I2C_GetError(&UCSI_HANDLE_I2C))                 \
      {                                                          \
        case HAL_I2C_ERROR_NONE:                                 \
        case HAL_I2C_ERROR_AF:                                   \
          break;                                                 \
        default :                                                \
          _retr = USBPD_FAIL;                                    \
          goto exit;                                             \
          break;                                                 \
      }                                                          \
    }                                                            \
    else                                                         \
    {                                                            \
      _retr = USBPD_FAIL;                                        \
      goto exit;                                                 \
    }                                                            \
  } while(0);

/* Free the mutex to indicate the operation complete */
#define OPERATION_COMPLETE()                                     \
  do                                                             \
  {                                                              \
    UCSI_HANDLE_I2C.State = HAL_I2C_STATE_READY;                 \
    __HAL_I2C_DISABLE_IT(&UCSI_HANDLE_I2C, (I2C_IT_ADDRI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI));\
    osSemaphoreRelease(sem_i2c_cplt_id);                         \
  } while(0);

#else
#define CHECK_I2C_STATUS_LISTEN()
#define OPERATION_COMPLETE()
#define LOCK_I2C_RESOURCE()          HAL_NVIC_DisableIRQ(ALERT_PORT0_EXTI_IRQn);
#define UNLOCK_I2C_RESOURCE()        HAL_NVIC_EnableIRQ(ALERT_PORT0_EXTI_IRQn);

#endif /* _RTOS */

/* Private variables ---------------------------------------------------------*/
#if defined(_TRACE)
/* UCSI debug verbose level */
uint8_t UCSI_DebugLevel = /*UCSI_DEBUG_LEVEL_2 | UCSI_DEBUG_LEVEL_1 | UCSI_DEBUG_LEVEL_0 | */ UCSI_DEBUG_NONE;
#endif /* _TRACE */

/* UCSI handle used for UCSI communication*/
static UCSI_USER_HandleTypeDef UCSI_Handles;
/* User handle use to allow internal communication with STM32MCU */
static UCSI_USER_HandleTypeDef User_Handles;
/* Keep the current I2C communication: either UCSI or User communication */
static uint8_t UCSI_USER_Communication = I2C_ADDRESS_INVALID;

/* Mutex definition */
#if defined(_RTOS)
osSemaphoreDef(sem_ucsi_i2c_cplt);
osSemaphoreId sem_i2c_cplt_id;
#endif /* _RTOS */

extern I2C_HandleTypeDef UCSI_HANDLE_I2C;
UCSI_IF_HandleTypeDef UCSI_IF_Handle;

/* Local variable to store port status */
uint8_t Register;
uint8_t I2C_Direction;
uint8_t *PtrUCSI_Data;

/**
  *   Save the FW version inside the FLASH to be read by the MPU in case of
  *   FW update.
  */
#if defined(__ICCARM__)
#pragma default_variable_attributes = @".VERSION"
#endif /* __ICCARM__ */

#define INFORMATION_SECTION_KEYWORD   (0xA56959A6U)

#if defined(__GNUC__)
uint32_t __attribute__((section(".VERSION"))) UCSI_FwVersion = USBPD_FW_VERSION;
uint32_t __attribute__((section(".VERSION"))) keyword = INFORMATION_SECTION_KEYWORD;
#else
const uint32_t UCSI_FwVersion = USBPD_FW_VERSION;
const uint32_t keyword = INFORMATION_SECTION_KEYWORD;
#endif /* __GNUC__ */

#if defined(__ICCARM__)
#pragma default_variable_attributes =
#endif /* __ICCARM__ */



#if defined(_TRACE)
/* Debug trace */
#define UCSI_IF_PPM_DEBUG_TRACE(_LEVEL_, ...)                                                               \
  do                                                                                                        \
  {                                                                                                         \
    if (UCSI_DebugLevel & (_LEVEL_))                                                                        \
    {                                                                                                       \
      char _str[70U];                                                                                       \
      (void)snprintf(_str, 70U, __VA_ARGS__);                                                               \
      USBPD_TRACE_Add(USBPD_TRACE_DEBUG, USBPD_PORT_0, 0U, (uint8_t*)_str, strlen(_str));                   \
    }                                                                                                       \
  } while(0);

/* Simple debug trace */
#define UCSI_IF_PPM_DEBUG_TRACE_NO_ARG(_LEVEL_, __MESSAGE__)                                                \
  do                                                                                                        \
  {                                                                                                         \
    if (UCSI_DebugLevel & (_LEVEL_))                                                                        \
    {                                                                                                       \
      USBPD_TRACE_Add(USBPD_TRACE_DEBUG, USBPD_PORT_0, 0U, (uint8_t*)(__MESSAGE__), sizeof(__MESSAGE__) - 1u);\
    }                                                                                                       \
  } while(0);

/* UCSI trace */
#define UCSI_IF_PPM_TRACE(_LEVEL_, _ADDR_, _ID_, _PTR_DATA_, _SIZE_)                                        \
  do                                                                                                        \
  {                                                                                                         \
    if (UCSI_DebugLevel & (_LEVEL_))                                                                        \
    {                                                                                                       \
      USBPD_TRACE_Add(USBPD_TRACE_UCSI, _ADDR_, _ID_, _PTR_DATA_, _SIZE_);                                  \
    }                                                                                                       \
  } while(0);
#else
#define UCSI_IF_PPM_DEBUG_TRACE(_LEVEL_, ...)
#define UCSI_IF_PPM_DEBUG_TRACE_NO_ARG(_LEVEL_, __MESSAGE__)
#define UCSI_IF_PPM_TRACE(_LEVEL_, _ADDR_, _ID_, _PTR_DATA_, _SIZE_)
#endif /* _TRACE */

/* Private function prototypes -----------------------------------------------*/
static USBPD_UCSI_Status_t  UCSI_I2C_Init(void (*ReceiveCommandCb)(uint8_t, uint8_t *));
static void                 UCSI_I2C_DeInit(void);
static void                 UCSI_AlertInit(void);
static void                 UCSI_AlertON(void);
static void                 UCSI_AlertOFF(void);
static USBPD_StatusTypeDef  USBPD_UCSI_PPM_ListenPort(uint8_t *pDirection, uint8_t *pRegister, uint8_t **ppData);
static void                 USBPD_UCSI_PPM_TaskListenI2C(void const *argument);
static uint8_t             *UCSI_GetDataSizeRegisterPtr(UCSI_REG_t Register, uint8_t *PtrDataSize);
static uint8_t             *UCSI_USER_GetDataSizeRegisterPtr(USER_REG Register, uint8_t *PtrDataSize);

/**
  * @brief  UCSI Driver definition.
  *
  * list all the driver interfaces used by the UCSI application.
  */
const USBPD_UCSI_Driver_t USBPD_UcsiDriver =
{
  UCSI_I2C_Init,
  UCSI_AlertON,
  UCSI_AlertOFF,
};

/**
  * @brief  Disable I2C used for UCSI
  * @param  None
  * @retval None
  */
static void UCSI_I2C_DeInit(void)
{
  I2C_HandleTypeDef *hi2c = &UCSI_HANDLE_I2C;
  if (HAL_OK != HAL_I2C_DeInit(hi2c))
  {
    /* Error to be managed */
    while (1) {};
  }
}

/**
  * @brief  Initialize UCSI
  * @param  *ReceiveCommandCb Pointer of reception function callback
  * @retval UCSI status
  */
static USBPD_UCSI_Status_t UCSI_I2C_Init(void (*ReceiveCommandCb)(uint8_t, uint8_t *))
{
  USBPD_UCSI_Status_t _retr = USBPD_UCSI_OK;

  MX_I2C_UCSI_HANDLE_INIT();
  /* Start the listen I2C task for UCSI*/
#if defined(_RTOS)
  if ((sem_i2c_cplt_id = osSemaphoreCreate(osSemaphore(sem_ucsi_i2c_cplt), 1U)) == 0)
  {
    /* Error occurred creating a Semaphore */
    _retr = USBPD_UCSI_ERROR;
  }
  /* Take the mutex_I2C_CPLT semaphores */
  osSemaphoreWait(sem_i2c_cplt_id, osWaitForever);
  osThreadDef(I2CTask, USBPD_UCSI_PPM_TaskListenI2C, osPriorityHigh, 0U, 120U);
  (void)osThreadCreate(osThread(I2CTask), NULL);
#else
#warning "NRTOS Version not implemented"
#endif /* _RTOS */

  UCSI_IF_Handle.ReceiveCommandCb = ReceiveCommandCb;

  /* Init Alert pin */
  UCSI_AlertInit();

  return _retr;
}

/**
  * @brief  ALERT ON
  * @param  None
  * @retval None
  */
static void UCSI_AlertON(void)
{
  /* Set alert signal */
  HAL_GPIO_WritePin(ALERT_GPIO_PORT, ALERT_GPIO_PIN, GPIO_PIN_RESET);
  UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_2, UCSI_USER_Communication, UCSI_TRACE_ALERT_ON, NULL, 0U);
}

/**
  * @brief  ALERT OFF
  * @param  None
  * @retval None
  */
static void UCSI_AlertOFF(void)
{
  /* Reset alert signal */
  HAL_GPIO_WritePin(ALERT_GPIO_PORT, ALERT_GPIO_PIN, GPIO_PIN_SET);
  UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_2, UCSI_USER_Communication, UCSI_TRACE_ALERT_OFF, NULL, 0U);
}

uint32_t _error = 0U;
static void USBPD_UCSI_PPM_TaskListenI2C(void const *argument)
{
  while (1)
  {
    if (USBPD_OK != USBPD_UCSI_PPM_ListenPort(&I2C_Direction, &Register, &PtrUCSI_Data))
    {
      /* Increment error counter */
      _error++;
    }
    else
    {
      if (UCSI_USER_Communication == I2C_ADDRESS_UCSI)
      {
        if (I2C_DIRECTION_TRANSMIT == I2C_Direction)
        {
          UCSI_IF_Handle.ReceiveCommandCb(Register, PtrUCSI_Data);
        }
      }
      UCSI_USER_Communication = I2C_ADDRESS_INVALID;
    }
  }
}

/**
  * @brief  UCSI Get the register id
  * @param  *pDirection
  * @param  *pRegister
  * @param  **ppData
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_UCSI_PPM_ListenPort(uint8_t *pDirection, uint8_t *pRegister, uint8_t **ppData)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
  UCSI_USER_HandleTypeDef *ptr_port = NULL;
  /* Put I2C in Listen */
  if (HAL_OK != HAL_I2C_EnableListen_IT(&UCSI_HANDLE_I2C))
  {
    /* Increment error counter */
    _error++;
  }

  CHECK_I2C_STATUS_LISTEN();

  if (UCSI_USER_Communication == I2C_ADDRESS_UCSI)
  {
    ptr_port = &UCSI_Handles;
  }
  else
  {
    ptr_port = &User_Handles;
  }

  if (ptr_port->Error == UCSI_ERROR_SEQ_TRANSMIT)
  {
    /* Error occurred during transmission, reinitialize I2C */
    UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_LISTEN_PORT, NULL, 0U);
    _retr = USBPD_ERROR;
    UCSI_I2C_DeInit();
    MX_I2C_UCSI_HANDLE_INIT();
    ptr_port->Error = UCSI_STATE_INIT;
  }
  else
  {
    /* No error occurred during transmission */
    *pDirection    = ptr_port->Direction;
    *pRegister     = ptr_port->Register;
    *ppData        = ptr_port->PtrBuffer;
  }

exit:
  /* Reset UCSI handles except buffer and error */
  if (ptr_port != NULL)
  {
    ptr_port->Direction     = 0U;
    ptr_port->Register      = UcsiRegDataStructureUnkown;
    ptr_port->State         = UCSI_STATE_INIT;
  }
  return _retr;
}

/*************************************************************************/
/* I2C callback implementation                                           */
/*************************************************************************/
/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
#if defined(_TRACE)
  if (UCSI_USER_Communication == I2C_ADDRESS_USER)
  {
    UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_ERROR_CPLT, NULL, 0U);
  }
  else
  {
    UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_ERROR_CPLT,
                      (uint8_t *)&UCSI_Handles.Register, 1U);
  }
#endif /* _TRACE */
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  OPERATION_COMPLETE();
#if defined(_TRACE)
  if (UCSI_USER_Communication == I2C_ADDRESS_USER)
  {
    UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_LISTEN_CPLT, NULL, 0U);
  }
  else
  {
    if (UCSI_Handles.Register < UcsiRegDataStructureUnkown)
    {
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_LISTEN_CPLT,
                        (uint8_t *)&UCSI_Handles.Register, 1U);
    }
    else
    {
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_LISTEN_CPLT, NULL, 0U);
    }
  }
#endif /* _TRACE */
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  UCSI_USER_Communication = AddrMatchCode;
  /* If command is sent at @ I2C_ADDRESS_USER, it means that MPU requested to switch in bootloader */
  UCSI_USER_HandleTypeDef *ptr_port;
  uint8_t *ptr_register;
  uint8_t size_reg;
  if (AddrMatchCode == I2C_ADDRESS_USER)
  {
    ptr_port = &User_Handles;
    ptr_register = (uint8_t *)&ptr_port->RegisterUser;
    size_reg = 2U;
  }
  else
  {
    ptr_port = &UCSI_Handles;
    ptr_register = (uint8_t *)&ptr_port->Register;
    size_reg = 1U;
  }
  ptr_port->Direction = TransferDirection;
  if (I2C_DIRECTION_TRANSMIT ==  TransferDirection)
  {
    UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_ADDRCB_WAIT_REGID,
                      ptr_register, size_reg);
    ptr_port->State = UCSI_STATE_WAIT_REGID;
    /* Register information is expected */
    if (HAL_OK != HAL_I2C_Slave_Sequential_Receive_IT(hi2c, ptr_register, size_reg, I2C_FIRST_FRAME))
    {
      /* Error detected */
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_ADDRCB_ERROR_RX, NULL, 0U);
      ptr_port->Error = UCSI_ERROR_SEQ_RECEIVE;
      OPERATION_COMPLETE();
    }
  }
  else
  {
    /* Get information about how many data to send */
    if (ptr_port->PtrBuffer != NULL)
    {
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_ADDRCB_CHANGE_FOR_READ,
                        ptr_register, size_reg);
      ptr_port->State = UCSI_STATE_CHANGE_FOR_READ;
      /* get data pointer */
      if (HAL_OK != HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, ptr_port->PtrBuffer,
                                                         ptr_port->DataSize, I2C_LAST_FRAME))
      {
        /* Error detected */
        UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_ADDRCB_ERROR_TX, NULL, 0U);
        ptr_port->Error = UCSI_ERROR_SEQ_TRANSMIT;
        OPERATION_COMPLETE();
      }
    }
    else
    {
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_ADDRCB_ERROR_TX, NULL, 0U);
      /* Error detected */
      ptr_port->Error = UCSI_ERROR_SEQ_TRANSMIT;
    }
  }
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  UCSI_USER_HandleTypeDef *ptr_port;
  /* communication based on UCSI communication*/
  if (UCSI_USER_Communication == I2C_ADDRESS_USER)
  {
    ptr_port = &User_Handles;
  }
  else
  {
    ptr_port = &UCSI_Handles;
  }

  /* The RX is complete */
  switch (ptr_port->State)
  {
    case UCSI_STATE_WAIT_REGID :
    {
      if (UCSI_USER_Communication == I2C_ADDRESS_USER)
      {
        /* Get information about how many data to send */
        if ((ptr_port->PtrBuffer = UCSI_USER_GetDataSizeRegisterPtr(ptr_port->RegisterUser,
                                                                    &ptr_port->DataSize)) != NULL)
        {
          UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_RX_CPLT_WAIT_REGID,
                            (uint8_t *)&ptr_port->RegisterUser, 2U);
          ptr_port->State = UCSI_STATE_READ_DATAEND;
          /* get data pointer  */
          if (HAL_OK != HAL_I2C_Slave_Sequential_Receive_IT(hi2c, ptr_port->PtrBuffer,
                                                            ptr_port->DataSize, I2C_LAST_FRAME))
          {
            /* Error detected */
            ptr_port->Error = UCSI_ERROR_SEQ_TRANSMIT;
          }
        }
        else
        {
          /* NACK the unknown request */
          UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_RX_CPLT_NACK,
                            (uint8_t *)&ptr_port->RegisterUser, 2U);
          /* Error detected */
          ptr_port->Error = UCSI_ERROR_SEQ_TRANSMIT;
          OPERATION_COMPLETE();
        }
      }
      else
      {
        __HAL_I2C_ENABLE_IT(&UCSI_HANDLE_I2C, I2C_IT_TXI);
        /* Get information about how many data to send */
        if ((ptr_port->PtrBuffer = UCSI_GetDataSizeRegisterPtr(ptr_port->Register,
                                                               &ptr_port->DataSize)) != NULL)
        {
          UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_RX_CPLT_WAIT_REGID,
                            (uint8_t *)&ptr_port->Register, 1);
          ptr_port->State = UCSI_STATE_READ_DATAEND;
          /* get data pointer  */
          if (HAL_OK != HAL_I2C_Slave_Sequential_Receive_IT(hi2c, ptr_port->PtrBuffer, ptr_port->DataSize,
                                                            I2C_LAST_FRAME))
          {
            /* Error detected */
            ptr_port->Error = UCSI_ERROR_SEQ_TRANSMIT;
          }
        }
        else
        {
          /* NACK the unknown request */
          UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_RX_CPLT_NACK,
                            (uint8_t *)&ptr_port->Register, 1U);
          /* Error detected */
          ptr_port->Error = UCSI_ERROR_SEQ_TRANSMIT;
          OPERATION_COMPLETE();
        }
      }
    }
    break;

    case UCSI_STATE_READ_DATAEND :
      if (UCSI_USER_Communication == I2C_ADDRESS_USER)
      {
        /* Do nothing */
      }
      else
      {
        UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_RX_CPLT_DATAEND,
                          (uint8_t *)&ptr_port->Register, 1U);
        /* Communication closed  */
        ptr_port->State = UCSI_STATE_INIT;
        OPERATION_COMPLETE();
      }
      break;
    case UCSI_STATE_INIT:
    case UCSI_STATE_CHANGE_FOR_READ:
    case UCSI_STATE_NACK:
    case UCSI_STATE_WRITE_RXCOUNT:
    case UCSI_ERROR_SEQ_RECEIVE:
    case UCSI_ERROR_SEQ_TRANSMIT:
    case UCSI_ERROR_SEQ_UNKOWN:
    case UCSI_ERROR_SEQ_TXCOUNT:
    default:
      break;
  }
}

/**
  * @brief  Slave Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  UCSI_USER_HandleTypeDef *ptr_port;
  if (UCSI_USER_Communication == I2C_ADDRESS_USER)
  {
    ptr_port = &User_Handles;
    UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_TX_CPLT_COMMAND,
                      (uint8_t *)&ptr_port->RegisterUser, 2U);
  }
  else
  {
    ptr_port = &UCSI_Handles;
    if (ptr_port->Register < UcsiRegDataStructureUnkown)
    {
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_TX_CPLT_COMMAND,
                        (uint8_t *)&ptr_port->Register, 1U);
    }
    else
    {
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_1, UCSI_USER_Communication, UCSI_TRACE_TX_CPLT_ERROR,
                        (uint8_t *)&ptr_port->Register, 1U);
      OPERATION_COMPLETE();
    }
  }

  ptr_port->State = UCSI_STATE_INIT;
}

/**
  * @brief  Initialize UCSI Alert
  * @param  None
  * @retval None
  */
static void UCSI_AlertInit(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable ALERT GPIO clock */
  ALERT_GPIO_CLK_ENABLE();

  /* Init Alert GPIO */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = ALERT_GPIO_PIN;
  /* Write in output before initialized GPIO (avoid spike to OFF)*/
  HAL_GPIO_WritePin(ALERT_GPIO_PORT, ALERT_GPIO_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(ALERT_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Return the datasize/pointer @ for a register used only for read operation
  * @param  Register    UCSI command
  * @param  PtrDataSize Pointer on the size of the UCSI command
  * @retval Pointer on the register
  */
static uint8_t *UCSI_GetDataSizeRegisterPtr(UCSI_REG_t Register, uint8_t *PtrDataSize)
{
  switch (Register)
  {
    case UcsiRegDataStructureVersion:
      *PtrDataSize = sizeof(UCSI_VERSION_t);
      break;
    case UcsiRegDataStructureCci:
      *PtrDataSize = sizeof(UCSI_CCI_t);
      break;
    case UcsiRegDataStructureControl:
      *PtrDataSize = sizeof(UCSI_CONTROL_t);
      break;
    case UcsiRegDataStructureMessageIn:
      *PtrDataSize = sizeof(UCSI_MESSAGE_IN_t);
      break;
    case UcsiRegDataStructureMessageOut:
      *PtrDataSize = sizeof(UCSI_MESSAGE_OUT_t);
      break;
    case UcsiRegDataStructureUnkown:
    case UcsiRegDataStructureReserved:
    default:
      /* Should not happen */
      *PtrDataSize = 0U;
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_UCSI_GET, NULL, 0U);
      break;
  }
  return USBPD_PPM_GetDataPointer(Register);
}

/**
  * @brief  Return the datasize/pointer @ for a register used for BL mode
  * @param  Register  UCSI command
  * @param  PtrDataSize   Pointer on the size of the UCSI command
  * @retval Pointer on the register
  */
static uint8_t *UCSI_USER_GetDataSizeRegisterPtr(USER_REG Register, uint8_t *PtrDataSize)
{
  uint8_t *ptr = NULL;
  switch (Register)
  {
    /* Retrieve the firmware version */
    case USER_CMD_RETRIEVE_FW_VERSION:
      ptr = (uint8_t *)&UCSI_FwVersion;
      *PtrDataSize = 4U;
      break;

    /* MPU -> MCU: request MCU to reset in BL mode    */
    case USER_CMD_RESET_TO_BL:
    {
      /* Switch to BL */
      FLASH_OBProgramInitTypeDef pOBInit;

      HAL_FLASH_Unlock();
      HAL_FLASH_OB_Unlock();

      /* Set and program option bytes */
      pOBInit.OptionType = OPTIONBYTE_USER;
      pOBInit.USERType = OB_USER_nBOOT_SEL | OB_USER_nBOOT1 | OB_USER_nBOOT0;
      pOBInit.USERConfig = OB_BOOT0_FROM_OB | OB_BOOT1_SYSTEM | OB_nBOOT0_RESET;
      HAL_FLASHEx_OBProgram(&pOBInit);
      HAL_FLASH_OB_Launch();
      break;
    }

    /* Unknown command */
    case USER_CMD_UNKNOWN:
    default:
      /* Should not happen */
      *PtrDataSize = 0U;
      UCSI_IF_PPM_TRACE(UCSI_DEBUG_LEVEL_0, UCSI_USER_Communication, UCSI_TRACE_USER_GET, NULL, 0U);
      break;
  }
  return ptr;
}
