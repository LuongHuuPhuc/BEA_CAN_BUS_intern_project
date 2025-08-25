/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "stdarg.h"
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef CAN1_pHeader;
CAN_RxHeaderTypeDef CAN1_pHeaderRx;
CAN_FilterTypeDef CAN1_sFilterConfig;
CAN_TxHeaderTypeDef CAN2_pHeader;
CAN_RxHeaderTypeDef CAN2_pHeaderRx;
CAN_FilterTypeDef CAN2_sFilterConfig;

uint32_t CAN1_pTxMailbox;
uint32_t CAN2_pTxMailbox;

uint8_t CAN1_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN1_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Node1_MsgCounter = 0;
uint8_t Node2_MsgCounter = 0;
uint8_t ForceWrongCRC = 0; // Flag to force wrong CRC on 0x012
uint32_t LastNode1TxTime;
uint32_t LastNode2TxTime;
uint32_t LastNode2RxTime;

uint8_t __attribute__((unused))uart3_receive;
uint16_t __attribute__((unused))NumBytesReq = 0;
uint8_t  __attribute__((unused))REQ_BUFFER[4096];
uint8_t  __attribute__((unused))REQ_1BYTE_DATA;
uint16_t __attribute__((unused))Num_Consecutive_Tester;
uint8_t  __attribute__((unused))Flg_Consecutive = 0;

unsigned int TimeStamp;
char bufsend[30] = "XXX: D1 D2 D3 D4 D5 D6 D7 D8  "; //Toi da 30
char uart_buf[1024];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void MX_CAN1_Setup(void);
void MX_CAN2_Setup(void);
void USART3_SendString(uint8_t *ch);
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame);

void __attribute__((unused))SID_22_Practice(void);
void __attribute__((unused))SID_2E_Practice(void);
void __attribute__((unused))SID_27_Practice(void);

void Data_callback(uint8_t *rx, uint8_t *tx, uint8_t source);
void ProcessNode1(void);
void ProcessNode2(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin); //Ham callback de xu ly ngat tu PA0
uint8_t CalculateCRC8(const uint8_t *data, uint8_t len);
void uart_printf(const char *fmt,...);
void UART_Receive_from_IT(void);

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

  SystemCoreClockUpdate();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  UART_Receive_from_IT();
//  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

  MX_CAN1_Setup();
  MX_CAN2_Setup();

  HAL_Delay(100); // ?????
  uart_printf("SYSCLK: %lu\r\n", HAL_RCC_GetSysClockFreq()); //debug dung o day va ko doc duoc HAL_GetTicḳ̣̣̣̣̣() => Loi Clock
  uart_printf("HCLK: %lu\r\n", HAL_RCC_GetHCLKFreq());


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Example Function to print can message via uart

  uint32_t LastNode1TxTime = HAL_GetTick();
  uint32_t LastNode2TxTime = HAL_GetTick();
  uint32_t LastNode2RxTime = HAL_GetTick();


  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint32_t currentTime = HAL_GetTick();
	sprintf(uart_buf, "Tick: %lu\r\n", currentTime);
	uart_printf(uart_buf);

	// Node 2 TX (0x0A2) moi 50ms (Node 2 gui truoc)
	if(currentTime - LastNode2TxTime >= NODE2_TX_CYCLE){
		ProcessNode2();
		LastNode2TxTime = currentTime;
	}

	//Node 1 TX (0x012) moi 4s
	if(currentTime - LastNode1TxTime >= NODE1_TX_CYCLE){
		ProcessNode1();
		LastNode1TxTime = currentTime;
	}

	//Kiem tra neu Node 2 khong nhan 0x012 trong thoi gian dai hoac sai CRC
	if(currentTime - LastNode2RxTime > NODE1_TX_CYCLE + 100){
		memset(CAN2_DATA_TX, 0x00, 6); //Set bytes 0-5 thanh 0x00
		CAN2_DATA_TX[6] = Node2_MsgCounter++;
		CAN2_DATA_TX[7] = CalculateCRC8(CAN2_DATA_TX, 7);
		HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox);
		PrintCANLog(CAN_ID_NODE2_TX, CAN2_DATA_TX);

		if(Node2_MsgCounter > 0xF) Node2_MsgCounter = 0; //Reset bo dem neu dem den 15
	}
  }

//    if(!BtnU) /*IG OFF->ON stimulation*/
//    {
//      delay(20);
//      USART3_SendString((uint8_t *)"IG OFF ");
//      while(!BtnU);
//      MX_CAN1_Setup();
//      MX_CAN2_Setup();
//      USART3_SendString((uint8_t *)"-> IG ON \n");
//      delay(20);
//    }
//  }
//
//  memset(&REQ_BUFFER,0x00,4096);
//  NumBytesReq = 0;

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
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC4 PC5 PC6
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//=============================================================================================================

void UART_Receive_from_IT(void){
	HAL_UART_Receive_IT(&huart3, &REQ_1BYTE_DATA, 1); //Bat dau nhan 1 byte
}


void MX_CAN1_Setup(void){
	CAN1_pHeader.StdId = CAN_ID_NODE1_TX; //Set ID tx Node1
	CAN1_pHeader.IDE = CAN_ID_STD;
	CAN1_pHeader.RTR = CAN_RTR_DATA;
	CAN1_pHeader.DLC = CAN_DLC;

	CAN1_sFilterConfig.FilterBank = 0;
	CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN1_sFilterConfig.FilterIdHigh = CAN_ID_NODE1_RX << 5; // 0x0A2 << 5
    CAN1_sFilterConfig.FilterIdLow = 0x0000;
    CAN1_sFilterConfig.FilterMaskIdHigh = 0xFFFF;
    CAN1_sFilterConfig.FilterMaskIdLow = 0x0000;
    CAN1_sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN1_sFilterConfig.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig) != HAL_OK){
		Error_Handler();
	}

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void MX_CAN2_Setup(void){
	CAN2_pHeader.StdId = CAN_ID_NODE2_TX;
	CAN2_pHeader.IDE = CAN_ID_STD;
	CAN2_pHeader.RTR = CAN_RTR_DATA;
	CAN2_pHeader.DLC = CAN_DLC;

	CAN2_sFilterConfig.FilterBank = 14;
	CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN2_sFilterConfig.FilterIdHigh = CAN_ID_NODE2_RX << 5; // 0x012 << 5
	CAN2_sFilterConfig.FilterIdLow = 0x0000;
	CAN2_sFilterConfig.FilterMaskIdHigh = 0xFFFF;
	CAN2_sFilterConfig.FilterMaskIdLow = 0x0000;
	CAN2_sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN2_sFilterConfig.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig) != HAL_OK){
		Error_Handler();
	}
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void __attribute__((unused))USART3_SendString(uint8_t *ch)
{
   while(*ch != 0)
   {
      if(HAL_UART_Transmit(&huart3, ch, 1, 1000) != HAL_OK){
    	  Error_Handler();
      }
      ch++;
   }
}

void uart_printf(const char *fmt,...){
	va_list args;
	va_start(args, fmt);
	vsnprintf(uart_buf, sizeof(uart_buf), fmt, args);
	va_end(args);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
}


void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame){
    char bufID[4] = "    ";
    char bufDat[3] = "   ";
    char bufTime[9] = "         ";

	sprintf(bufTime, "%lu", HAL_GetTick());
	uart_printf(bufTime);
	uart_printf(" ");

	sprintf(bufID, "%03X", CANID);
	for(uint8_t i = 0; i < 3; i ++)
	{
		bufsend[i] = bufID[i];
	}
	bufsend[3] = ':';
	bufsend[4] = ' ';


	for(uint16_t i = 0; i < 8; i++)
	{
		sprintf(bufDat, "%02X", CAN_Frame[i]);
		bufsend[i * 3 + 5] = bufDat[0];
		bufsend[i * 3 + 6] = bufDat[1];
		bufsend[i * 3 + 7] = ' ';
	}
	bufsend[29] = '\n';
	uart_printf(bufsend);
}


uint8_t CalculateCRC8(const uint8_t *data, uint8_t len){
  uint8_t crc = 0xFF; // Initial value for CRC-8 SAE J1850
  for (uint8_t i = 0; i < len; i++)
  {
	crc ^= data[i];
	for (uint8_t j = 0; j < 8; j++)
	{
	  if (crc & 0x80)
		crc = (crc << 1) ^ 0x1D; // Polynomial x^8 + x^4 + x^3 + x^2 + 1
	  else
		crc <<= 1;
	}
  }
  return crc ^ 0xFF;
}


void ProcessNode1(void){
	CAN1_DATA_TX[0] = CAN1_DATA_RX[0]; //Copy byte 0 from RX
	CAN1_DATA_TX[1] = CAN1_DATA_RX[1]; //Copy byte 1 from RX
	CAN1_DATA_TX[2] = CAN1_DATA_TX[0] + CAN1_DATA_TX[1]; //Tong 2 byte 0 va 1
	CAN1_DATA_TX[3] = 0x00;
	CAN1_DATA_TX[4] = 0x00;
	CAN1_DATA_TX[5] = 0x00;
	CAN1_DATA_TX[6] = Node1_MsgCounter++;
	CAN1_DATA_TX[7] = ForceWrongCRC ? 0x00 : CalculateCRC8(CAN1_DATA_TX, 7); //Wrong CRC if flag set
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox); //Gui tu Node 1
	PrintCANLog(CAN_ID_NODE1_TX, CAN1_DATA_TX);

	if(Node1_MsgCounter > 0xF) Node1_MsgCounter = 0;
}


void ProcessNode2(void){
	// Vi du 2 gia tri value1 va value2 cua Node2
	CAN2_DATA_TX[0] = 0x10; //Value1
	CAN2_DATA_TX[1] = 0x20; //Value2
	CAN2_DATA_TX[2] = CAN2_DATA_TX[0] + CAN2_DATA_TX[1];
	CAN2_DATA_TX[3] = 0x00;
	CAN2_DATA_TX[4] = 0x00;
	CAN2_DATA_TX[5] = 0x00;
	CAN2_DATA_TX[6] = Node2_MsgCounter++;
	CAN2_DATA_TX[7] = CalculateCRC8(CAN2_DATA_TX, 7);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox); //Gui tu Node 2
	PrintCANLog(CAN_ID_NODE2_TX, CAN2_DATA_TX);

	if(Node2_MsgCounter > 0xF) Node2_MsgCounter = 0; //Neu counter den den 15 (0xF)
}


//Xu ly ngat khi co message CAN nhan duoc trong FIFO0, luu du lieu de xu ly tiep
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(hcan == &hcan1){ //Node 1 nhan tu Node 2
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX) == HAL_OK){ //Doc FIFO tu Node 1
			uint8_t crc = CalculateCRC8(CAN1_DATA_RX, 7);

			if(CAN1_DATA_RX[7] != crc) return; //Kiem tra checksum

			TimeStamp = HAL_GetTick();
			PrintCANLog(CAN_ID_NODE1_RX, CAN1_DATA_RX);
			Data_callback(CAN1_DATA_RX, CAN1_DATA_TX, 0); //Node 1 gui phan hoi voi ID 0x012
		}
	}

	else if(hcan == &hcan2){ //Node 2 nhan tu Node 1
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_pHeaderRx, CAN2_DATA_RX) == HAL_OK){ //Doc FIFO tu Node 2
			uint8_t crc = CalculateCRC8(CAN2_DATA_RX, 7);

			if(crc != CAN2_DATA_RX[7]){
				//CRC sai, dat byte 0-5 thanh 0x00 cho 0x0A2 ke tiep
				memset(CAN2_DATA_TX, 0x00, 6);
				return;
			}

			TimeStamp = HAL_GetTick();
			PrintCANLog(CAN_ID_NODE2_RX, CAN2_DATA_RX);
			//Node 2 sau do chi nhan, khong phan hoi
		}
	}
}


//source = 0 (Node 1 gui den Node 2), source = 1 (Node 2 gui den Node 1)
void Data_callback(uint8_t *rx, uint8_t *tx, uint8_t source){
	tx[0] = rx[0]; //Sao chep Value1
	tx[1] = rx[1]; //Sao chep Value2
	tx[2] = rx[1] + rx[0];
	tx[3] = 0x00;
	tx[4] = 0x00;
	tx[5] = 0x00;
	tx[6] = (rx[6] + 1) % 0x10; //Tang message counter
	tx[7] = CalculateCRC8(tx, 7);

	TimeStamp = HAL_GetTick();

	if(source == 0){ //Node 1 gui phan hoi den Node 2
		HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, tx, &CAN1_pTxMailbox); //Su dung CAN1 voi ID 0x012
		PrintCANLog(CAN1_pHeader.StdId, tx); //In ID 0x012

	}else{ //Node 2 gui phan hoi den Node 1
		HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, tx, &CAN2_pTxMailbox); //Su dung CAN2 voi ID 0x0A2
		PrintCANLog(CAN2_pHeader.StdId, tx); //In ID 0x0A2
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		ForceWrongCRC = 1; //Force wrong CRC cho 0x012 ke tiep
		uart_printf("Forced wrong CRC for 0x012\r\n");
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		REQ_BUFFER[NumBytesReq] = REQ_1BYTE_DATA;
		NumBytesReq++;
		if(NumBytesReq == 2){
			CAN1_DATA_TX[0] = REQ_BUFFER[0]; //Value1
			CAN1_DATA_TX[1] = REQ_BUFFER[1]; //Value2
			CAN1_DATA_TX[2] = 0x00;
			CAN1_DATA_TX[3] = 0x00;
			CAN1_DATA_TX[4] = 0x00;
			CAN1_DATA_TX[5] = 0x00;
			CAN1_DATA_TX[6] = 0x00; //Counter
			CAN1_DATA_TX[7] = CalculateCRC8(CAN1_DATA_TX, 7);//checksum
			HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox);
			PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
			NumBytesReq = 0; //Reset buffer
		}
		HAL_UART_Receive_IT(&huart3, &REQ_1BYTE_DATA, 1); //Tiep tuc nhan 1 byte
	}
}


static void __attribute__((unused))delay(uint16_t delay){
	HAL_Delay(delay);
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
  uart_printf("uart error !\r\n");
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
