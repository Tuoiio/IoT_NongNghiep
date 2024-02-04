#include <stdio.h>
#include <string.h>
#include "main.h"
#include <delay.h>
#include <LCD20X4.h>
#include <DS1307.h>
#include <DHT22.h>
#include <Flash.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
extern DS1307_HandleTypeDef DS1307;

char *Days[] =  {"SUN","MON","TUE","WED","THU","FRI","SAT"};

uint8_t Status_Menu_Setting = 0,Status_Mode_Time = 0, Status_Mode_Warning = 0, Status_Mode_Auto = 0, Status_Mode_Manual = 0;

uint8_t Status_Mode_Lamp = 0, Status_Mode_Pump = 0, Status_Mode_Fan = 0;

int8_t Old_Display_LCD_Main = 0,  Status_Mode_Pump_On_Off = 0, Display_Main = 0;

int8_t Old_Display_Lamp, Display_Lamp;

_Bool   LCD_Main_Or_Setting, LCD_Menu_Setting, LCD_Set_Auto, LCD_Heating_Lamp, LCD_Pump, LCD_Pump_On_Off, LCD_Set_Fan;

_Bool LCD_Set_Time, LCD_Set_Warning, LCD_Set_Relay, LCD_Set_Pump_TIM_1, LCD_Set_Pump_TIM_2, Clock_1Hz , Prioritized;

_Bool b_Auto_Set_Reset_Pump_TIM_1, b_Auto_Set_Reset_Pump_TIM_2, b_LCD_Manual, Mode_Auto_Or_Manual;

_Bool b_Manual_Set_Reset_Heating_Lamp, b_Manual_Set_Reset_Pump, b_Manual_Set_Reset_Fan;

_Bool b_Warning_Low_Humidity, b_Warning_High_Humidity, b_Warning_Low_Temperature, b_Warning_High_Temperature, b_Status_DHT;

_Bool b_Auto_Morning_Set_Reset_Lamp,b_Auto_Evening_Set_Reset_Lamp;

_Bool b_Auto_Set_Reset_Fan, b_Status_Lamp, b_Status_Fan , b_Status_Pump , b_Morning_Evening;

uint16_t Buffer_ADC, DoAmAo, DoAmThat;

float f_Nhiet_Do, f_Do_Am_Khong_Khi;

uint32_t lastDelay, last_Delay_1Hz;

char Rx_Data[60] = { 0 };
char Tx_Data[20] = { 0 };


struct warning{
	int8_t High_Temperature;
	int8_t Low_Temperature;
	int8_t High_Humidity;
	int8_t Low_Humidity;
} Warning;



struct on_off{
	int8_t Turn_On;
	int8_t Turn_Off;
} Relay_Fan;


#pragma pack(1)
struct Relay_Lamp{
	struct on_off On_Off;
	int8_t Hour;
	int8_t Minute;
}Relay_Lamp_Morning, Relay_Lamp_Evening;
#pragma pack()

#pragma pack(1)
struct Relay_Pump{
	int8_t Minute ;
	int8_t Hour;
	int8_t Soil_Moisture;
	int8_t Status_Mode;
	_Bool On_Off;
}Relay_Pump_TIM_1, Relay_Pump_TIM_2, *Relay_Pump;
#pragma  pack()

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

void DieuKhien(void);
void KeyBoard(void);
void Hien_Thi_LCD(void);
void Set_Para_Clock(void);
void Set_Para_Warning(void);
void Set_Para_Lamp(void);
void Set_Para_Pump(void);
void Set_Para_Fan(void);
void Save_Data_Waring_Flash(void);
void Save_Data_Lamp_Flash(void);
void Save_Data_Pump_Flash(void);
void Save_Data_Fan_Flash(void);
void Read_Data_Save_Flash(void);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)Rx_Data,60);
}



void Bip(void){
	uint8_t i;
	for(i = 0; i < 200; i++){
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		Delay_us(200);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		Delay_us(200);
	}
}
int main(void){
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	HAL_TIM_Base_Start(&htim4);
	LCD20X4_Init();
	DS1307_Init();
	Read_Data_Save_Flash();
	LCD20X4_Gotoxy(3,0);
	LCD20X4_PutString("IoT Nong Nghiep");
	LCD20X4_Gotoxy(0,1);
	LCD20X4_PutString("He Thong Giam Sat Tu");
	LCD20X4_Gotoxy(0,2);
	LCD20X4_PutString("Xa Va Dieu Khien Tu");
	LCD20X4_Gotoxy(0,3);
	LCD20X4_PutString("Dong Trong Nha Kinh");
	Delay_ms(5000);
	LCD20X4_Clear();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)Rx_Data, 60);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	lastDelay = HAL_GetTick();
	last_Delay_1Hz = HAL_GetTick();
	while (1)
  { 
	// 1.5s doc nhiet do, do am va do am dat 1 lan
	if(HAL_GetTick() - lastDelay > 1000){
		// Doc tin hieu analog do dam dat
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&Buffer_ADC, 1);
		DoAmAo = (Buffer_ADC * 100) / 4095;
		DoAmThat = 100 - DoAmAo;
		// Doc tin hieu 1-wire nhiet do va do am khong khi
		b_Status_DHT = DHT22_Start();
		if(b_Status_DHT){
			DHT22_Read();
			f_Nhiet_Do = DHT22_Temperature();
			f_Do_Am_Khong_Khi = DHT22_Humidity();
		} 
		// Truyen Data sang ESP01
		sprintf(Tx_Data, "%2.1f %2.1f %d ", f_Nhiet_Do, f_Do_Am_Khong_Khi, DoAmThat);
		HAL_UART_Transmit(&huart2, (uint8_t*)Tx_Data, strlen(Tx_Data), 100);
		
		sprintf(Tx_Data, "%d %d %d ", b_Status_Lamp, b_Status_Fan, b_Status_Pump);
		HAL_UART_Transmit(&huart2, (uint8_t*)Tx_Data, strlen(Tx_Data), 100);
		
		sprintf(Tx_Data, "%d %d %d %d", b_Warning_Low_Temperature, b_Warning_High_Temperature, b_Warning_Low_Humidity, b_Warning_High_Humidity);
		HAL_UART_Transmit(&huart2, (uint8_t*)Tx_Data, strlen(Tx_Data), 100);
		lastDelay = HAL_GetTick();
	}
	// Tao xung 1hz
	if(HAL_GetTick() - last_Delay_1Hz > 500){
		if(Status_Mode_Time == 0) DS1307_Read_Time();
		Clock_1Hz = !Clock_1Hz;
		last_Delay_1Hz = HAL_GetTick();
	}
	  KeyBoard();
 	  Hien_Thi_LCD();
		DieuKhien();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | Relay_4_Pin | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Relay_3_Pin | Relay_2_Pin | Relay_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DHT_Pin_Pin Relay_4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin Buzzer_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | Relay_4_Pin | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_Mode_Pin Button_Ok_Pin Button_Up_Pin Button_Down_Pin */
  GPIO_InitStruct.Pin = Button_Mode_Pin | Button_Ok_Pin | Button_Up_Pin | Button_Down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_EN_Pin LCD_Led_Pin LCD_D4_Pin Relay_3_Pin  Relay_2_Pin  Relay_1_Pin*/
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | Relay_3_Pin | Relay_2_Pin | Relay_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void KeyBoard(void){
	 static _Bool Old_Mode = 0, Old_Ok = 0, Old_Up = 0, Old_Down = 0;
	 static uint8_t Time_Break_Mode = 0, Time_Break_Up = 0, Time_Break_Down = 0, Time_Wait_Open_Display_Manual = 0;
	/*******************************************Chon che do ************************************/
	if(Button_Mode == 0){
		Delay_ms(10);
		Time_Break_Mode++;
		if(Time_Break_Mode > 70 && Time_Break_Mode < 72){
			Bip();
			LCD20X4_Clear();
			LCD_Main_Or_Setting = LCD_Menu_Setting = 1;
			Status_Menu_Setting = 1;
			Time_Break_Mode = 1; // tranh truong hop giu qua lau lam tran gia tri cua bien
		}
		if(!Button_Mode && Old_Mode){
			Bip();
			if(b_LCD_Manual){
				Status_Mode_Manual++;
				if(Status_Mode_Manual > 3) Status_Mode_Manual = 0;
			}
			// Chon trang thai cai dat thoi gian
			if(LCD_Set_Time){
				Status_Mode_Time++;
				if(Status_Mode_Time > 7) Status_Mode_Time = 1;   // status_Mode_Time = 0; thi doc tin hieu tu DS1307
			}
			// Chon trang thai cai dat canh bao
			if(LCD_Set_Warning){
				Status_Mode_Warning++;
				if(Status_Mode_Warning > 4) Status_Mode_Warning = 0;
			}
			// Chon trang thai cai dat nhiet do den suoi
			if(LCD_Heating_Lamp){
				Status_Mode_Lamp++;
				if(Status_Mode_Lamp > 9) Status_Mode_Lamp = 1;
			}
			// Chon trang thai cai dat do am cho bom
			if(LCD_Pump){
				Status_Mode_Pump_On_Off ++;
				if(Status_Mode_Pump_On_Off > 4) Status_Mode_Pump_On_Off = 1;
			}
			if(LCD_Set_Pump_TIM_1 || LCD_Set_Pump_TIM_2){
				Relay_Pump->Status_Mode++;
				if(Relay_Pump->Status_Mode > 4) Relay_Pump->Status_Mode = 1;
			}
			if(LCD_Set_Fan){
				Status_Mode_Fan++;
				if(Status_Mode_Fan > 3) Status_Mode_Fan = 1;
			}
		}
	} else {
		Time_Break_Mode = 0;
	}
	Old_Mode = Button_Mode;
	
	/*******************************************Nut bam tang gia tri ************************************/
	if(Button_Up == 0){
		Delay_ms(20);
		// Nhan giu lien tuc trong 1 thoi gian thi gia tri tang lien tuc
		Time_Break_Up++;
		if(Time_Break_Up > 17){
			Time_Break_Up = 12;
			Old_Up = 1;
		}
		// Nhan nha luon thi gia tri tang 1 don vi
		if(!Button_Up && Old_Up){
			Clock_1Hz = 1;
			if(Time_Break_Up == 1) Bip();		
			if(LCD_Main_Or_Setting == 0){
				if(b_LCD_Manual){
					switch(Status_Mode_Manual){			// muc 0 thiet bi se duoc bat
						case 0: { b_Manual_Set_Reset_Heating_Lamp = 1; 	break; } 
						case 1: { b_Manual_Set_Reset_Pump = 1;				break; }
						case 2: { b_Manual_Set_Reset_Fan = 1;				break; }
						case 3: {															break; }
						default:{															break; }
					}
				} else {
					Display_Main--;
					if(Display_Main < 0) Display_Main = 7;
				}
			}
			if(LCD_Main_Or_Setting == 1){
				// Chon che do de cai dat
				if(LCD_Menu_Setting){
					Status_Menu_Setting++;
					if(Status_Menu_Setting > 4) Status_Menu_Setting = 1;
				}
				// Chon che do cai dat thoi gian
				if(LCD_Set_Time){
					switch(Status_Mode_Time){
						case 1: { DS1307.Minute++;		break; }
						case 2: { DS1307.Hour++; 		break; }
						case 3: { DS1307.Day++; 		break; }
						case 4: { DS1307.Date++;		break; }
						case 5: { DS1307.Month++;		break; }
						case 6: { DS1307.Year++; 		break; }
						case 7: {           			    	break; }
						default:{			    				break; }
					}
				}
				// Chon che do cai dat canh bao
				if(LCD_Set_Warning){
					switch(Status_Mode_Warning){
						case 0: { Warning.Low_Temperature++;  break; }
						case 1: { Warning.High_Temperature++; break; }
						case 2: { Warning.Low_Humidity++;		break; }
						case 3: { Warning.High_Humidity++;	 	break; }
						case 4 : {										  	break; }
						default:{                                        	break; }
					}
				}
				// Chon che do cai dat Auto
				if(LCD_Set_Auto){
					Status_Mode_Auto++;
					if(Status_Mode_Auto > 4) Status_Mode_Auto = 1;
				}
				// Cai dat nhiet do bat den suoi
				if(LCD_Heating_Lamp){
					switch(Status_Mode_Lamp){
						case 1: { Relay_Lamp_Morning.Hour++; 				 break; }
						case 2: { Relay_Lamp_Morning.Minute++;  			 break; }
						case 3: { Relay_Lamp_Evening.Hour++;				 break; }
						case 4: { Relay_Lamp_Evening.Minute++;			 break; }
						case 5: { Relay_Lamp_Morning.On_Off.Turn_On++; break; }
						case 6: { Relay_Lamp_Morning.On_Off.Turn_Off++; break; }
						case 7: { Relay_Lamp_Evening.On_Off.Turn_On++;  break; }
						case 8: { Relay_Lamp_Evening.On_Off.Turn_Off++;	 break; }
						case 9: { 														 break; }
						default: { 												  	 break; }
					}
				}
				if(LCD_Pump){
					switch(Status_Mode_Pump_On_Off){
						case 1: { Relay_Pump_TIM_1.On_Off = 1; break; }
						case 2: { Relay_Pump_TIM_2.On_Off = 1; break; }
						case 3: { 											break; }
						case 4: { 											break; }
						default:{ 											break; }
					}
				}
				if(LCD_Set_Pump_TIM_1 || LCD_Set_Pump_TIM_2){
					switch(Relay_Pump->Status_Mode){
						case 1: { Relay_Pump->Hour++; 				break; }
						case 2: { Relay_Pump->Minute++; 			break; }
						case 3: { Relay_Pump->Soil_Moisture++; 	break; }
						case 4: { 												break; }
						default: { 											break; }
					}
				}
				// Cai dat nhiet do quat
				if(LCD_Set_Fan){
					switch(Status_Mode_Fan){
						case 1: { Relay_Fan.Turn_On++;  	break; }
						case 2: { Relay_Fan.Turn_Off++; 	break; }
						case 3 : {									break; }
						default:{                                  break; }
					}
				}
			}
		}
	} else {
		Time_Break_Up = 0;
	}
	Old_Up = Button_Up;
	
	
	/*******************************************Nut bam giam gia tri ************************************/
	if(Button_Down == 0){
		Delay_ms(20);
		// Nhan giu lien tuc trong 1 thoi gian thi gia tri tang lien tuc
		Time_Break_Down++;
		if(Time_Break_Down > 17){
			Time_Break_Down = 12;
			Old_Down = 1;
		}
		// Nhan nha luon thi gia tri giam 1 don vi
		if(!Button_Down && Old_Down){
			Clock_1Hz = 1;
			if(Time_Break_Down == 1) Bip();
			// Chon che do cai dat
			if(LCD_Main_Or_Setting == 0){
				if(b_LCD_Manual){
					switch(Status_Mode_Manual){			// muc 1 thiet bi se tat
						case 0: { b_Manual_Set_Reset_Heating_Lamp = 0; break; }
						case 1: { b_Manual_Set_Reset_Pump = 0;			break; }
						case 2: { b_Manual_Set_Reset_Fan = 0;				break; }
						case 3: {														break; }
						default:{														break; }
					}
				} else {
					Display_Main++;
					if(Display_Main > 7) Display_Main = 0;
				}
			}
			if(LCD_Main_Or_Setting == 1){
				if(LCD_Menu_Setting){
					Status_Menu_Setting--;
					if(Status_Menu_Setting == 0) Status_Menu_Setting = 4;
				}
				// Chon che do cai dat thoi gian
				if(LCD_Set_Time){
					switch(Status_Mode_Time){
						case 1: { DS1307.Minute--;	break; }
						case 2: { DS1307.Hour--;	 	break; }
						case 3: { DS1307.Day--; 		break; }
						case 4: { DS1307.Date--;		break; }
						case 5: { DS1307.Month--;	break; }
						case 6: { DS1307.Year--;	 	break; }
						case 7: {                 			break; }
						default:{							break; }
					}
				}
				// Chon che do cai dat canh bao
				if(LCD_Set_Warning){
					switch(Status_Mode_Warning){
						case 0: { Warning.Low_Temperature--; 	break; }
						case 1: { Warning.High_Temperature--; 	break; }
						case 2: { Warning.Low_Humidity--; 	  	break; }
						case 3: { Warning.High_Humidity--;	  	break; }
						case 4 : {										  	break; }
						default:{                                        	break; }
					}
				}
				// Chon che do cai dat tu dong
				if(LCD_Set_Auto){
					Status_Mode_Auto--;
					if(Status_Mode_Auto == 0) Status_Mode_Auto = 4;
				}
				// Cai dat giam nhiet do den suoi
				if(LCD_Heating_Lamp){
					switch(Status_Mode_Lamp){
						case 1: { Relay_Lamp_Morning.Hour--; 					break; }
						case 2: { Relay_Lamp_Morning.Minute--;  				break; }
						case 3: { Relay_Lamp_Evening.Hour--;				 		break; }
						case 4: { Relay_Lamp_Evening.Minute--;					break; }
						case 5: { Relay_Lamp_Morning.On_Off.Turn_On--; 		break; }
						case 6: { Relay_Lamp_Morning.On_Off.Turn_Off--;  	break; }
						case 7: { Relay_Lamp_Evening.On_Off.Turn_On--; 		break; }
						case 8: { Relay_Lamp_Evening.On_Off.Turn_Off--;		break; }
						case 9: { 															break; }
						default: { 												  		break; }
					}
				}
				// Tat hoat dong thoi gian 1 hoac 2
				if(LCD_Pump){
					switch(Status_Mode_Pump_On_Off){
						case 1: { Relay_Pump_TIM_1.On_Off = 0; break; }
						case 2: { Relay_Pump_TIM_2.On_Off = 0; break; }
						case 3: { 											break; }
						case 4: { 											break; }
						default:{ 											break; }
					}
				}
				// Giam gia tri cai dat bom 
				if(LCD_Set_Pump_TIM_1 || LCD_Set_Pump_TIM_2){
					switch(Relay_Pump->Status_Mode){
						case 1: { Relay_Pump->Hour--; 			break; }
						case 2: { Relay_Pump->Minute--; 			break; }
						case 3: { Relay_Pump->Soil_Moisture--; 	break; }
						case 4: {											break; }
						default: { 										break; }
					}
				}
				if(LCD_Set_Fan){
					switch(Status_Mode_Fan){
						case 1: { Relay_Fan.Turn_On--;  break; }
						case 2: { Relay_Fan.Turn_Off--; 	 break; }
						case 3 : {								 break; }
						default:{                               break; }
					}
				}
			}				
		}
	} else {
		Time_Break_Down = 0;
	}
	Old_Down = Button_Down;
	
	
	/*******************************************Nut bam hoan thanh ************************************/
	if(Button_Ok == 0){
		Delay_ms(10);
		Time_Wait_Open_Display_Manual++;
		if(Time_Wait_Open_Display_Manual > 70 && Time_Wait_Open_Display_Manual < 72){
			Time_Wait_Open_Display_Manual = 1;  // tranh truong hop giu qua lau lam tran gia tri cua bien
			if(LCD_Main_Or_Setting == 0){
				Bip();
				b_LCD_Manual = 1;
				Mode_Auto_Or_Manual = 1;
				LCD20X4_Clear();
				HAL_GPIO_WritePin(GPIOA, Relay_1_Pin | Relay_2_Pin | Relay_3_Pin, GPIO_PIN_SET);
				b_Manual_Set_Reset_Fan = b_Manual_Set_Reset_Heating_Lamp = b_Manual_Set_Reset_Pump = 0;
				b_Status_Fan = b_Status_Lamp = b_Status_Pump = 0;
			}
		}
		if(!Button_Ok && Old_Ok){
			Bip();	
			if(b_LCD_Manual){
				if(Status_Mode_Manual == 3){
					b_LCD_Manual = 0;
					Mode_Auto_Or_Manual = 0;
					Status_Mode_Manual = 0;
					LCD20X4_Clear();
					HAL_GPIO_WritePin(GPIOA, Relay_1_Pin | Relay_2_Pin | Relay_3_Pin, GPIO_PIN_SET);
					b_Status_Fan = b_Status_Lamp = b_Status_Pump = 0;
				}
			}
			if(LCD_Menu_Setting){
				if(Status_Menu_Setting == 1){  LCD_Set_Time = 1; Status_Mode_Time = 1; LCD_Set_Warning = 0; LCD_Set_Auto = 0; DS1307.Second = 0; }
				if(Status_Menu_Setting == 2){  LCD_Set_Time = 0; Status_Mode_Pump_On_Off = 1; LCD_Set_Warning = 1; LCD_Set_Auto = 0; }
				if(Status_Menu_Setting == 3){  LCD_Set_Time = 0; LCD_Set_Warning = 0;  LCD_Set_Auto = 1; Status_Mode_Auto = 0; } 
				if(Status_Menu_Setting == 4){  LCD_Main_Or_Setting = 0; Status_Menu_Setting = 0; }
				LCD_Menu_Setting = 0;
				LCD20X4_Clear();
			}
			// Cai dat xong thoi gian
			if(LCD_Set_Time){
				if(Status_Mode_Time == 7){
						Status_Mode_Time = LCD_Set_Time = 0;
						LCD_Menu_Setting = 1;
						LCD20X4_Clear();
						DS1307_Write_Time();
				}
			}
			// Cai dat xong canh bao
			if(LCD_Set_Warning){
				if(Status_Mode_Warning == 4){
					LCD_Set_Warning =  Status_Mode_Warning = 0;
					LCD_Menu_Setting = 1;
					Save_Data_Waring_Flash();
					LCD20X4_Clear();
				}
				// Con thieu ghi luu tru gia tri vao DS1307
			}
			// Chon xong che do cai dat tu dong
			if(LCD_Set_Auto){
				if(Status_Mode_Auto == 1){	LCD_Heating_Lamp = Status_Mode_Lamp = 1; LCD_Set_Auto = 0; Old_Display_Lamp = Display_Lamp = 0; }
				if(Status_Mode_Auto == 2){	LCD_Pump = Status_Mode_Pump_On_Off = 1; LCD_Set_Auto = 0; }
				if(Status_Mode_Auto == 3){	LCD_Set_Fan = Status_Mode_Fan = 1; LCD_Set_Auto = 0; }
				if(Status_Mode_Auto == 4){
					LCD_Main_Or_Setting = 0;
					LCD_Set_Auto = 0;
					Status_Mode_Auto = 1;
				}
				//LCD_Set_Auto = 0;	// neu status_mode_auto = 1,2,3 thi sau khi cai dat xong cac thong so thi quay  lai bat LCD_Set_Auto = 1;
				LCD20X4_Clear();
			}
			//Cai dat xong den suoi
			if(LCD_Heating_Lamp){
				if(Status_Mode_Lamp == 9){
					LCD_Heating_Lamp = 0;
					Status_Mode_Lamp = 0;
					LCD_Set_Auto = 1;
					Save_Data_Lamp_Flash();
					LCD20X4_Clear();
				}
			}
			//Cai dat xong bom nuoc
			if(LCD_Pump){
				if(Status_Mode_Pump_On_Off == 3){
					 LCD_Set_Pump_TIM_1 = Relay_Pump_TIM_1.Status_Mode = 1; 
					LCD_Set_Pump_TIM_2 = Relay_Pump_TIM_2.Status_Mode = 1; 
					Prioritized = 0;
					LCD_Pump = 0;
					LCD20X4_Clear();
				}
				if(Status_Mode_Pump_On_Off == 4){
					Save_Data_Pump_Flash();
					Status_Mode_Pump_On_Off = 0;
					LCD_Pump = 0;
					LCD20X4_Clear();
					LCD_Set_Auto = 1;
				}
			}
			//Cai dat xong bom nuoc thoi gian 1
			if(Relay_Pump_TIM_1.Status_Mode == 4){
				LCD_Set_Pump_TIM_1 = 0;
				LCD20X4_Clear();
				Prioritized = 1;
				Relay_Pump_TIM_1.Status_Mode = 0;
				Relay_Pump_TIM_2.Status_Mode = 1;
				LCD_Set_Pump_TIM_2 = 1;

			}
			// Cai dat xong bom nuoc thoi gian 2
			if(Relay_Pump_TIM_2.Status_Mode == 4){
				LCD_Set_Pump_TIM_2 = 0;
				Relay_Pump_TIM_2.Status_Mode = 0;
				LCD20X4_Clear();
				LCD_Pump = 1;
			}
			// Cai dat xong thong so quat
			if(LCD_Set_Fan){
				if(Status_Mode_Fan == 3){
					LCD_Set_Fan =  Status_Mode_Fan = 0;
					LCD20X4_Clear();
					Save_Data_Fan_Flash();
					LCD_Set_Auto = 1;
				}
			}
		}
	}
	Old_Ok = Button_Ok;
	
	if(LCD_Set_Pump_TIM_1 && !Prioritized) Relay_Pump = &Relay_Pump_TIM_1;
	if(LCD_Set_Pump_TIM_2 && Prioritized) Relay_Pump = &Relay_Pump_TIM_2;
	
	if(Status_Mode_Time != 0)								Set_Para_Clock();
	if(LCD_Set_Warning) 										Set_Para_Warning();
	if(LCD_Heating_Lamp){
		Set_Para_Lamp();
		switch(Status_Mode_Lamp){
		case 1: case 2: case 3: case 4: { Display_Lamp = 0;	break; }
		case 5: case 6: 					 { Display_Lamp = 1;	break; }
		case 7: case 8: case 9: 			 { Display_Lamp = 2;	break; }
		default: 								 {								break; }
	}
	if(Old_Display_Lamp != Display_Lamp) LCD20X4_Clear();
	Old_Display_Lamp = Display_Lamp;
}
	if(LCD_Set_Pump_TIM_1 || LCD_Set_Pump_TIM_2) Set_Para_Pump();
	if(LCD_Set_Fan)												Set_Para_Fan();
}

void Set_Para_Clock(void){
	unsigned char y;
	y = (DS1307.Year % 4)* 100;
	// Tang
	if(DS1307.Minute > 59) DS1307.Minute = 0;
	if(DS1307.Hour > 23) 	  DS1307.Hour = 0;
	if(DS1307.Day > 7) 	  DS1307.Day = 1;
	if((DS1307.Month == 2) && (DS1307.Date > 29) && (y == 0)) DS1307.Date = 1;
	else if((DS1307.Month == 2) && (DS1307.Date > 29) && (y != 0)) DS1307.Date = 1;
	else if(((DS1307.Month == 4) || (DS1307.Month == 6) || (DS1307.Month == 9) || (DS1307.Month == 11)) && DS1307.Date > 30 && (y != 0))	DS1307.Date = 1;
	else if(DS1307.Date > 31) DS1307.Date = 1;
	if(DS1307.Month > 12) DS1307.Month = 1;
	
	// Giam
	if(DS1307.Minute < 0) DS1307.Minute = 59;
	if(DS1307.Hour < 0) 	  DS1307.Hour = 23;
	if(DS1307.Day < 1) 	  DS1307.Day = 7;
	if((DS1307.Month == 2) && (DS1307.Date < 1) && (y == 0)) DS1307.Date = 29;
	else if((DS1307.Month == 2) && (DS1307.Date > 29) && (y != 0)) DS1307.Date = 1;
	else if(((DS1307.Month == 4) || (DS1307.Month == 6) || (DS1307.Month == 9) || (DS1307.Month == 11)) && (DS1307.Date < 1) && (y != 0))	DS1307.Date = 30;
	else if(DS1307.Date < 1) DS1307.Date = 31;
	if(DS1307.Month < 1) DS1307.Month = 12;
}

void Set_Para_Warning(void){
	if(Warning.Low_Humidity >= 99) Warning.Low_Humidity = 0;
	if(Warning.Low_Humidity < 0) Warning.Low_Humidity = 99;
	if(Warning.High_Humidity > 99) Warning.High_Humidity = 0;
	if(Warning.High_Humidity < 0) Warning.High_Humidity = 99;
	if(Warning.Low_Temperature >= 99) Warning.Low_Temperature = 0;
	if(Warning.Low_Temperature < 0) Warning.Low_Temperature = 99;
	if(Warning.High_Temperature > 99) Warning.High_Temperature = 0;
	if(Warning.High_Temperature < 0) Warning.High_Temperature = 99;
}

void Set_Para_Lamp(void){
	// Hour_AM
	if(Relay_Lamp_Morning.Hour < 0) Relay_Lamp_Morning.Hour = 23;
	if(Relay_Lamp_Morning.Hour > 23) Relay_Lamp_Morning.Hour = 0;
	//Minute_AM
	if(Relay_Lamp_Morning.Minute < 0) Relay_Lamp_Morning.Minute = 59;
	if(Relay_Lamp_Morning.Minute > 59) Relay_Lamp_Morning.Minute = 0;
	// Hour_PM
	if(Relay_Lamp_Evening.Hour < 0) Relay_Lamp_Evening.Hour = 23;
	if(Relay_Lamp_Evening.Hour > 23) Relay_Lamp_Evening.Hour = 0;
	//Minute_PM
	if(Relay_Lamp_Evening.Minute < 0) Relay_Lamp_Evening.Minute = 59;
	if(Relay_Lamp_Evening.Minute > 59) Relay_Lamp_Evening.Minute = 0;
	//Turn_On_morning
	if(Relay_Lamp_Morning.On_Off.Turn_On < 0) Relay_Lamp_Morning.On_Off.Turn_On = 99;
	if(Relay_Lamp_Morning.On_Off.Turn_On >= 99) Relay_Lamp_Morning.On_Off.Turn_On = 0;
	//Turn off morning
	if(Relay_Lamp_Morning.On_Off.Turn_Off < 0) Relay_Lamp_Morning.On_Off.Turn_Off = 99;
	if(Relay_Lamp_Morning.On_Off.Turn_Off > 99) Relay_Lamp_Morning.On_Off.Turn_Off = 0;
	//turn on evening
	if(Relay_Lamp_Evening.On_Off.Turn_On < 0) Relay_Lamp_Evening.On_Off.Turn_On = 99;
	if(Relay_Lamp_Evening.On_Off.Turn_On > 99) Relay_Lamp_Evening.On_Off.Turn_On = 0;
	//turn off evening
	if(Relay_Lamp_Evening.On_Off.Turn_Off < 0) Relay_Lamp_Evening.On_Off.Turn_Off = 99;
	if(Relay_Lamp_Evening.On_Off.Turn_Off > 99) Relay_Lamp_Evening.On_Off.Turn_Off = 0;
}

void Set_Para_Pump(void){
	if(Relay_Pump->Hour > 23) 				Relay_Pump->Hour = 0;
	if(Relay_Pump->Hour < 0)   				Relay_Pump->Hour = 23;
	if(Relay_Pump->Minute > 59)				Relay_Pump->Minute = 0;
	if(Relay_Pump->Minute < 0)   			Relay_Pump->Minute = 59;
	if(Relay_Pump->Soil_Moisture > 100) 	Relay_Pump->Soil_Moisture = 0;
	if(Relay_Pump->Soil_Moisture < 0)    	Relay_Pump->Soil_Moisture = 100;
}

void Set_Para_Fan(void){
	if(Relay_Fan.Turn_On > 99) Relay_Fan.Turn_On = 0;
	if(Relay_Fan.Turn_On < 0) Relay_Fan.Turn_On = 99;
	if(Relay_Fan.Turn_Off >= 99) Relay_Fan.Turn_Off = 0;
	if(Relay_Fan.Turn_Off < 0) Relay_Fan.Turn_Off = 99;
	
	if(Relay_Fan.Turn_Off > Relay_Fan.Turn_On) Relay_Fan.Turn_On = Relay_Fan.Turn_Off + 1;
}

void Save_Data_Waring_Flash(void){
	int8_t Save_Data_Warning[4];
	Save_Data_Warning[0] = Warning.Low_Humidity;
	Save_Data_Warning[1] = Warning.High_Humidity;
	Save_Data_Warning[2] = Warning.Low_Temperature;
	Save_Data_Warning[3] = Warning.High_Temperature;
	Flash_Erase(_PAGE_60_);
	Flash_Write_Array(_PAGE_60_ + 2, (uint8_t*)Save_Data_Warning, 4);
}

void Save_Data_Lamp_Flash(void){
	int8_t Save_Data_Lamp[8];
	Save_Data_Lamp[0] = Relay_Lamp_Morning.Hour;
	Save_Data_Lamp[1] = Relay_Lamp_Morning.Minute;
	Save_Data_Lamp[2] = Relay_Lamp_Morning.On_Off.Turn_On;
	Save_Data_Lamp[3] = Relay_Lamp_Morning.On_Off.Turn_Off;
	Save_Data_Lamp[4] = Relay_Lamp_Evening.Hour;
	Save_Data_Lamp[5] = Relay_Lamp_Evening.Minute;
	Save_Data_Lamp[6] = Relay_Lamp_Evening.On_Off.Turn_On;
	Save_Data_Lamp[7] = Relay_Lamp_Evening.On_Off.Turn_Off;
	Flash_Erase(_PAGE_61_);
	Flash_Write_Array(_PAGE_61_ + 2, (uint8_t*)Save_Data_Lamp, 8);
}

void Save_Data_Pump_Flash(void){
	int8_t Save_Data_Pump[8];
	Save_Data_Pump[0] = Relay_Pump_TIM_1.On_Off;
	Save_Data_Pump[1] = Relay_Pump_TIM_1.Hour;
	Save_Data_Pump[2] = Relay_Pump_TIM_1.Minute;
	Save_Data_Pump[3] =Relay_Pump_TIM_1.Soil_Moisture;
	Save_Data_Pump[4] = Relay_Pump_TIM_2.On_Off;
	Save_Data_Pump[5] = Relay_Pump_TIM_2.Hour;
	Save_Data_Pump[6] = Relay_Pump_TIM_2.Minute;
	Save_Data_Pump[7] =Relay_Pump_TIM_2.Soil_Moisture;
	Flash_Erase(_PAGE_62_);
	Flash_Write_Array(_PAGE_62_ + 2, (uint8_t*)Save_Data_Pump, 8);
}

void Save_Data_Fan_Flash(void){
	int8_t Save_Data_Fan[2];
	Save_Data_Fan[0] = Relay_Fan.Turn_On;
	Save_Data_Fan[1] = Relay_Fan.Turn_Off;
	Flash_Erase(_PAGE_63_);
	Flash_Write_Array(_PAGE_63_ + 2, (uint8_t*)Save_Data_Fan, 2);
}

void Read_Data_Save_Flash(void){
	// Read data setting warning 
	int8_t Read_Data_Warning[4];
	Flash_Read_Array(_PAGE_60_ + 2, (uint8_t *)Read_Data_Warning, 4);
	Warning.Low_Humidity =  	  Read_Data_Warning[0];
	Warning.High_Humidity =  	  Read_Data_Warning[1];
	Warning.Low_Temperature =   Read_Data_Warning[2];
	Warning.High_Temperature =  Read_Data_Warning[3];
	
	//Read data heating lamp 
	int8_t Read_Data_Lamp[8];
	Flash_Read_Array(_PAGE_61_ + 2, (uint8_t *)Read_Data_Lamp, 8);
	Relay_Lamp_Morning.Hour = 				Read_Data_Lamp[0];
	Relay_Lamp_Morning.Minute =  			Read_Data_Lamp[1];
	Relay_Lamp_Morning.On_Off.Turn_On =  Read_Data_Lamp[2];
	Relay_Lamp_Morning.On_Off.Turn_Off =  Read_Data_Lamp[3];
	Relay_Lamp_Evening.Hour =  				 Read_Data_Lamp[4];
	Relay_Lamp_Evening.Minute =  				 Read_Data_Lamp[5];
	Relay_Lamp_Evening.On_Off.Turn_On =    Read_Data_Lamp[6];
	Relay_Lamp_Evening.On_Off.Turn_Off =    Read_Data_Lamp[7];
	
	// Read data pump 
	int8_t Read_Data_Pump[8];
	Flash_Read_Array(_PAGE_62_ + 2, (uint8_t *)Read_Data_Pump, 8);
	Relay_Pump_TIM_1.On_Off = 		 Read_Data_Pump[0];
	Relay_Pump_TIM_1.Hour = 			 Read_Data_Pump[1];
	Relay_Pump_TIM_1.Minute = 		 Read_Data_Pump[2];
	Relay_Pump_TIM_1.Soil_Moisture = Read_Data_Pump[3];
	Relay_Pump_TIM_2.On_Off = 		 Read_Data_Pump[4];
	Relay_Pump_TIM_2.Hour = 			 Read_Data_Pump[5];
	Relay_Pump_TIM_2.Minute = 		 Read_Data_Pump[6];
	Relay_Pump_TIM_2.Soil_Moisture = Read_Data_Pump[7];
	
	// Read data fan
	int8_t Read_Data_Fan[2];
	Flash_Read_Array(_PAGE_63_ + 2, (uint8_t *)Read_Data_Fan, 2);
	Relay_Fan.Turn_On = Read_Data_Fan[0];
	Relay_Fan.Turn_Off = Read_Data_Fan[1];
}

void Hien_Thi_LCD(void){
	
/****************************************************************HIen thi du lieu len man hinh************************************************/
	if(LCD_Main_Or_Setting == 0){
		if(Old_Display_LCD_Main != Display_Main) LCD20X4_Clear();
		Old_Display_LCD_Main = Display_Main;
		if(b_LCD_Manual){
			if(Clock_1Hz){
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Mode Manual");
				LCD20X4_Gotoxy(15,0);
				LCD20X4_PutString("EXIT");
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("Den Suoi: ");
				if(b_Manual_Set_Reset_Heating_Lamp) LCD20X4_PutString("ON ");
				else												 LCD20X4_PutString("OFF");
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Bom Nuoc: ");
				if(b_Manual_Set_Reset_Pump) LCD20X4_PutString("ON ");
				else									LCD20X4_PutString("OFF");
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("Quat Gio: ");
				if(b_Manual_Set_Reset_Fan) LCD20X4_PutString("ON ");
				else									LCD20X4_PutString("OFF");
			} else {
				switch(Status_Mode_Manual){
					case 0: { LCD20X4_Gotoxy(10,1); LCD20X4_PutString("   "); break; }
					case 1: { LCD20X4_Gotoxy(10,2); LCD20X4_PutString("   "); break; }
					case 2: { LCD20X4_Gotoxy(10,3); LCD20X4_PutString("   "); break; }
					case 3: {	LCD20X4_Gotoxy(15,0); LCD20X4_PutString("    "); break; }
					default: {																   break; }
				}
			}
		} else {
			if(Display_Main == 0){
				// Line 0
				LCD20X4_Gotoxy(4,0);
				LCD20X4_PutString("Do Am Dat: ");
				LCD20X4_SendInteger(DoAmThat);
				LCD20X4_PutString("%   ");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				if(b_Status_DHT){
					LCD20X4_PutString("T:");
					LCD20X4_SendFloat(f_Nhiet_Do);
					if(f_Nhiet_Do >= 0.0 && f_Nhiet_Do <= 10.0)	LCD20X4_Gotoxy(5,1);
					LCD20X4_PutChar(0xDF);
					LCD20X4_PutString("C     ");		
					LCD20X4_Gotoxy(12,1);
					LCD20X4_PutString("H:");
					LCD20X4_SendFloat(f_Do_Am_Khong_Khi);
					if(f_Do_Am_Khong_Khi >= 0.0 && f_Do_Am_Khong_Khi <= 10.0)	LCD20X4_Gotoxy(15,1);
					LCD20X4_PutString("% ");
				} else {
					LCD20X4_PutString("Not Connect DHT     ");
				}
				// Line 2
				LCD20X4_Gotoxy(6,2);
				LCD20X4_PutChar(DS1307.Hour / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Hour % 10 | 0x30);
				LCD20X4_PutChar(':');
				LCD20X4_PutChar(DS1307.Minute / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Minute % 10 | 0x30);
				LCD20X4_PutChar(':');
				LCD20X4_PutChar(DS1307.Second / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Second % 10 | 0x30);
				// Line 3
				LCD20X4_Gotoxy(2,3);
				LCD20X4_PutString(Days[DS1307.Day-1]);
				LCD20X4_PutString("  ");
				LCD20X4_PutChar(DS1307.Date / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Date % 10 | 0x30);
				LCD20X4_PutChar('/');
				LCD20X4_PutChar(DS1307.Month / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Month % 10 | 0x30);
				LCD20X4_PutString("/20");
				LCD20X4_PutChar(DS1307.Year / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Year % 10 | 0x30);
				LCD20X4_PutString("   ");
			}
			if(Display_Main == 1){
				// Hien thi luc chua ket noi duoc
				if(strlen(Rx_Data) == 0){
					LCD20X4_Gotoxy(0,0);
					LCD20X4_PutString("Wifi Disconnected");
				}
				// Hien thi luc ket noi duoc
				for(uint8_t i = 0; i < strlen(Rx_Data); i++){
					// Line 0
					if(i == 0) LCD20X4_Gotoxy(0,0);
					if(i < 18) LCD20X4_PutChar(Rx_Data[i]);
					// Line 1
					if(i == 18) LCD20X4_Gotoxy(0,1);
					if(i >= 18 && i < 34)  LCD20X4_PutChar(Rx_Data[i]);
					// Line 2
					if(i == 34) LCD20X4_Gotoxy(0,2);
					if(i >= 34 && i < 43) LCD20X4_PutChar(Rx_Data[i]);
					// Line 3
					if(i == 43) LCD20X4_Gotoxy(0,3);
					if(strlen(Rx_Data) == 56)	if(i >= 43 && i <= 56) LCD20X4_PutChar(Rx_Data[i]);
					if(strlen(Rx_Data) == 57)	if(i >= 43 && i <= 57) LCD20X4_PutChar(Rx_Data[i]);
					if(strlen(Rx_Data) == 58)	if(i >= 43 && i <= 58) LCD20X4_PutChar(Rx_Data[i]);
				}
			}
			if(Display_Main == 2){
				//				//Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Thong so canh bao");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("L_Temp:");
				LCD20X4_SendInteger(Warning.Low_Temperature);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("H_Temp:");
				LCD20X4_SendInteger(Warning.High_Temperature);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
				// Line 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("L_Humi:");
				LCD20X4_SendInteger(Warning.Low_Humidity);
				LCD20X4_PutString("% ");
				LCD20X4_Gotoxy(10,3);
				LCD20X4_PutString("H_Humi:");
				LCD20X4_SendInteger(Warning.High_Humidity);
				LCD20X4_PutString("% ");
		}
		if(Display_Main == 3){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Thong so cai dat den");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("Ban Ngay   ");
				LCD20X4_SendInteger(Relay_Lamp_Morning.Hour);
				LCD20X4_PutChar(':');
				LCD20X4_SendInteger(Relay_Lamp_Morning.Minute);
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Nhiet do bat:");
				LCD20X4_SendInteger(Relay_Lamp_Morning.On_Off.Turn_On);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
				// Line 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("Nhiet do tat: ");
				LCD20X4_SendInteger(Relay_Lamp_Morning.On_Off.Turn_Off);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
			}
			if(Display_Main == 4){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Thong so cai dat den");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("Ban Dem   ");
				LCD20X4_SendInteger(Relay_Lamp_Evening.Hour);
				LCD20X4_PutChar(':');
				LCD20X4_SendInteger(Relay_Lamp_Evening.Minute);
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Nhiet do bat:");
				LCD20X4_SendInteger(Relay_Lamp_Evening.On_Off.Turn_On);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
				// Line 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("Nhiet do tat: ");
				LCD20X4_SendInteger(Relay_Lamp_Evening.On_Off.Turn_Off);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
			}
			if(Display_Main == 5){
				// Line 1
					LCD20X4_Gotoxy(0,0);
					LCD20X4_PutString("Thong so quat gio");
					LCD20X4_Gotoxy(0,1);
					LCD20X4_PutString("Do am bat: ");
					LCD20X4_SendInteger(Relay_Fan.Turn_On);
					LCD20X4_PutChar(0xDF);
					LCD20X4_PutString("% ");
					// Line 1
					LCD20X4_Gotoxy(0,2);
					LCD20X4_PutString("Do am tat: ");
					LCD20X4_SendInteger(Relay_Fan.Turn_Off);
					LCD20X4_PutChar(0xDF);
					LCD20X4_PutString("% ");
				}
			if(Display_Main == 6){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Thong so bom gio 1");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("TIME 1:");
				if(Relay_Pump_TIM_1.On_Off == 1) LCD20X4_PutString("ON ");
				else											 LCD20X4_PutString("OFF");
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Hour:");
				LCD20X4_SendInteger(Relay_Pump_TIM_1.Hour);
				LCD20X4_Gotoxy(10,2);
				LCD20X4_PutString("Minute:");
				LCD20X4_SendInteger(Relay_Pump_TIM_1.Minute);
				// Line 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("Do am bat bom:");
				LCD20X4_SendInteger(Relay_Pump_TIM_1.Soil_Moisture);
				LCD20X4_PutString("% ");
			}
			if(Display_Main == 7){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Thong so bom gio 2");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("TIME 2:");
				if(Relay_Pump_TIM_2.On_Off == 1) LCD20X4_PutString("ON ");
				else											 LCD20X4_PutString("OFF");
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Hour:");
				LCD20X4_SendInteger(Relay_Pump_TIM_2.Hour);
				LCD20X4_Gotoxy(10,2);
				LCD20X4_PutString("Minute:");
				LCD20X4_SendInteger(Relay_Pump_TIM_2.Minute);
				// Line 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("Do am bat bom:");
				LCD20X4_SendInteger(Relay_Pump_TIM_2.Soil_Moisture);
				LCD20X4_PutString("% ");
			}
		}
	}  else {	
		/******************************************************* Hien thi man hinh chon che do cai dat**********************************************************/
		if(LCD_Menu_Setting){
			if(Clock_1Hz){
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Menu Setting");
				LCD20X4_Gotoxy(15,0);
				LCD20X4_PutString("EXIT");
				// Dong 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("1: ");
				LCD20X4_PutString("Set Time");
				// Dong 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("2: ");
				LCD20X4_PutString("Set Warning");
				// Dong 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("3: ");
				LCD20X4_PutString("Set Auto Mode ");
			} else {
				switch(Status_Menu_Setting){
					case 0: {																	 	break; }
					case 1: { LCD20X4_Gotoxy(0,1); LCD20X4_PutString("   ");	 	break; }
					case 2: { LCD20X4_Gotoxy(0,2); LCD20X4_PutString("   "); 	 	break; }
					case 3: { LCD20X4_Gotoxy(0,3); LCD20X4_PutString("   ");  		break; }
					case 4: { LCD20X4_Gotoxy(15,0); LCD20X4_PutString("    "); 	break; }
					default:{																	  	break; }
				}
			}
		}
		
		//*************************************************** Man hinh hien thi cai dat lai thoi gian ******************************************************//
		if(LCD_Set_Time){
			if(Clock_1Hz){
				LCD20X4_Gotoxy(15,0);
				LCD20X4_PutString("EXIT");
				LCD20X4_Gotoxy(6,2);
				LCD20X4_PutChar(DS1307.Hour / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Hour % 10 | 0x30);
				LCD20X4_PutChar(':');
				LCD20X4_PutChar(DS1307.Minute / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Minute % 10 | 0x30);
				LCD20X4_PutChar(':');
				LCD20X4_PutChar(DS1307.Second / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Second % 10 | 0x30);
			
				LCD20X4_Gotoxy(2,3);
				LCD20X4_PutString(Days[DS1307.Day-1]);
				LCD20X4_PutString("  ");
				LCD20X4_PutChar(DS1307.Date / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Date % 10 | 0x30);
				LCD20X4_PutChar('/');
				LCD20X4_PutChar(DS1307.Month / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Month % 10 | 0x30);
				LCD20X4_PutString("/20");
				LCD20X4_PutChar(DS1307.Year / 10 | 0x30);
				LCD20X4_PutChar(DS1307.Year % 10 | 0x30);
			} else {
				switch(Status_Mode_Time){
				case 1: {LCD20X4_Gotoxy(9,2);  LCD20X4_PutString("  ") ;   break; }
				case 2: {LCD20X4_Gotoxy(6,2);  LCD20X4_PutString("  ");    break; }
				case 3: {LCD20X4_Gotoxy(2,3);  LCD20X4_PutString("   ");   break; }
				case 4: {LCD20X4_Gotoxy(7,3);  LCD20X4_PutString("  ");    break; }
				case 5: {LCD20X4_Gotoxy(10,3); LCD20X4_PutString("  ");   break; }
				case 6: {LCD20X4_Gotoxy(15,3); LCD20X4_PutString("  ");   break; }
				case 7: {LCD20X4_Gotoxy(15,0); LCD20X4_PutString("    "); break; }
				default:{ 																   break; }
				}
			}
		}
		// Set Warning 
		if(LCD_Set_Warning){
			if(Clock_1Hz){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("L_Temp: ");
				LCD20X4_SendInteger(Warning.Low_Temperature);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
				LCD20X4_Gotoxy(15,0);
				LCD20X4_PutString("EXIT");
				
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("H_Temp: ");
				LCD20X4_SendInteger(Warning.High_Temperature);
				LCD20X4_PutChar(0xDF);
				LCD20X4_PutString("C ");
				
				// Line 3
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("L_Humi: ");
				LCD20X4_SendInteger(Warning.Low_Humidity);
				LCD20X4_PutString("% ");
				
				// Line 4
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("H_Humi: ");
				LCD20X4_SendInteger(Warning.High_Humidity);
				LCD20X4_PutString("% ");
			} else {
				switch(Status_Mode_Warning){
					case 0: { LCD20X4_Gotoxy(8,0); LCD20X4_PutString("   "); 	break; }
					case 1: { LCD20X4_Gotoxy(8,1); LCD20X4_PutString("   "); 	break; }
					case 2: { LCD20X4_Gotoxy(8,2); LCD20X4_PutString("   "); 	break; }
					case 3: { LCD20X4_Gotoxy(8,3); LCD20X4_PutString("   "); 	break; }
					case 4: { LCD20X4_Gotoxy(15,0); LCD20X4_PutString("    "); break; }
					default: {																  	break; }
				}
			}
		}
		// Set_Parameter_Automation
		if(LCD_Set_Auto){
			if(Clock_1Hz){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("1: ");
				LCD20X4_PutString("Den suoi");
				LCD20X4_Gotoxy(15,0);
				LCD20X4_PutString("EXIT");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("2: ");
				LCD20X4_PutString("Bom nuoc");
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("3: ");
				LCD20X4_PutString("Quat thong gio");
			} else {
				switch(Status_Mode_Auto){
					case 0: {																	break; }
					case 1: { LCD20X4_Gotoxy(0,0); LCD20X4_PutString("   "); 	break; }
					case 2: { LCD20X4_Gotoxy(0,1); LCD20X4_PutString("   "); 	break; }
					case 3: { LCD20X4_Gotoxy(0,2); LCD20X4_PutString("   ");   break; }
					case 4: { LCD20X4_Gotoxy(15,0); LCD20X4_PutString("    "); break; }
					default:{																	 	break; }
				}
			}
		}		
		if(LCD_Heating_Lamp){
			if(Clock_1Hz){
				// Line 0
					LCD20X4_Gotoxy(0,0);
					LCD20X4_PutString("Cai Dat Den Suoi");
				if(Display_Lamp == 0){
					// Line 1
					LCD20X4_Gotoxy(0,1);
					LCD20X4_PutString("Ban Ngay");
					LCD20X4_Gotoxy(10,1);
					LCD20X4_PutString("H:");
					LCD20X4_SendInteger(Relay_Lamp_Morning.Hour);
					LCD20X4_Gotoxy(15,1);
					LCD20X4_PutString("M:");
					LCD20X4_SendInteger(Relay_Lamp_Morning.Minute);
					// Line 2
					LCD20X4_Gotoxy(0,2);
					LCD20X4_PutString("Ban Dem");
					LCD20X4_Gotoxy(10,2);
					LCD20X4_PutString("H:");
					LCD20X4_SendInteger(Relay_Lamp_Evening.Hour);
					LCD20X4_Gotoxy(15,2);
					LCD20X4_PutString("M:");
					LCD20X4_SendInteger(Relay_Lamp_Evening.Minute);
				} if(Display_Lamp == 1){
					// Line 1
					LCD20X4_Gotoxy(0,1);
					LCD20X4_PutString("Ban ngay   Bat < Tat");
					// Line 2
					LCD20X4_Gotoxy(0,2);
					LCD20X4_PutString("Nhiet do bat:");
					LCD20X4_SendInteger(Relay_Lamp_Morning.On_Off.Turn_On);
					LCD20X4_PutChar(0xDF);
					LCD20X4_PutString("C ");
					// Line 3
					LCD20X4_Gotoxy(0,3);
					LCD20X4_PutString("Nhiet do tat:");
					LCD20X4_SendInteger(Relay_Lamp_Morning.On_Off.Turn_Off);
					LCD20X4_PutChar(0xDF);
					LCD20X4_PutString("C ");
				}
				if(Display_Lamp == 2){
					// Line 1
					LCD20X4_Gotoxy(0,1);
					LCD20X4_PutString("Ban dem    Bat < Tat");
					// Line 2
					LCD20X4_Gotoxy(0,2);
					LCD20X4_PutString("Nhiet do bat:");
					LCD20X4_SendInteger(Relay_Lamp_Evening.On_Off.Turn_On);
					LCD20X4_PutChar(0xDF);
					LCD20X4_PutString("C ");
					// Line 3
					LCD20X4_Gotoxy(0,3);
					LCD20X4_PutString("Nhiet do tat:");
					LCD20X4_SendInteger(Relay_Lamp_Evening.On_Off.Turn_Off);
					LCD20X4_PutChar(0xDF);
					LCD20X4_PutString("C ");
					LCD20X4_Gotoxy(16,0);
					LCD20X4_PutString("EXIT");
				}
			} else {
				switch(Status_Mode_Lamp){
					case 1: { LCD20X4_Gotoxy(12,1); LCD20X4_PutString("  "); 		break; }
					case 2: { LCD20X4_Gotoxy(17,1); LCD20X4_PutString("  ");		break; }
					case 3: { LCD20X4_Gotoxy(12,2); LCD20X4_PutString("  ");		break; }
					case 4: { LCD20X4_Gotoxy(17,2); LCD20X4_PutString("  ");		break; }
					case 5: { LCD20X4_Gotoxy(13,2); LCD20X4_PutString("  ");		break; }
					case 6: { LCD20X4_Gotoxy(13,3); LCD20X4_PutString("  ");	 	break; }
					case 7: { LCD20X4_Gotoxy(13,2); LCD20X4_PutString("  "); 		break; }
					case 8: { LCD20X4_Gotoxy(13,3); LCD20X4_PutString("  ");		break; }
					case 9: { LCD20X4_Gotoxy(16,0); LCD20X4_PutString("    "); 	break; }
					default:{																			break; }
				}
			}
		}
		if(LCD_Pump){
			if(Clock_1Hz){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Cai Dat Bom Nuoc");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("Time 1:");
				if(Relay_Pump_TIM_1.On_Off == 1) LCD20X4_PutString("ON ");
				else											 LCD20X4_PutString("OFF");
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Time 2:");
				if(Relay_Pump_TIM_2.On_Off == 1) LCD20X4_PutString("ON ");
				else											 LCD20X4_PutString("OFF");
				// Line 3
				LCD20X4_Gotoxy(3,3);
				LCD20X4_PutString("NEXT");
				LCD20X4_Gotoxy(13,3);
				LCD20X4_PutString("EXIT");
			} else {
				switch(Status_Mode_Pump_On_Off){
					case 1: { LCD20X4_Gotoxy(7,1); LCD20X4_PutString("   "); 	break; }
					case 2: { LCD20X4_Gotoxy(7,2); LCD20X4_PutString("   ");  	break; }
					case 3: { LCD20X4_Gotoxy(3,3); LCD20X4_PutString("    ");  break; }
					case 4: { LCD20X4_Gotoxy(13,3); LCD20X4_PutString("    "); break; }
					default: {																	break; }
				}
			}
		}
		if(LCD_Set_Pump_TIM_1 ||	LCD_Set_Pump_TIM_2){
			// Khi chuyen man hinh giua 2 dis
			if(Clock_1Hz){
				// Line 0
				LCD20X4_Gotoxy(2,0);
				LCD20X4_PutString("Cai Dat Bom Nuoc");
				// Line 1
				LCD20X4_Gotoxy(4,1);
				if(LCD_Set_Pump_TIM_1 && !Prioritized) LCD20X4_PutString("Thoi Gian 1");
				if(LCD_Set_Pump_TIM_2 && Prioritized) LCD20X4_PutString("Thoi Gian 2");
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Gio:");
				LCD20X4_SendInteger(Relay_Pump->Hour);
				LCD20X4_Gotoxy(10,2);
				LCD20X4_PutString("Phut:");
				LCD20X4_SendInteger(Relay_Pump->Minute);
				// Line 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("Do am dat:");
				LCD20X4_SendInteger(Relay_Pump->Soil_Moisture);
				LCD20X4_PutChar('%');
				LCD20X4_Gotoxy(16,3);
				LCD20X4_PutString("EXIT");
			} else {
				switch(Relay_Pump->Status_Mode){
					case 1: {LCD20X4_Gotoxy(4,2); 	LCD20X4_PutString("    "); break; }
					case 2: {LCD20X4_Gotoxy(15,2); LCD20X4_PutString("    "); break; }
					case 3: {LCD20X4_Gotoxy(10,3); LCD20X4_PutString("    "); break; }
					case 4: {LCD20X4_Gotoxy(16,3); LCD20X4_PutString("    "); break; }
					default: {																   break; }
				}
			}
		}
		// Relay_Fan
		if(LCD_Set_Fan){
			if(Clock_1Hz){
				// Line 0
				LCD20X4_Gotoxy(0,0);
				LCD20X4_PutString("Cai dat quat gio");
				// Line 1
				LCD20X4_Gotoxy(0,1);
				LCD20X4_PutString("Do am bat:");
				LCD20X4_SendInteger(Relay_Fan.Turn_On);
				LCD20X4_PutString("% ");
				// Line 2
				LCD20X4_Gotoxy(0,2);
				LCD20X4_PutString("Do am tat:");
				LCD20X4_SendInteger(Relay_Fan.Turn_Off);
				LCD20X4_PutString("% ");
				// Line 3
				LCD20X4_Gotoxy(0,3);
				LCD20X4_PutString("Bat > Tat");
				LCD20X4_Gotoxy(15,3);
				LCD20X4_PutString("EXIT");
				
			} else {
				switch(Status_Mode_Fan){
					case 1: { LCD20X4_Gotoxy(10,1); LCD20X4_PutString("   "); 	break; }
					case 2: { LCD20X4_Gotoxy(10,2); LCD20X4_PutString("   "); 	break; }
					case 3: { LCD20X4_Gotoxy(15,3); LCD20X4_PutString("    "); break; }
					default: {																  	break; }
				}
			}
		}
	}
}

void DieuKhien(void){
	// Bat bom gio 1
	if(Relay_Pump_TIM_1.On_Off){
		if((Relay_Pump_TIM_1.Hour == DS1307.Hour) && (Relay_Pump_TIM_1.Minute == DS1307.Minute) && (DS1307.Second < 10)){
			if(Relay_Pump_TIM_1.Soil_Moisture > DoAmThat){
				b_Auto_Set_Reset_Pump_TIM_1 = 1;
			}
		}
	}
	if(Relay_Pump_TIM_1.Soil_Moisture < DoAmThat) b_Auto_Set_Reset_Pump_TIM_1 = 0;
	// Bat bom gio 2
	if(Relay_Pump_TIM_2.On_Off){
		if((Relay_Pump_TIM_2.Hour == DS1307.Hour) && (Relay_Pump_TIM_2.Minute == DS1307.Minute) && (DS1307.Second < 10)){
			if(Relay_Pump_TIM_2.Soil_Moisture > DoAmThat){
				b_Auto_Set_Reset_Pump_TIM_2 = 1;
			}
		}
	}
	if(Relay_Pump_TIM_2.Soil_Moisture < DoAmThat) b_Auto_Set_Reset_Pump_TIM_2 = 0;
		
	// Bat den suoi
	// Ban ngay
	if((Relay_Lamp_Morning.Hour == DS1307.Hour && Relay_Lamp_Morning.Minute <= DS1307.Minute) || Relay_Lamp_Morning.Hour < DS1307.Hour){
		if((Relay_Lamp_Evening.Hour == DS1307.Hour && Relay_Lamp_Evening.Minute >= DS1307.Minute) || Relay_Lamp_Evening.Hour > DS1307.Hour){
			if(Relay_Lamp_Morning.On_Off.Turn_On > f_Nhiet_Do){
				b_Auto_Morning_Set_Reset_Lamp = 1;
			}
			if(Relay_Lamp_Morning.On_Off.Turn_Off < f_Nhiet_Do){
				b_Auto_Morning_Set_Reset_Lamp = 0;
			}
		}
	} else { 	// Ban dem
		if(Relay_Lamp_Evening.On_Off.Turn_On > f_Nhiet_Do){
			b_Auto_Evening_Set_Reset_Lamp = 1;
		}
		if(Relay_Lamp_Evening.On_Off.Turn_Off < f_Nhiet_Do){
			b_Auto_Evening_Set_Reset_Lamp = 0;
		}
	}	
		
	// Bat quat thong gio
	if(Relay_Fan.Turn_On < f_Do_Am_Khong_Khi)	b_Auto_Set_Reset_Fan = 1;
	if(Relay_Fan.Turn_Off >= f_Do_Am_Khong_Khi) 	b_Auto_Set_Reset_Fan = 0;
	
	
	if( !Mode_Auto_Or_Manual){
		// Mode Auto
		// Bom nuoc
		if(b_Auto_Set_Reset_Pump_TIM_1 || b_Auto_Set_Reset_Pump_TIM_2){
			HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, GPIO_PIN_RESET);
			b_Status_Pump = 1;
		} else {
			HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, GPIO_PIN_SET);
			b_Status_Pump = 0;
		}
		// Den suoi
		if(b_Auto_Morning_Set_Reset_Lamp ||b_Auto_Evening_Set_Reset_Lamp){
			HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, GPIO_PIN_RESET);
			b_Status_Lamp = 1;
		} else {
			HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, GPIO_PIN_SET);
			b_Status_Lamp = 0;
		}
		// Quat
		if(b_Auto_Set_Reset_Fan){
			HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, GPIO_PIN_RESET);
			b_Status_Fan = 1;
		} else {
			HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, GPIO_PIN_SET);
			b_Status_Fan = 0;
		}
	} else {
		// Bom nuoc
		if(b_Manual_Set_Reset_Pump){
			HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, GPIO_PIN_RESET);
			b_Status_Pump = 1;
		} else {
			HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, GPIO_PIN_SET);
			b_Status_Pump = 0;
		}
		// Den suoi
		if(b_Manual_Set_Reset_Heating_Lamp){
			HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, GPIO_PIN_RESET);
			b_Status_Lamp = 1;
		} else {
			HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, GPIO_PIN_SET);
			b_Status_Lamp = 0;
		}
		// Quat
		if(b_Manual_Set_Reset_Fan){
			HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, GPIO_PIN_RESET);
			b_Status_Fan = 1;
		} else {
			HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, GPIO_PIN_SET);
			b_Status_Fan = 0;
		}
	}
	// Canh bao do am thap
	if(Warning.Low_Humidity > f_Do_Am_Khong_Khi) b_Warning_Low_Humidity = 1;
	else												  				b_Warning_Low_Humidity = 0;
	
	// Canh bao do am cao
	if(Warning.High_Humidity < f_Do_Am_Khong_Khi) b_Warning_High_Humidity = 1;
	else												  				b_Warning_High_Humidity = 0;
	
	//Canh bao nhiet do thap
	if(Warning.Low_Temperature > f_Nhiet_Do) b_Warning_Low_Temperature = 1;
	else													   b_Warning_Low_Temperature = 0;
	
	// Canh bao nhiet do cao
	if(Warning.High_Temperature < f_Nhiet_Do) b_Warning_High_Temperature = 1;
	else														b_Warning_High_Temperature = 0;
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
