/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "loopback.h"
#include "wizchip_conf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
OSPI_RegularCmdTypeDef com;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

OSPI_HandleTypeDef hospi1;
MDMA_HandleTypeDef hmdma_octospi1_fifo_th;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t W6300_mode = 0x00;//0; //W6100 >> 0xFF
wiz_NetInfo gWIZNETINFO = {.mac = {0x00, 0x08, 0xdc, 0x12, 0x34, 0x45},
                           .ip = {192, 168, 15, 100},
                           .sn = {255, 255, 255, 0},
                           .gw = {192, 168, 15, 1},
                           .dns = {8, 8, 8, 8},
                           .lla = {0xfe, 0x80, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00,
                                   0x02, 0x08, 0xdc, 0xff,
                                   0xfe, 0xff, 0xff, 0xff},
                           .gua = {0x20, 0x01, 0x02, 0xb8,
                                   0x00, 0x10, 0x00, 0x01,
                                   0x02, 0x08, 0xdc, 0xff,
                                   0xfe, 0xff, 0xff, 0xff},
                           .sn6 = {0xff, 0xff, 0xff, 0xff,
                                   0xff, 0xff, 0xff, 0xff,
                                   0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00},
                           .gw6 = {0xfe, 0x80, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00,
                                   0x02, 0x00, 0x87, 0xff,
                                   0xfe, 0x08, 0x4c, 0x81}};

uint8_t WIZ_Dest_IP[4] = {192, 168, 15, 2};                  //DST_IP Address


uint8_t DestIP6_L[16] = {0xfe,0x80, 0x00,0x00,
						  0x00,0x00, 0x00,0x00,
                          0x31,0x71,0x98,0x05,
                          0x70,0x24,0x4b,0xb1
						};

uint8_t DestIP6_G[16] = {0x20,0x01,0x02,0xb8,
                          0x00,0x10,0x00,0x01,
                          0x31,0x71,0x98,0x05,
                          0x70,0x24,0x4b,0xb1
                         };

uint8_t Router_IP[16]= {0xff,0x02,0x00,0x00,
                          0x00,0x00,0x00,0x00,
                          0x00,0x00,0x00,0x00,
                          0x00,0x00,0x00,0x02
                         };
uint8_t data_buff[2048];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t rxData;
uint8_t rx_buffer[2048]= {0,};
int rx_index = 0;
uint8_t rx_flag =0;
uint8_t qspi_flag_tx = 0, qspi_flag_rx = 0;

int _write(int fd, char *str, int len)
{
  for (int i = 0; i < len; i++)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)&str[i], 1, 0xFFFF);
  }
  return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if 0
  if (is_testing == 1)
  {
    PRINT_DBG("testing...\r\n");
  }
  else // is_testing == 0
  {
    if (cmd_mode == 0)
    {
      HAL_UART_Transmit(&huart2, &rxData, 1, 1000);
    }
    if (rxData == '\n')
    {
      if (rx_buffer[rx_index - 1] == '\r')
      {
        rx_index--;
        rx_flag = 1;
        rx_buffer[rx_index] = 0;
      }
      else
      {
        rx_index = 0;
        HAL_UART_Transmit(&huart2, "not support format\r\n", 20, 1000);
      }
    }
    else if (rxData == 0x08) // back space
    {
      rx_index--;
    }
    else
    {
      rx_buffer[rx_index++] = rxData;
    }
  }


#endif
  HAL_UART_Receive_IT(&huart2, &rxData, 1);
}
void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi)
{
	qspi_flag_rx = 0;
}
void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi)
{
	qspi_flag_tx = 0;
}
uint8_t qspi_read_buf(uint8_t op_code, uint32_t AddrSel, uint8_t *pbuf, uint16_t len)
{
    uint32_t OSPI_status = 0;
    uint8_t ret = 0;
	switch(W6300_mode)
	{
		case 0x00:	//single
			com.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
			com.DataMode = HAL_OSPI_DATA_1_LINE;//HAL_OSPI_DATA_4_LINES;

			com.DummyCycles = 8;
			break;
		case 0x01:	//dual
			com.AddressMode = HAL_OSPI_ADDRESS_2_LINES;
			com.DataMode = HAL_OSPI_DATA_2_LINES;
			//com.InstructionMode = HAL_OSPI_INSTRUCTION_2_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
			com.DummyCycles = 4;
			break;
		case 0x02:	//quad
			com.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
			com.DataMode = HAL_OSPI_DATA_4_LINES;
			//com.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
			com.DummyCycles = 2;
			break;
        case 0x04:
            printf("not Qspi mode bus mode \r\n");
            return 6;
            break;
		default :
			com.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
			com.DataMode = HAL_OSPI_DATA_1_LINE;//HAL_OSPI_DATA_4_LINES;
			//com.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;//HAL_OSPI_INSTRUCTION_4_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
			com.DummyCycles = 0;
			break;
	}
	com.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;//HAL_OSPI_INSTRUCTION_4_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...

	com.Instruction = op_code;//0xAB;    // Command
	com.AddressSize = HAL_OSPI_ADDRESS_16_BITS;
	//com.AddressMode = HAL_OSPI_ADDRESS_1_LINE;//HAL_OSPI_ADDRESS_4_LINES;//QSPI_ADDRESS_1_LINE;
	com.Address = AddrSel;//0x00000000;

	com.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	com.AlternateBytes = HAL_OSPI_ALTERNATE_BYTES_NONE;
	com.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_NONE;


	//com.DataMode = HAL_OSPI_DATA_1_LINE;//HAL_OSPI_DATA_4_LINES;
	//com.NbData = 1;

	com.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
	//com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	com.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	com.NbData = len;
  if ((ret = HAL_OSPI_Command(&hospi1, &com, 10)) != HAL_OK)
  {
		printf("[%s > %s : %d]Cmd Error ret:%02X st:%X\r\n",__FILE__, __FUNCTION__, __LINE__ , ret, (uint16_t)OSPI_status);
		return 4;
	}
	qspi_flag_rx = 1;
	if ((ret = HAL_OSPI_Receive_DMA(&hospi1, pbuf)) != HAL_OK)
	{
		printf("[%s > %s : %d]Recv Error ret:%02X st:%X\r\n",__FILE__, __FUNCTION__, __LINE__, ret, (uint16_t)OSPI_status);
		return 5;
	}
#if 1
	while(qspi_flag_rx)
	{
		//count and return
	}
#endif
#if 0
    while((OSPI_status=HAL_OSPI_GetState(&hospi1)) > 4)
    {
        hospi1.Instance->FCR = OSPI_status;
    	//hospi1.State = 0;
    }
#endif
	return 0;
}
uint8_t qspi_write_buf(uint8_t op_code, uint32_t AddrSel, uint8_t *pbuf, uint16_t len)
{
    uint32_t OSPI_status = 0;
    uint8_t ret = 0;
	switch(W6300_mode)
	{
		case 0x00:	//single
			com.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
			com.DataMode = HAL_OSPI_DATA_1_LINE;//HAL_OSPI_DATA_4_LINES;
			//com.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;//HAL_OSPI_INSTRUCTION_4_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
			com.DummyCycles = 8;
			break;
		case 0x01:	//dual
			com.AddressMode = HAL_OSPI_ADDRESS_2_LINES;
			com.DataMode = HAL_OSPI_DATA_2_LINES;
			//com.InstructionMode = HAL_OSPI_INSTRUCTION_2_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
			com.DummyCycles = 4;
			break;
		case 0x02:	//quad
			com.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
			com.DataMode = HAL_OSPI_DATA_4_LINES;
			//com.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
			com.DummyCycles = 2;
			break;
        case 0x04:
            printf("not Qspi mode bus mode \r\n");
            return 6;
            break;
		default :
			com.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
			com.DataMode = HAL_OSPI_DATA_1_LINE;//HAL_OSPI_DATA_4_LINES;
			//com.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;//HAL_OSPI_INSTRUCTION_4_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
			com.DummyCycles = 0;
			break;
	}
	com.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;//HAL_OSPI_INSTRUCTION_4_LINES;//QSPI_INSTRUCTION_1_LINE; // QSPI_INSTRUCTION_...
	com.Instruction = op_code;//0xAB;    // Command
	com.AddressSize = HAL_OSPI_ADDRESS_16_BITS;
	//com.AddressMode = HAL_OSPI_ADDRESS_1_LINE;//HAL_OSPI_ADDRESS_4_LINES;//QSPI_ADDRESS_1_LINE;
	com.Address = AddrSel;//0x00000000;

	com.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	com.AlternateBytes = HAL_OSPI_ALTERNATE_BYTES_NONE;
	com.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_NONE;


	//com.DataMode = HAL_OSPI_DATA_1_LINE;//HAL_OSPI_DATA_4_LINES;
	//com.NbData = 1;

	com.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
	//com.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	com.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	com.NbData = len;
	//if ((ret = HAL_OSPI_Command(&hospi1, &com, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK)
  if ((ret = HAL_OSPI_Command(&hospi1, &com, 10)) != HAL_OK)
	{
		printf("[%s > %s : %d]CMD Error ret:%02X st:%X\r\n",__FILE__, __FUNCTION__, __LINE__, ret, (uint16_t)OSPI_status);
		return 4;
	}
	qspi_flag_tx = 1;
	if ((ret = HAL_OSPI_Transmit_DMA(&hospi1, pbuf)) != HAL_OK)
	{
		printf("[%s > %s : %d]Send Error ret:%02X, st:%X\r\n",__FILE__, __FUNCTION__, __LINE__, ret, (uint16_t)OSPI_status);
		return 5;
	}
#if 1
	while(qspi_flag_tx)
	{
		//count and return
	}
#endif
#if 0
	while((OSPI_status=HAL_OSPI_GetState(&hospi1)) > 4)
    {
        hospi1.Instance->FCR = OSPI_status;
		//hospi1.State = 0;
    }
#endif
	return 0;
}
void W6300Initialze(void)
{
	//W6100Reset();

#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_QSPI_

   reg_wizchip_qspi_cbfunc(qspi_read_buf, qspi_write_buf);

#elif _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_
/* SPI method callback registration */
	#if defined SPI_DMA
	reg_wizchip_spi_cbfunc(W6100SpiReadByte, W6100SpiWriteByte, W6100SpiReadBurst, W6100SpiWriteBurst);
	#else
	reg_wizchip_spi_cbfunc(W6100SpiReadByte, W6100SpiWriteByte, 0, 0);
	#endif
	/* CS function register */
	reg_wizchip_cs_cbfunc(W6100CsEnable, W6100CsDisable);
#else
/* Indirect bus method callback registration */
	#if defined BUS_DMA
	reg_wizchip_bus_cbfunc(W6100BusReadByte, W6100BusWriteByte, W6100BusReadBurst, W6100BusWriteBurst);
	#else
	reg_wizchip_bus_cbfunc(W6100BusReadByte, W6100BusWriteByte, 0, 0);
	#endif
#endif
	uint8_t temp;
	//unsigned char W6100_AdrSet[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};
   // unsigned char W6100_AdrSet[2][8] = {{4, 4, 4, 4, 4, 4, 4, 4}, {4, 4, 4, 4, 4, 4, 4, 4}};
	unsigned char W6100_AdrSet[2][8] = {{0, 8, 0, 0, 0, 0, 0, 0}, {0, 8, 0, 0, 0, 0, 0, 0}};
/*	do
	{
		if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
		{
			printf("Unknown PHY link status.\r\n");
		}
	} while (temp == PHY_LINK_OFF);
	printf("PHY OK.\r\n");
*/
	temp = IK_DEST_UNREACH;

	if (ctlwizchip(CW_INIT_WIZCHIP, (void *)W6100_AdrSet) == -1)
	{
		printf("W6300 initialized fail.\r\n");
	}

	if (ctlwizchip(CW_SET_INTRMASK, &temp) == -1)
	{
		printf("W6300 interrupt\r\n");
	}
	//printf("interrupt mask: %02x\r\n",getIMR());
}
uint32_t get_time(void)
{
    return HAL_GetTick();
}
void print_network_information(void);
void FPGA_Reset(void);
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
  uint16_t i = 0;
  uint8_t syslock = SYS_NET_LOCK;
  uint8_t tmp = 0;
  PLL2_ClocksTypeDef PLL2_Clk_data;
  char mode_char[4][5]={"Sing","Dual","Quad"};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MDMA_Init();
  MX_OCTOSPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("W6300 loop back test \r\n");
  printf("Compile %s - %s \r\n", __DATE__, __TIME__);
  HAL_UART_Receive_IT(&huart2, &rxData, 1);
  HAL_RCCEx_GetPLL2ClockFreq(&PLL2_Clk_data);
  printf("SET PLL2 P:%ld, Q:%ld, R:%ld \r\n", PLL2_Clk_data.PLL2_P_Frequency, PLL2_Clk_data.PLL2_Q_Frequency, PLL2_Clk_data.PLL2_R_Frequency);
  printf("QSPI CLK %d Mhz \r\n", (uint16_t)(PLL2_Clk_data.PLL2_R_Frequency / hospi1.Init.ClockPrescaler / 1000000));
#if 1
#if (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_BUS_INDIR_)
  HAL_GPIO_WritePin(MOD0_GPIO_Port, MOD0_Pin, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(MOD0_GPIO_Port, MOD0_Pin, GPIO_PIN_RESET);
#endif
  HAL_GPIO_WritePin(MOD1_GPIO_Port, MOD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOD2_GPIO_Port, MOD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOD3_GPIO_Port, MOD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOD4_GPIO_Port, MOD4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOD5_GPIO_Port, MOD5_Pin, GPIO_PIN_RESET);
#endif

  HAL_GPIO_WritePin(RSTn_GPIO_Port, RSTn_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(RSTn_GPIO_Port, RSTn_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
#if 1
  HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_Delay(500);
#endif
  W6300Initialze();
  ctlwizchip(CW_SYS_UNLOCK, &syslock);
  ctlnetwork(CN_SET_NETINFO, &gWIZNETINFO);
  printf("VERSION(%04x) = %04x \r\n", _VER_, getVER());
  for (i = 0; i < 8; i++)
  {
    printf("%d : max size = %d k \r\n", i, getSn_TxMAX(i));
  }
  print_network_information();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	loopback_tcps(1, data_buff, 5000, AS_IPV4);
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
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI|RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 60;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 5;
  PeriphClkInitStruct.PLL2.PLL2R = 10;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 17;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 2;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MOD4_Pin|MOD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RSTn_GPIO_Port, RSTn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOD0_Pin|SPI_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MOD2_Pin|MOD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOD1_GPIO_Port, MOD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOD4_Pin MOD5_Pin */
  GPIO_InitStruct.Pin = MOD4_Pin|MOD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RSTn_Pin */
  GPIO_InitStruct.Pin = RSTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RSTn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOD0_Pin SPI_EN_Pin */
  GPIO_InitStruct.Pin = MOD0_Pin|SPI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOD2_Pin MOD3_Pin */
  GPIO_InitStruct.Pin = MOD2_Pin|MOD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MOD1_Pin */
  GPIO_InitStruct.Pin = MOD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOD1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print_network_information(void)
{
	wiz_NetInfo gWIZNETINFO1;
#if 0
	wizchip_getnetinfo(&gWIZNETINFO);
	printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
	printf("IP address : %d.%d.%d.%d\r\n",gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
	printf("SN Mask	   : %d.%d.%d.%d\r\n",gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
	printf("Gate way   : %d.%d.%d.%d\r\n",gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
	printf("DNS Server : %d.%d.%d.%d\r\n",gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
	printf("LLA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO.lla[0],gWIZNETINFO.lla[1],gWIZNETINFO.lla[2],gWIZNETINFO.lla[3],\
									gWIZNETINFO.lla[4],gWIZNETINFO.lla[5],gWIZNETINFO.lla[6],gWIZNETINFO.lla[7],\
									gWIZNETINFO.lla[8],gWIZNETINFO.lla[9],gWIZNETINFO.lla[10],gWIZNETINFO.lla[11],\
									gWIZNETINFO.lla[12],gWIZNETINFO.lla[13],gWIZNETINFO.lla[14],gWIZNETINFO.lla[15]);
	printf("GUA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO.gua[0],gWIZNETINFO.gua[1],gWIZNETINFO.gua[2],gWIZNETINFO.gua[3],\
									gWIZNETINFO.gua[4],gWIZNETINFO.gua[5],gWIZNETINFO.gua[6],gWIZNETINFO.gua[7],\
									gWIZNETINFO.gua[8],gWIZNETINFO.gua[9],gWIZNETINFO.gua[10],gWIZNETINFO.gua[11],\
									gWIZNETINFO.gua[12],gWIZNETINFO.gua[13],gWIZNETINFO.gua[14],gWIZNETINFO.gua[15]);
	printf("SN6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO.sn6[0],gWIZNETINFO.sn6[1],gWIZNETINFO.sn6[2],gWIZNETINFO.sn6[3],\
									gWIZNETINFO.sn6[4],gWIZNETINFO.sn6[5],gWIZNETINFO.sn6[6],gWIZNETINFO.sn6[7],\
									gWIZNETINFO.sn6[8],gWIZNETINFO.sn6[9],gWIZNETINFO.sn6[10],gWIZNETINFO.sn6[11],\
									gWIZNETINFO.sn6[12],gWIZNETINFO.sn6[13],gWIZNETINFO.sn6[14],gWIZNETINFO.sn6[15]);
	printf("GW6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO.gw6[0],gWIZNETINFO.gw6[1],gWIZNETINFO.gw6[2],gWIZNETINFO.gw6[3],\
									gWIZNETINFO.gw6[4],gWIZNETINFO.gw6[5],gWIZNETINFO.gw6[6],gWIZNETINFO.gw6[7],\
									gWIZNETINFO.gw6[8],gWIZNETINFO.gw6[9],gWIZNETINFO.gw6[10],gWIZNETINFO.gw6[11],\
									gWIZNETINFO.gw6[12],gWIZNETINFO.gw6[13],gWIZNETINFO.gw6[14],gWIZNETINFO.gw6[15]);
#else
	wizchip_getnetinfo(&gWIZNETINFO1);
		printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",gWIZNETINFO1.mac[0],gWIZNETINFO1.mac[1],gWIZNETINFO1.mac[2],gWIZNETINFO1.mac[3],gWIZNETINFO1.mac[4],gWIZNETINFO1.mac[5]);
		printf("IP address : %d.%d.%d.%d\r\n",gWIZNETINFO1.ip[0],gWIZNETINFO1.ip[1],gWIZNETINFO1.ip[2],gWIZNETINFO1.ip[3]);
		printf("SN Mask	   : %d.%d.%d.%d\r\n",gWIZNETINFO1.sn[0],gWIZNETINFO1.sn[1],gWIZNETINFO1.sn[2],gWIZNETINFO1.sn[3]);
		printf("Gate way   : %d.%d.%d.%d\r\n",gWIZNETINFO1.gw[0],gWIZNETINFO1.gw[1],gWIZNETINFO1.gw[2],gWIZNETINFO1.gw[3]);
		//printf("DNS Server : %d.%d.%d.%d\r\n",gWIZNETINFO1.dns[0],gWIZNETINFO1.dns[1],gWIZNETINFO1.dns[2],gWIZNETINFO1.dns[3]);
		printf("LLA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO1.lla[0],gWIZNETINFO1.lla[1],gWIZNETINFO1.lla[2],gWIZNETINFO1.lla[3],\
										gWIZNETINFO1.lla[4],gWIZNETINFO1.lla[5],gWIZNETINFO1.lla[6],gWIZNETINFO1.lla[7],\
										gWIZNETINFO1.lla[8],gWIZNETINFO1.lla[9],gWIZNETINFO1.lla[10],gWIZNETINFO1.lla[11],\
										gWIZNETINFO1.lla[12],gWIZNETINFO1.lla[13],gWIZNETINFO1.lla[14],gWIZNETINFO1.lla[15]);
		printf("GUA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO1.gua[0],gWIZNETINFO1.gua[1],gWIZNETINFO1.gua[2],gWIZNETINFO1.gua[3],\
										gWIZNETINFO1.gua[4],gWIZNETINFO1.gua[5],gWIZNETINFO1.gua[6],gWIZNETINFO1.gua[7],\
										gWIZNETINFO1.gua[8],gWIZNETINFO1.gua[9],gWIZNETINFO1.gua[10],gWIZNETINFO1.gua[11],\
										gWIZNETINFO1.gua[12],gWIZNETINFO1.gua[13],gWIZNETINFO1.gua[14],gWIZNETINFO1.gua[15]);
		printf("SN6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO1.sn6[0],gWIZNETINFO1.sn6[1],gWIZNETINFO1.sn6[2],gWIZNETINFO1.sn6[3],\
										gWIZNETINFO1.sn6[4],gWIZNETINFO1.sn6[5],gWIZNETINFO1.sn6[6],gWIZNETINFO1.sn6[7],\
										gWIZNETINFO1.sn6[8],gWIZNETINFO1.sn6[9],gWIZNETINFO1.sn6[10],gWIZNETINFO1.sn6[11],\
										gWIZNETINFO1.sn6[12],gWIZNETINFO1.sn6[13],gWIZNETINFO1.sn6[14],gWIZNETINFO1.sn6[15]);
		printf("GW6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO1.gw6[0],gWIZNETINFO1.gw6[1],gWIZNETINFO1.gw6[2],gWIZNETINFO1.gw6[3],\
										gWIZNETINFO1.gw6[4],gWIZNETINFO1.gw6[5],gWIZNETINFO1.gw6[6],gWIZNETINFO1.gw6[7],\
										gWIZNETINFO1.gw6[8],gWIZNETINFO1.gw6[9],gWIZNETINFO1.gw6[10],gWIZNETINFO1.gw6[11],\
										gWIZNETINFO1.gw6[12],gWIZNETINFO1.gw6[13],gWIZNETINFO1.gw6[14],gWIZNETINFO1.gw6[15]);
#endif
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
