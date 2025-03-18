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
#define TRUE 1
#define FALSE 0
#define DEBUG_RX 1

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Define packet structure */
typedef struct {
    uint8_t delim0;     // Starting delimiter (0x3C)
    uint8_t interface;  // Interface identifier
    uint8_t cmdH;       // Command high byte
    uint8_t cmdL;       // Command low byte
    uint8_t lengthH;    // Payload length high byte
    uint8_t lengthL;    // Payload length low byte
    uint8_t cmd;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t msb_byte4;
    uint8_t ropk_byte5;
    uint8_t l2hdr;
    uint8_t byte7;
    uint8_t byte8;
    uint8_t byte9;
    uint8_t byte10;
    uint8_t byte11;
    uint8_t payload[123];   //Payload data
    uint8_t checksum;   // Checksum value
    uint8_t delim1;     // Ending delimiter (0x3E)
} DataPacket;


typedef enum {
	SM_STATE_MASTER,
	SM_STATE_SLAVE,
	SM_STATE_OTHER
} SystemState;

#define MAX_RUN_TIME    (60 * 5)
#define NUM_OF_POLL_FD  1

#define PIN_ASSERT      GPIO_HIGH
#define PIN_DEASSERT    GPIO_LOW

#define MAX_FORWARD_DISPLAY_TEXT_LENGTH   150
#define MAX_SALOON_DISPLAY_TEXT_LENGTH    800

#define MAX_FRAME_LIST_SIZE   20


typedef struct pad_name_TAG{
	int gpio_num;
	int pin_num;
} pad_name;

static pad_name hreq_pad = { 1, 3 };
static pad_name srdy_pad = { 1, 2 };
static pad_name sreq_pad = { 1, 9 };
static int hreq_pin = -1;
static int srdy_pin = -1;
static int sreq_pin = -1;
static int fd_hreq = -1;
static int fd_srdy = -1;
static int fd_sreq = -1;
static int hreq_timeout_ms = 30;
static int srdy_timeout_ms = 30;
static int sreq_timeout_ms = 30;
static int pwr_comms_fd= -1;

/* ======================  SPI Declarations =======================  */
/* Common definitions for both sides of the link                     */

/* The SPI transfer needs to have exactly this size.
   This is an arbitrary number but it needs to be agreed by the
   Master and the Slave.                                            */
#define SPI_LEN		224

/* The full transaction buffer has three pieces, the length byte, the
   type-of-transaction byte and the data itself, this is why the data
   length is 2 bytes smaller than the MAX_SPI_LEN value.             */
#define MAX_DATA_SIZE   (SPI_LEN - 2)

/* The Transaction Type enum values are to be defined depending on
   the application needs. The following types of messages are just
   examples.                                                         */
typedef enum {
	TT_NULL = 0,  /* Null value (Only mandatory element of the enum) */
	TT_DOWN_IDENT, /* Downlink message asking for device ID          */
	TT_UP_NID,    /* Uplink message containing the device & Prog ID  */
	TT_UP_MSG,    /* Generic Uplink (Slave to Master) message        */
	TT_DOWN_MSG,  /* Generic Downlink (Master to Slave) message      */
	TT_UP_TFXMSG,  /* Uplink tfxmessage */
	TT_DOWN_TFXMSG /* Downlink tfxmessage */
} TTYPE;

/* All transactions are contained in this structure.  The length of
   the transaction (transLen) will include transType and the length
   of the data, it won't include transLen.                           */
typedef struct {
	uint8_t transLen;
	uint8_t transType;	/* enum TTYPE */
	uint8_t data[MAX_DATA_SIZE];
} sSpiBuf;



typedef enum
{
  TEST = 0,
  FORWARDFACING = 1,
  SALOON = 2,
  NEURON = 3, //!< reserved for local packets from echelon to attached device
  SIDE = 4,
  KEYPAD = 5,
  AUDIO = 8,
  TALKBACKAMP = 9,
  GLOBAL = 10,
  NONE = 11,
  APISC = 12
} tfxOutputType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG "I'm sending the following string \r\n"
#define MAIN_MENU "Select the option you are interested in:\r\n\t1. Toggle LD2 LED\r\n\t2. Read USER BUTTON status\r\n\t3. Clear screen and print this message"
#define PROMPT "\r\n> "
#define TIME 2500


#define FALSE 0
#define TRUE  1

#include <stdint.h>

//#define DEBUG_SEMLTCH_DECODE 1
//#define DEBUG_SEMLTCH_DECODE_BASIC 1
#define PWL_START 0x3C
#define PWL_ESC 0x3D
#define PWL_END 0x3E

typedef struct
{
  int16_t msg_len;
  int16_t echelon_msg_len;
  uint8_t semitech_header[20];
  uint8_t echelon_header[12];
  uint16_t semitech_command;
  uint8_t checksum;

} power_line_data_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SystemState SM_State;
char msgIn[256];
int count_find;
int last_val;
int last_val_slave;
char decoded_payload[256];
int result_Cogan;
int result_Received;
bool echealon_payload = FALSE;
__IO ITStatus UartReady = SET;

/* Private function prototypes -----------------------------------------------*/


char msgOut[256];
char msgOutR[256];
char msgPayload[256];
char msgStatus[256];
int total_length;



int rbp = 0; // Buffer position
bool rbpkt = false; // Packet flag
int packet_count = 0; // Packet counter
bool new_data_received = false; // Flag to indicate new data reception

int mod1_count = 0;//module one
int mod2_count = 0;//module two
int mod3_count = 0;//module yhree
int mod4_count = 0;
uint32_t start_time,end_time, elapsed_time;


uint8_t act_rx_packet[65];
uint8_t FinalData[150];
uint8_t RxData[150];
uint8_t temp[2];
int indx = 0;
int global_last_val =0;
uint8_t buffer_slave_it[1024];
uint8_t buffer2[1024];
uint8_t Acounter = 0, incorrect_val = 0, correct_val=0,skipped_val =0;
uint8_t rxData_getProtocol[150];
uint8_t copy_rxData_getProtocol[150];
int des_length=10;

#define GPIO_PIN_ENTRY GPIO_PIN_4   // PB4
#define GPIO_PORT_ENTRY GPIOB       // GPIOB



#define MODULE_1_PIN GPIO_PIN_6  // PA6
#define MODULE_2_PIN GPIO_PIN_7  // PA7
#define MODULE_3_PIN GPIO_PIN_7  // PC7
#define MODULE_4_PIN GPIO_PIN_8  // PA8

// Define GPIO ports
#define MODULE_1_PORT GPIOA
#define MODULE_2_PORT GPIOA
#define MODULE_3_PORT GPIOC
#define MODULE_4_PORT GPIOA


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t calculate_crc16(const unsigned char* const message, const unsigned int messageLength);
static int build_message_frames(uint8_t* test_text, sSpiBuf* frames, int max_frames, uint8_t output_type);
static int get_header_length(uint8_t *text);
void send_test_message(uint8_t *test_msg);
uint8_t* send_custom_text(uint8_t *test_msg);
uint8_t checksum_packet(uint8_t *packet, int packet_length);
void AppSlaveBoard();
void AppMasterBoard();
void start_transmission(void);
void P2P_Init(void);
void Device_Process();
void Init_Device(void);
void received_custom_text(uint8_t *custom_message);
char* my_strstr(char* str1, char* str2, int num_bytes);
char* decode_powerline(char* buffer, int buffer_length, power_line_data_t *pwl_data, char* payload, int *bytes_read);
int decode_finalData(uint8_t * message );
void startModuleTrans();
uint8_t next_module(uint8_t x);




// UART Example
//void printWelcomeMessage(void);
//uint8_t readUserInput(void);
//uint8_t processUserInput(uint8_t opt);

// Data Packet for Semitech EVK2400
void sendPacket(DataPacket* packet);
uint8_t calculateChecksum(DataPacket* packet);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define sMAX_CRC_DIGITS   256
#define sCRC_BYTE_WIDTH   8
#define sCRC_WIDTH        16
#define sCRC_MASK         0xFF

// N.B.
//
// <header><message_frame><crc:2>
// <header> = <output_type:1>\T<transaction_id:3>\X<frames:1>\F<frame_index:1>
//
// header size + frame size + CRC size <= transfer limit
// 12          + frame size + 2        <= 224
//
// => frame_size <= 224 - 12 - 2
//    frame_size <= 210
// N.B. MAX_DATA_SIZE breaks saloon displays, hence 'sPOWERLINE_TRANSFER_LIMIT' set lower
#define sPOWERLINE_TRANSFER_LIMIT    200
#define sPOWERLINE_FRAME_SIZE        (sPOWERLINE_TRANSFER_LIMIT - 12 - 2)


static int trans_id=1;
static int counterPacket=1;

  // Static CRC table used to calculate CRC values.
  static const unsigned short sCRC_TABLE[sMAX_CRC_DIGITS] = {
  	  0,32773,32783,   10,32795,   30,   20,32785,32819,   54,
  	 60,32825,   40,32813,32807,   34,32867,  102,  108,32873,
  	120,32893,32887,  114,   80,32853,32863,   90,32843,   78,
  	 68,32833,32963,  198,  204,32969,  216,32989,32983,  210,
  	240,33013,33023,  250,33003,  238,  228,32993,  160,32933,
  	32943,  170,32955,  190,  180,32945,32915,  150,  156,32921,
  	136,32909,32903,  130,33155,  390,  396,33161,  408,33181,
  	33175,  402,  432,33205,33215,  442,33195,  430,  420,33185,
  	480,33253,33263,  490,33275,  510,  500,33265,33235,  470,
  	476,33241,  456,33229,33223,  450,  320,33093,33103,  330,
  	33115,  350,  340,33105,33139,  374,  380,33145,  360,33133,
  	33127,  354,33059,  294,  300,33065,  312,33085,33079,  306,
  	272,33045,33055,  282,33035,  270,  260,33025,33539,  774,
  	780,33545,  792,33565,33559,  786,  816,33589,33599,  826,
  	33579,  814,  804,33569,  864,33637,33647,  874,33659,  894,
  	884,33649,33619,  854,  860,33625,  840,33613,33607,  834,
  	960,33733,33743,  970,33755,  990,  980,33745,33779, 1014,
  	1020,33785, 1000,33773,33767,  994,33699,  934,  940,33705,
  	952,33725,33719,  946,  912,33685,33695,  922,33675,  910,
  	900,33665,  640,33413,33423,  650,33435,  670,  660,33425,
  	33459,  694,  700,33465,  680,33453,33447,  674,33507,  742,
  	748,33513,  760,33533,33527,  754,  720,33493,33503,  730,
  	33483,  718,  708,33473,33347,  582,  588,33353,  600,33373,
  	33367,  594,  624,33397,33407,  634,33387,  622,  612,33377,
  	544,33317,33327,  554,33339,  574,  564,33329,33299,  534,
  	540,33305,  520,33293,33287,  514
  };

  uint8_t echecomm_header[8]={0x32, 0xA3, 0xBA, 0x00, 0x49, 0x53, 0x49, 0x01};
  uint8_t semitechcomm_header[18]={0x3C, 0x98, 0x01, 0x00, 0x00, 0x87, 0x00, 0x00,  0x00, 0x00, 0x00, 0x7B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


  uint8_t buffer_1k[1024];
  uint8_t combined_message[150];
  uint8_t rx_message[255];
  uint8_t rxin_buf[255];

  int gap_time = 2500; // time in ms 3500


  char test_msg_text1[] = "\\D1\\C2\\P00\\M10\\J00 Hello World.";
  char test_msg_text2[] = "\\D1\\C1\\P128\\M11\\J00ABCDEFGHIJKLMNOPQRSTUVWXYZ-abcdefghijklmnopqrstuvwxyz+1234567890++0987654321";
  char test_msg_text3[] = "\\D1\\C2\\P112\\M11\\J00 Once when light was the only inhabitant of the multi-verse its sorrow gave birth to a great darkness within it. One day it took this darkness and buried it in the vast beyond, on doing so it returned to joy. However after several centuries had passed, the light realised that its work had not deminished with the passing time and it posited that it was no longer alone.";

  char test_msg_text4[] = "\\D1\\C1\\P152\\M11\\J00A555DEFGHIJKLMNOPQRSTUVWXYZ-abcdefghijklmnopqrstuvwxyz+1234567890++0987654321__ Once when light was the only inhabitant of the multi-verse its so555";
  char test_msg_dan[] = "\\S1\\C1\\P112\\M11\\J00The next stop is Cogan Thank you. Neges Teithioân Saffach. Travel Safer message.";
  char test_msg_danv2[] = "\\S1\\C1\\P112\\M11\\J00The next stop is Cogan %d Thank you. Neges Teithioân Saffach. Travel Safer message.";

  char test_msg_danv3[] = "\\D1\\C2\\P96\\M11\\J00Cogan one %d.";
  char test_msg_danv4[] = "\\D1\\C2\\P96\\M11\\J00Cogan two %d.";
  char test_msg_danv5[] = "\\R1\\Module four %d";
  char test_msg_danv6[];

  // Define states for transmission status
  typedef enum {
      INITIAL_STATE,            // Initial state for module_x
      READY_FOR_TRANSMISSION,   // Ready to transmit after detecting correct module
      TRANSMISSION_DONE,        // Transmission completed
      WAITING_FOR_PREV_MODULE,  // Ensuring correct order before proceeding
      MONITOR_ORDER_MODULES     // Monitoring previous module before re-entering READY
  } TransmissionState;

  TransmissionState transmission_state = INITIAL_STATE;
  uint32_t transmission_timestamp = 0;
  uint32_t transmission_delay = 1000; // 500 ms delay before transmission
  uint32_t lastRX_time = 0;

  const int module_x = 4;  // Define the module number
  uint8_t next;
  uint8_t following_mod_x;
  volatile bool packetReady_2Dec = false;
  volatile bool is_spi_receiving = false;



  int confirmationReceived = 1;  // Global flag for confirmation status
//Packet confirmation results
  static int result0CountPC = 0;  // Count of successfully transmitted packets (Result 0)
  static int result1CountPC = 0;  // Count of channel busy packets (Result 1)
  static int result2CountPC = 0;  // Count of transmit error packets (Result 2)
//Packet indication results
  static int validPacketCount = 0;
  static int crcErrorCount = 0;
  static int framingErrorCount = 0;
  static int parityErrorCount = 0;
  static int receiveTimeoutCount = 0;
  static int signalLossCount = 0;
  static int transmitBreakCount = 0;

  static int loss_trans_count = 0;


  volatile uint32_t buttonPressCount = 0;
  volatile uint8_t isReceivingData = 0;

  static sSpiBuf frame_list[MAX_FRAME_LIST_SIZE];
  static int frame_index=0;
  static volatile uint32_t int_ctr;
  int global_last_val_slave =0;



uint8_t Buffer_RX_DMA[150];
uint8_t Buffer_Src[]={0,1,2,3,4,5,6,7,8,9};
uint8_t Buffer_Dest[10];
uint32_t tick_count = 0;

typedef enum {
    MASTER_MODE,
    SLAVE_MODE
} Mode;

Mode currentMode = SLAVE_MODE;  // Default mode is slave

GPIO_PinState currentPinState;
GPIO_PinState lastPinState = GPIO_PIN_RESET; // Start with pin low
uint32_t debounceDelay = 10; // Debounce delay in milliseconds

uint32_t timer_counter;
uint8_t operationStarted = 0; // Flag to indicate if operation has started
volatile uint8_t start_module = 0;//Indicate operation of module
volatile uint8_t timerFlag = 0;
uint8_t transmitted_times = 0;
uint8_t spi_rx_buffer;
volatile uint8_t result_dec = 254;
uint8_t previous_result_dec = 2;
char searchmod1[] ="one";
char searchmod2[] ="two";
char searchmod3[] ="three";
char searchmod4[] ="four";
int twoFound = 0;
#define TIMEOUT_MS 5000 // Timeout value in milliseconds
#define TIMEOUT_MT 3000// Timeout missed transmission

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


  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);






  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  //Init_Device();
  /*
  uint8_t txData[] = {0xAA, 0xBB, 0xCC, 0xDD};
  uint8_t rxData[4];
  HAL_SPI_Transmit(&hspi2, (uint8_t *)txData, strlen(msgStatus), HAL_MAX_DELAY);

*/



 P2P_Init();
 //HAL_TIM_Base_Start_IT(&htim3);

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	// HAL_SPI_Receive_IT(&hspi2, &spi_rx_buffer, 1);
	//HAL_TIM_Base_Start_IT(&htim3);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	 // HAL_Delay(500);//Blink twice
	 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	 // HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  //HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);  // Second highest priority for SPI
 // HAL_NVIC_EnableIRQ(SPI2_IRQn);

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Timer_GPIO_Port, Timer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Module_1_Pin|Module_2_Pin|Module_4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Module_3_GPIO_Port, Module_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Timer_Pin */
  GPIO_InitStruct.Pin = Timer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Timer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HostReq_Pin */
  GPIO_InitStruct.Pin = HostReq_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HostReq_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Module_1_Pin Module_2_Pin Module_4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Module_1_Pin|Module_2_Pin|Module_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Module_3_Pin */
  GPIO_InitStruct.Pin = Module_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Module_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Entry_Pin */
  GPIO_InitStruct.Pin = Entry_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Entry_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SS_Pin */
  GPIO_InitStruct.Pin = SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//! Calculate the 16-bit CRC value for a message in a buffer
uint16_t calculate_crc16(const unsigned char* const message, const unsigned int messageLength)
{
	// Initialise the return value.
	uint16_t crc_val = -1;

	unsigned int index= 0;
	while(index < messageLength)
	{
		const unsigned char data_i = message[index];
		const unsigned crc_table_index = ((crc_val >> (sCRC_WIDTH - sCRC_BYTE_WIDTH)) ^ data_i) & sCRC_MASK;
		const unsigned short crc_table_entry = sCRC_TABLE[crc_table_index];
		crc_val = crc_table_entry ^ (crc_val << sCRC_BYTE_WIDTH);
		index++;
	}

	return crc_val;
}

static int get_header_length(uint8_t *text)
{
	if ((text == NULL) || (text[0]== '\0') || (text[0] != '\\'))
	{
		return 0;
	}

	int i, len, idx_mark, last_i;
	int header_length;

	len= strlen(text);
	idx_mark= 0;
	header_length- 0;

printf("  header = '");
	for (i=0; i<len; i++)
	{
		if (text[i] == '\\')
		{
			idx_mark= i;
		}
		else if ((i - idx_mark) > 4)
		{
			header_length= i;  // NB: ignore current character
			break;
		}
		else if (((i - idx_mark) > 1) && (isdigit(text[i]) == FALSE))
		{
			header_length= i;  // NB: ignore current character
			break;
		}
printf("%c", text[i]);

		last_i= i;
	}
printf("'\n");

	if (((idx_mark + 1) >= len) || ((last_i + 1) >= len))
	{
		return len;
	}

	return header_length;
}


static int build_message_frames(uint8_t* test_text, sSpiBuf* frames, int max_frames, uint8_t output_type)
{
	int count= 0;
	sSpiBuf send_buffer;
	int full_text_len;
	int header_len;
	int str_len;
	int i, floop, offset, space_for_frame;
	uint16_t crc;
	int num_frames, frame;
	uint8_t buffer[sPOWERLINE_FRAME_SIZE];

	sprintf(msgStatus,"Building message, transType= %d, target-output-type= %d\r\n", TT_DOWN_MSG, output_type);
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	if ((test_text == NULL) || (test_text[0] == '\0') || (frames == NULL))
	{
		sprintf(msgStatus,"Invalid power comms message.\r\n");
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);


		return -1;
	}

	offset= 0;
	full_text_len= strlen(test_text);
	header_len= get_header_length(test_text);

	sprintf(msgStatus,"  test text = '%s'\r\n", test_text);
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	sprintf(msgStatus,"  length of text = %d\r\n", full_text_len);
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	//sprintf(msgStatus,"  header length = %d\r\n", header_len);
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	if (output_type == FORWARDFACING)
	{   // NB: I found forward facing display mis-behaved if the full 'sPOWERLINE_FRAME_SIZE' bytes (i.e. 186 bytes) where used, '181' seemed the point at which it started fail

		// clip the string for the display's maximum length
		if ((full_text_len - header_len) > MAX_FORWARD_DISPLAY_TEXT_LENGTH)
		{
			full_text_len= (MAX_FORWARD_DISPLAY_TEXT_LENGTH + header_len);

			sprintf(msgStatus,"Error, text length is too long, clipping its length to %d.\r\n", full_text_len);
			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		}

		// determine how much space is available in the frame for text
		space_for_frame = (181 - 2); // remove space for CRC, as this is always needed
	}
	else
	{
		// clip the string for the display's maximum length
		if ((full_text_len - header_len) > MAX_SALOON_DISPLAY_TEXT_LENGTH)
		{
			full_text_len= (MAX_SALOON_DISPLAY_TEXT_LENGTH + header_len);

			sprintf(msgStatus,"Error, text length is too long, clipping its length to %d.\r\n", full_text_len);
			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		}

		// determine how much space is available in the frame for text
		space_for_frame = (sPOWERLINE_FRAME_SIZE - 2); // remove space for CRC, as this is always needed
	}
	num_frames= ((full_text_len / sPOWERLINE_FRAME_SIZE) + 1);
	num_frames= (((full_text_len + (num_frames * 12)) / sPOWERLINE_FRAME_SIZE) + 1);     // take into account space required each frame header, (NB: the frame header is assumed to be '12' characters in length)
	frame= 0;

	// ensure there is enough space to store the frames needed for the message
	if (num_frames > max_frames)
	{
		sprintf(msgStatus,"Error message requires to many frames, (%d). \n", num_frames);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		return -2;
	}

	// build up the frames needed for the message
	for (floop=0; floop < num_frames; floop++)
	{
		int _len;

		memset (send_buffer.data, (char)65, sizeof(send_buffer.data));
		memset (buffer, '\0', sizeof(buffer));
		frame= floop;

		// set frame's transaction type
		send_buffer.transType = TT_DOWN_MSG;


		// build frame's message data,
		//   NB: the first byte of the message is a binary value and is not part of the CRC
		send_buffer.data[0]=(uint8_t)output_type;
//tr0;//remove this
		sprintf(&send_buffer.data[1], "\\T%03d", (uint16_t)counterPacket);
		//sprintf(&send_buffer.data[1], "\\T%03u\\X%01u\\F%01u", (uint16_t)trans_id, (uint16_t)num_frames, (uint16_t)frame);
		sprintf(msgStatus, "el mensaje es %s \r\n.", &send_buffer.data[1]);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		_len= strlen(&send_buffer.data[1]) + 1;
		for (i = 0; i < (space_for_frame - _len); ++i)
		{
			if (offset < full_text_len)
			{
				buffer[i]= test_text[offset];
			}
			else
			{
				break;
			}

			offset++;
		}
		sprintf(&send_buffer.data[_len], "%s", buffer);
		str_len= strlen(&send_buffer.data[1]) + 1;

		// set the frame's message length
		send_buffer.transLen = (str_len + 2) + 1;    // NB: this '+ 1' is needed to last byte of CRC

		sprintf(msgStatus, "  frame = %d\r\n", frame);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		sprintf(msgStatus,"   transaction id = %d\r\n", trans_id);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		sprintf(msgStatus,"   buffer string = '%s'\r\n", send_buffer.data);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		sprintf(msgStatus,"   length = %d\r\n", send_buffer.transLen);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		// calculate and store the message's CRC
		//    NB: the first byte is *not* part of the CRC calculation, in these messages
		crc= calculate_crc16(&send_buffer.data[1], (str_len-1));
		send_buffer.data[str_len]=(uint8_t)(0xFF & (crc >> 8));
		send_buffer.data[str_len+1]=(uint8_t)(0xFF & crc);

		sprintf(msgStatus,"   CRC = %d (0x%04X)\r\n", crc, crc);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		if ((str_len + 2) > sPOWERLINE_TRANSFER_LIMIT)
		{
			printf("   Frame %u is too long, at %u bytes.\n", frame, (str_len + 2));
		}

		// copy the message in to the frames list
		memcpy(&frames[count], &send_buffer, sizeof(send_buffer));
		count++;
		if (count > max_frames)
		{
			break;
		}
	}


	// advance the transaction id
	trans_id++;
	if (trans_id == 255) {   // wrap around 'trans_id' early, as display's can't handle the value
		trans_id= 1;
	}

	return count;//count send_buffer.data
}


uint8_t* send_custom_text(uint8_t *test_msg) {
	int num_frames;
	int i;
	//int total_length;
	uint8_t *text;
	sSpiBuf send_buffer;



	if (test_msg[0] == '\\') {  // user string has escape sequence at start
		text= test_msg;
	}else {  // no escape sequence at start of user string

		sprintf(buffer_1k, "\\D1\\C1\\P149\\M11\\J00%s", test_msg);
		//HAL_UART_Transmit(&huart2, (uint8_t *)buffer_1k, strlen(buffer_1k), HAL_MAX_DELAY);

		text= &buffer_1k[0];
	}
	num_frames= build_message_frames(text, frame_list, MAX_FRAME_LIST_SIZE, FORWARDFACING);//SALOON FORWARDFACING

	sprintf(msgStatus, "\r\n Number of frames %d \r\n",num_frames);
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	//add headers
	uint8_t echecomm_header[8]= {0x32, 0xA1, 0xBA, 0x00, 0x49, 0x53, 0x49, 0x01};
	uint8_t semitechcomm_header[18]= {0x3C, 0x98, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t end_semih[2]= {0x00,0x3E};

	//sprintf(buffer_1k, "%s \r\n", text);
	//HAL_UART_Transmit(&huart2, (uint8_t *)buffer_1k, strlen(buffer_1k), HAL_MAX_DELAY);
	// Calculate the total message length (headers + text)

	uint8_t *new_text_ptr=NULL;
	int new_text_len=0;

	for (i=0; i<num_frames; i++) {
		new_text_ptr=frame_list[i].data;
		new_text_len=frame_list[i].transLen-1;
		total_length = sizeof(echecomm_header) + sizeof(semitechcomm_header) + new_text_len + sizeof(end_semih);
		sprintf(msgStatus, "NEW text \n");
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		sprintf(msgStatus, "\r\n %s \r\n", new_text_ptr);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, new_text_len, HAL_MAX_DELAY);
		sprintf(msgStatus, " \r\n Str len %02X \r\n ",new_text_len);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		// Create a buffer with enough space for the complete message
		//uint8_t combined_message[total_length]; this was made global_last_val

		//sprintf(msgStatus, " \r\n Str len %04X \r\n ",strlen(text));
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		//Length for semitech header
		uint8_t calculated_length_phy = sizeof(echecomm_header)+new_text_len;
		uint8_t total_calculated_length = calculated_length_phy + 12;

		sprintf(msgStatus, " \r\n Calculated Length phy = %d, total = %d \r\n ",calculated_length_phy, total_calculated_length);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		// add the modified headers

		semitechcomm_header[5] = total_calculated_length;
		semitechcomm_header[11] = calculated_length_phy;

		// Copy headers into the combined_message buffer
		memcpy(combined_message, semitechcomm_header, sizeof(semitechcomm_header));
		memcpy(combined_message + sizeof(semitechcomm_header), echecomm_header, sizeof(echecomm_header));

		// Copy the text into the combined_message buffer
		memcpy(combined_message + sizeof(echecomm_header) + sizeof(semitechcomm_header), new_text_ptr, new_text_len);

		//Inverted code
		for (int j = 1; j < total_length - 1; j++) {
			if (combined_message[j] == 0x3C || combined_message[j] == 0x3D || combined_message[j] == 0x3E) {
				// Update the total length to account for the additional character
				total_length ++;
				new_text_len ++;
				//shift the string to the right
				for (int k = total_length - 1; k > j + 2; k--) {
					//sprintf(msgStatus, " \r\n K = %d Old value = %02X New value = %02X \r\n ",k, combined_message[k],combined_message[k-1]);
					//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
					combined_message[k] = combined_message[k - 1];
				}
				// Escape the special characters
				combined_message[j+1] = ~combined_message[j];
				combined_message[j] = 0x3D;

				//combined_message[j+1] = ~combined_message[j];
				// Increase j to skip the substituted part and avoid processing it again
				j++;
				sprintf(msgStatus, " \r\n New string if 3X \r\n ");
				//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				for (i=0; i<total_length; i++) {
					sprintf(msgStatus, "%02X",combined_message[i]);
					//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				}
				sprintf(msgStatus, " \r\n %d END \r\n ", total_length);
				//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
			}
		}

		//Add checksum into the string
		uint8_t checksum = checksum_packet(combined_message,total_length); //total_length-2
		//uint8_t checksum2 = checksum_packet(packetData,sizeof(packetData)-2);
		sprintf(msgStatus, " \r\nChecksum %02X \r\n ",checksum);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		// Check if checksum needs escaping
		if (checksum == 0x3C || checksum == 0x3D || checksum == 0x3E) {
			// Update total length to accommodate escape marker and escaped checksum
			total_length += 1;

			// Append escape marker and escaped checksum
			combined_message[total_length - 3] = 0x3D;        // Escape marker
			combined_message[total_length - 2] = ~checksum;   // Escaped checksum value
		} else {
			// Append checksum normally
			//total_length++;
			combined_message[total_length - 2] = checksum;    // Add checksum before the delimiter
		}

		// Append the final delimiter (0x3C)
		combined_message[total_length - 1] = 0x3E;

		//sprintf(msgStatus, " \r\nChecksum2 %02X \r\n ",checksum2); //Checksum print
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		sprintf(msgOut, "Sending \r\n  ");
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		sprintf(msgStatus, "  \r\n Starts of message master \r\n ");
		// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		for (i=0; i<total_length; i++) {
			sprintf(msgStatus, "%02X",combined_message[i]);
			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		}
		sprintf(msgStatus, " \r\n %d END \r\n ", total_length);
		// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		for (int i=0; i<total_length; i++) {
			sprintf(msgStatus,"%c", combined_message[i]);
			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		}

        //Transmit string here via UART
		//  HAL_UART_Transmit(&huart1, (uint8_t*)combined_message, total_length, 100);

		/*
            Received RX UART
            This code processes received UART data, extracts a packet, transmits information via UART, and responds based on a condition.

            sprintf(msgStatus, " \r\n Received RX UART \r\n ");
            HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

            //Look for a packet

       if (temp[0] == '>'){
        memcpy (FinalData, RxData, indx);
        indx = 0;
        packet_count++;
        temp[0]='\0';
        }

        sprintf(msgOut,"\r\n Callback number %d\r\n",int_ctr);
        HAL_UART_Transmit(&huart2, (uint8_t *)msgOut, strlen(msgOut), HAL_MAX_DELAY);

        sprintf(msgStatus,"\r\n The string is: \r\n ");
        HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

        for (int i=0;i<sizeof(FinalData);i++){
            sprintf(msgStatus,"%c", FinalData[i]);
            HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
        }

        sprintf(msgStatus, "\n\r");
            HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

            decode_finalData(FinalData);

            if (echealon_payload) {
                sprintf(msgStatus,"The condition is met (flag is true).\n");
                HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
                char slave_reply[] = "\\I have received %d.";

                sprintf(msgStatus,"\r\n The Message Sent back to master is \n\r");
                  HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

                  sprintf(buffer_slave_it,slave_reply,global_last_val);
                  uint8_t* r2ss = send_custom_text(buffer_slave_it);

            } else {
                sprintf(msgStatus,"The condition is not met (flag is false).\n");
                 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
                 }
            echealon_payload = FALSE;
            memset(FinalData,'\0',sizeof(FinalData));*/

		// Print the total packet count
		//sprintf(msgStatus, "\n \r Packet number %d \n \r", packet_count);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		/*

            This code is attempting to extract and process packets from the rx_message buffer. It identifies packets based on < (start) and > (end) delimiters and transmits them via UART.

            int packet_count = 0;
            int rbp=0;
            uint8_t rxin_buf[256]; //temporary buffer
            bool rbpkt = FALSE;
            for (int i = 0; i<sizeof(rx_message); i++){
                if(rx_message[i] == '<'){
                    rbpkt = TRUE; //indicate beginning of packet
                }
                if(rbpkt){
                 memset(rxin_buf,'\0',sizeof(rxin_buf));
                 rxin_buf[rbp] = rx_message[i];
                 if (rx_message[i]=='>'){
                    rbpkt = FALSE;
                    packet_count++;

                    // Process the complete packet stored in rxin_buf
                    sprintf(msgStatus, "Complete packet received: ");
                  HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

                  for (int j = 0; j <= rbp; j++) {
                      sprintf(msgStatus, "%c", rxin_buf[j]);
                      HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
                      }

                  sprintf(msgStatus, "\n\r");
                  HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

                }
                if (rbp < sizeof(rxin_buf) - 1) {
                    rbp++;
                }
               }
            }
            sprintf(msgStatus,"\n \r Packet number %d \n \r", packet_count);
            HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);*/

		//This is the message that I´ll transmit
		/*sprintf(msgStatus,"\n \r Packet to transmit %d \n \r");
		HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		for (i=0; i<sizeof(combined_message); i++) {
			sprintf(msgStatus, "%02X",combined_message[i]);
			HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		}
		sprintf(msgStatus," \r\n the length is %d \r\n",sizeof(combined_message));
		HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);*/

	}
	return (uint8_t*)combined_message;
	memset(combined_message,'\0',sizeof(combined_message));
}



uint8_t checksum_packet(uint8_t *packet, int packet_length){
	uint32_t checksum = 0;
	uint32_t checksum_total = 0;

	int i;

	checksum = 0;

	for (i=0; i<packet_length; i++){
		if (packet[i] == 0x3C || packet[i] == 0x3E || packet[i] == 0x3D){
			checksum = 0x00;
		}
		else if (packet[i-1] != 0x3D){
			checksum = packet[i];
		}
		else {
				checksum = packet[i]^255;
		}
		checksum_total += checksum;

		//checksum += packet[i];
		//sprintf(msgStatus,"\r\n Checksum value used = %d packet = %02X Checksum = %02X Checksum_DEC = %d \r\n",i ,packet[i],checksum,checksum);
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	}

	sprintf(msgStatus,"\r\n Checksum fun Packet Len= %d \r\n",packet_length);
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	checksum =256-(checksum_total % 256);//checksum = 256-(checksum % 256);


	return checksum;

}


//Function to select the state of the device



/*
void received_custom_text(uint8_t *custom_message){

    //int count_find = 0;
	memset(custom_message,'\0',sizeof(custom_message));


#ifdef DEBUG_RX
	 sprintf(msgStatus, " \r\n Received  ");
	 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif
	 //HAL_UART_Receive(&huart1, (uint8_t *)custom_message, sizeof(msgIn), gap_time/2);// 62UART Received custom_message
	 //msgIn[sizeof(msgIn) - 1] = '\0';
	 //HAL_UART_Receive_IT(&huart1, (uint8_t *)msgIn, sizeof(msgIn));

	//reception_complete = 0;
	//HAL_UART_Transmit(&huart1, TX_String, sizeof(TX_String),10);

		// wait for reception to be completed
		//while (reception_complete ==0) {}



	 for (int i = 0 ; i < sizeof(custom_message); i++){
		 sprintf( msgStatus,"%02X",custom_message[i]);
		 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		 }
	 sprintf(msgStatus, "\r\n  ");
	 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	 for (int i=0; i < sizeof(custom_message); i++){
		 msgStatus[i]=0;
		 }

	 sprintf(msgStatus,"\r\n The string is: \r\n ");
	 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	 for (int i=0;i<sizeof(custom_message);i++){
		 sprintf(msgStatus,"%c", custom_message[i]);
		 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		 }
	 sprintf(msgStatus, "\n\r");
	 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);


	 /*
	  * This is Code for Decode the string
	  *
	  * /

	 uint8_t test_msg_data[] = { 0x3C, 0x3C, 0x98, 0x01, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x62, 0x3E, 0x3C, 0x98, 0x01, 0x10, 0x00, 0x32,
	                              0x1F, 0xFF, 0x00, 0x00, 0x18, 0x12, 0x00, 0x2A, 0x32, 0xA3, 0xBA, 0x00, 0x49, 0x53, 0x49, 0x01, 0x01, 0x5C, 0x54, 0x32, 0x32,
	                              0x35, 0x5C, 0x58, 0x31, 0x5C, 0x46, 0x30, 0x5C, 0x49, 0x20, 0x68, 0x61, 0x76, 0x65, 0x20, 0x72, 0x65, 0x63, 0x65, 0x69,
	                              0x76, 0x65, 0x64, 0x20, 0x2D, 0x31, 0x2E, 0x13, 0xC2, 0xEC, 0x3E };

	  power_line_data_t global_msg_data;
	  uint8_t global_decoded_payload[256];
	  //char decoded_payload;

	  unsigned char *ptr1;
	  int bytes_read;
	  int total_bytes_read;
	  int buffer_size;

	  total_bytes_read= 0;
	  buffer_size=sizeof(msgIn);// sizeof(msgIn);//i change msgIn for test_msg_data62 FinalData
	  do
	  {// chnage &msgIn for &test_msg_data below
	    ptr1= decode_powerline((char *)&msgIn[total_bytes_read], (buffer_size-total_bytes_read) , &global_msg_data, (char*)global_decoded_payload, &bytes_read);
	    if (ptr1 != NULL) // check if call to function and message buffer was valid
	    {
	      if (global_msg_data.msg_len > -1)  // check if complete powerline message found
	      {  // complete powerline message found
	sprintf(msgStatus,"\rPWRLine data: semiTechLen=%d, echelonLen=%d, semitech_command=0x%04X, checksum=0x%02X\n Semi-tech header: \n\r", global_msg_data.msg_len, global_msg_data.echelon_msg_len, global_msg_data.semitech_command, global_msg_data.checksum);
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	        int n;

	        // display semi-tech header
	        n= sizeof(global_msg_data.semitech_header);
	        for(int i=0; i<n; i++)
	        {
	          sprintf(msgStatus,"%02X,",global_msg_data.semitech_header[i]);
	          //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	        }

	        // dispaly any Echelon sub-message data
	        sprintf(msgStatus,"\r\nEchelon header: \n\r");
	        //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	        if (global_msg_data.echelon_msg_len > 0)
	        {
	          n= sizeof(global_msg_data.echelon_header);
	          for(int i=0; i<n; i++)
	          {
	            sprintf(msgStatus,"%02X,",global_msg_data.echelon_header[i]);
	          // HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          }
	          sprintf(msgStatus,"\r\nEchelon payload: \n\r");
	          //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          n= global_msg_data.echelon_msg_len - 8;  // NB: remove size of Echelon header
	          echealon_payload = TRUE;
	          for(int i=0; i<n; i++)
	          {
	            sprintf(msgStatus,"%02X,",global_decoded_payload[i]);
	           //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          }

	          memcpy(decoded_payload,global_decoded_payload,strlen(global_decoded_payload));
	          sprintf(msgStatus,"\n\rString that I copied \n\r ");
	          //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          for(int i=0; i<n; i++)

	        	  {
	        	  sprintf(msgStatus,"%02X,",decoded_payload[i]);
	        	  //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	        	  }

	          char* decoded_strSample;
	          decoded_strSample = decoded_payload;//char_ptr; //"teacher teach tea";


	          char search[] = "Cogan";
	          char search1[]= "received";
	          char *ptr_str = strstr(decoded_strSample, search);
	          char *ptr_str1=strstr(decoded_strSample,search1);

	          if (ptr_str != NULL) // Substring found
	          	{
	        	  int position = ptr_str - decoded_strSample;
	          	sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, search, position);
	          	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          	result_Cogan= GetNumber(&decoded_strSample[position]);
	          	sprintf(msgStatus,"\r\n** The number in Cogan is %d **\r\n", result_Cogan);
	          	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

         		}
         		else // Substring not found
	          	{
	          	sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, search);
	          	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          	}


	          if (ptr_str1 != NULL) // Substring found
	          	{
	        	  int position1 = ptr_str1 - decoded_strSample;
	          	sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, search1, position1);
	          	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          	result_Received= GetNumber(&decoded_strSample[position1]);
	            sprintf(msgStatus,"\r\n** The number in received is %d **\r\n", result_Received);
	          	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         		}
         		else // Substring not found
	          	{
	          	sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, search1);
	          	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          	}

	        }
	        else if (global_msg_data.semitech_command == 0x0101)  // check for response from "Data Confirm" message
	        {

	          // display semi-tech payload
	          sprintf(msgStatus,"\rSemi-tech payload: \n\r");
	          //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          n= global_msg_data.msg_len;
	          for(int i=0; i<n; i++)
	          {
	            sprintf(msgStatus,"%02X,",global_decoded_payload[i]);
	            //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	          }
	        }
	       sprintf(msgStatus,"\n");
	      // HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	      }
	      else if (bytes_read > 0)  // check if unexpected start of powerline message encountered
	      {  // found unexpected message start
	sprintf(msgStatus,"\rDEBUG: unexpected start of message found \r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	      }

	      // mark the read bytes as consumed, i.e. move pased them in buffer
	      total_bytes_read+= bytes_read;

	      if (total_bytes_read < buffer_size)
	      {
	sprintf(msgStatus,"\rDEBUG: get next message in buffer\n\r");
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	      }

	    }
	    else
	    {  // message buffer invalid or reached end of it
	      break;
	    }
	  } while (total_bytes_read<buffer_size);
	  for (int i=0;i<sizeof(msgIn);i++){
		  sprintf(msgStatus,"%c", custom_message[i]);
		  HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		  }
	  custom_message[sizeof(msgIn)-1]='\0';
	  sprintf(msgStatus, "\n\r");
	  HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	  memset(msgIn,'\0',sizeof(msgIn));




	 }
*/

int decode_finalData(uint8_t * message ){

/*
 * This is Code for Decode the string
 *
 * */

uint8_t test_msg_data[] = { 0x3C, 0x3C, 0x98, 0x01, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x62, 0x3E, 0x3C, 0x98, 0x01, 0x10, 0x00, 0x32,
                             0x1F, 0xFF, 0x00, 0x00, 0x18, 0x12, 0x00, 0x2A, 0x32, 0xA3, 0xBA, 0x00, 0x49, 0x53, 0x49, 0x01, 0x01, 0x5C, 0x54, 0x32, 0x32,
                             0x35, 0x5C, 0x58, 0x31, 0x5C, 0x46, 0x30, 0x5C, 0x49, 0x20, 0x68, 0x61, 0x76, 0x65, 0x20, 0x72, 0x65, 0x63, 0x65, 0x69,
                             0x76, 0x65, 0x64, 0x20, 0x2D, 0x31, 0x2E, 0x13, 0xC2, 0xEC, 0x3E };

 power_line_data_t global_msg_data;
 uint8_t global_decoded_payload[150];
 //char decoded_payload;

 unsigned char *ptr1;
 int bytes_read;
 int total_bytes_read;
 int buffer_size;
 int found_value = 0;

 total_bytes_read= 0;
 buffer_size=sizeof(FinalData);// sizeof(msgIn);//i change msgIn for test_msg_data62 FinalData
 do
 {// chnage &msgIn for &test_msg_data below
   ptr1= decode_powerline((char *)&FinalData[total_bytes_read], (buffer_size-total_bytes_read) , &global_msg_data, (char*)global_decoded_payload, &bytes_read);
   if (ptr1 != NULL) // check if call to function and message buffer was valid
   {
     if (global_msg_data.msg_len > -1)  // check if complete powerline message found
     {  // complete powerline message found
sprintf(msgStatus,"\rPWRLine data: semiTechLen=%d, echelonLen=%d, semitech_command=0x%04X, checksum=0x%02X\n Semi-tech header: \n\r", global_msg_data.msg_len, global_msg_data.echelon_msg_len, global_msg_data.semitech_command, global_msg_data.checksum);
//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

       int n;

       // display semi-tech header
       n= sizeof(global_msg_data.semitech_header);
       for(int i=0; i<n; i++)
       {
         sprintf(msgStatus,"%02X,",global_msg_data.semitech_header[i]);
         //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
       }

       // dispaly any Echelon sub-message data
       sprintf(msgStatus,"\r\nEchelon header: \n\r");
       //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
       if (global_msg_data.echelon_msg_len > 0)
       {
         n= sizeof(global_msg_data.echelon_header);
         for(int i=0; i<n; i++)
         {
           sprintf(msgStatus,"%02X,",global_msg_data.echelon_header[i]);
          //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         }
         sprintf(msgStatus,"\r\nEchelon payload: \n\r");
         //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         n= global_msg_data.echelon_msg_len - 8;  // NB: remove size of Echelon header
         echealon_payload = TRUE;
         for(int i=0; i<n; i++)
         {
           sprintf(msgStatus,"%02X,",global_decoded_payload[i]);
          //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         }

         //memcpy(decoded_payload,global_decoded_payload,strlen(global_decoded_payload));
         memcpy(decoded_payload,global_decoded_payload,sizeof(global_decoded_payload));
         sprintf(msgStatus,"\n\rString that I copied \n\r ");
         //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         for(int i=0; i<n; i++)

       	  {
       	  sprintf(msgStatus,"%02X,",decoded_payload[i]);
       	  //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
       	  }

         char* decoded_strSample;
         decoded_strSample = decoded_payload;//char_ptr; //"teacher teach tea";
         sprintf(msgStatus,"Decoded string %s \n\r",decoded_strSample);
         //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);


         //char search[] = "Cogan";
         //char search1[]= "received";
         char searchmod1[] ="one";
         char searchmod2[] ="two";
         char searchmod3[] ="three";
         char searchmod4[] ="four";
         //char *ptr_str = strstr(decoded_strSample, search);
         //char *ptr_str1=strstr(decoded_strSample,search1);
         char *ptr_strmod1=strstr(decoded_strSample,searchmod1);
         char *ptr_strmod2=strstr(decoded_strSample,searchmod2);
         char *ptr_strmod3=strstr(decoded_strSample,searchmod3);
         char *ptr_strmod4=strstr(decoded_strSample,searchmod4);
/*
         if (ptr_str != NULL) // Substring found
         	{
       	  int position = ptr_str - decoded_strSample;
         	sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, search, position);
         	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         	result_Cogan= GetNumber(&decoded_strSample[position]);
         	sprintf(msgStatus,"\r\n** The number in Cogan is %d **\r\n", result_Cogan);
         	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

    		}
    		else // Substring not found
         	{
         	sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, search);
         	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         	}


         if (ptr_str1 != NULL) // Substring found
         	{
       	  int position1 = ptr_str1 - decoded_strSample;
         	sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, search1, position1);
         	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         	result_Received= GetNumber(&decoded_strSample[position1]);
           sprintf(msgStatus,"\r\n** The number in received is %d **\r\n", result_Received);
         	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    		}
    		else // Substring not found
         	{
         	sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, search1);
         	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         	}*/
         if (ptr_strmod1 != NULL) // Substring found
            {
        	 int position = ptr_strmod1 - decoded_strSample;
        	 sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, searchmod1, position);
        	 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
        	 //result_Cogan= GetNumber(&decoded_strSample[position]);
        	 found_value = 1;
        	 mod1_count++;
        	 end_time = HAL_GetTick();  // Capture current tick value
        	 elapsed_time = end_time - start_time;  // Calculate elapsed time in ms

        	 sprintf(msgStatus,"** The number of mod one found is %d **\r\n", mod1_count);
        	 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
        	 sprintf(msgStatus,"The elapsed time is %d ms **\r\n", elapsed_time);
        	// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);


        	 }
            else // Substring not found
            {
            sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, searchmod1);
          //  HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
            }

         if (ptr_strmod2 != NULL) // Substring found
         {
        	 int position = ptr_strmod2 - decoded_strSample;
        	 sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, searchmod2, position);
        	 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

        	 //result_Cogan= GetNumber(&decoded_strSample[position]);
        	 found_value = 2;
        	 mod2_count++;
        	 end_time = HAL_GetTick();  // Capture current tick value
        	        	 elapsed_time = end_time - start_time;  // Calculate elapsed time in ms
        	 sprintf(msgStatus,"** The number of mod two found is %d **\r\n", mod2_count);
        	// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
        	 sprintf(msgStatus,"The elapsed time is %d ms **\r\n", elapsed_time);
        	  //      	 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         }
         else // Substring not found
         {
        	 sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, searchmod2);
        	// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         }
         if (ptr_strmod3 != NULL) // Substring found
         {
        	 int position = ptr_strmod3 - decoded_strSample;
        	 sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, searchmod3, position);
        	 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
        	 //result_Cogan= GetNumber(&decoded_strSample[position]);
        	 found_value = 3;
        	 mod3_count++;
        	 end_time = HAL_GetTick();  // Capture current tick value
        	         	        	 elapsed_time = end_time - start_time;  // Calculate elapsed time in ms
        	 sprintf(msgStatus,"** The number of mod three found is %d **\r\n", mod3_count);
        	// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
        	 sprintf(msgStatus,"The elapsed time is %d ms **\r\n", elapsed_time);
        	    //    	 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         }
         else // Substring not found
         {
        	 sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, searchmod3);
        	// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         }
         if (ptr_strmod4 != NULL) // Substring found
                  {
                 	 int position = ptr_strmod4 - decoded_strSample;
                 	 sprintf(msgStatus,"'%s' contains '%s' and is in %d \r\n", decoded_strSample, searchmod4, position);
                 	 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
                 	 //result_Cogan= GetNumber(&decoded_strSample[position]);
                 	found_value = 4;
                 	 mod4_count++;
                 	 end_time = HAL_GetTick();  // Capture current tick value
                 	 elapsed_time = end_time - start_time;  // Calculate elapsed time in ms
                 	 sprintf(msgStatus,"** The number of mod four found is %d **\r\n", mod4_count);
                 	// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
                 	 sprintf(msgStatus,"The elapsed time is %d ms **\r\n", elapsed_time);
                 	 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
                  }
                  else // Substring not found
                  {
                 	 sprintf(msgStatus,"'%s' doesn't contain '%s'\r\n", decoded_strSample, searchmod4);
                 	// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
                  }


        /* // Create table header
             sprintf(msgStatus, "Module  | Count\r\n");
             HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

             sprintf(msgStatus, "--------|------\r\n");
             HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

             // Display mod1_count
             sprintf(msgStatus, "Mod 1   | %d\r\n", mod1_count);
             HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

             // Display mod2_count
             sprintf(msgStatus, "Mod 2   | %d\r\n", mod2_count);
             HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

             // Display mod3_count
             sprintf(msgStatus, "Mod 3   | %d\r\n", mod3_count);
             HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

             // Optionally, add a separator at the end
             sprintf(msgStatus, "--------|------\r\n");
             HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);*/

       }
       else if (global_msg_data.semitech_command == 0x0101)  // check for response from "Data Confirm" message
       {

         // display semi-tech payload
         sprintf(msgStatus,"\rSemi-tech payload: \n\r");
         //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         n= global_msg_data.msg_len;
         for(int i=0; i<n; i++)
         {
           sprintf(msgStatus,"%02X,",global_decoded_payload[i]);
          // HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
         }
       }
      sprintf(msgStatus,"\n");
     // HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
     }
     else if (bytes_read > 0)  // check if unexpected start of powerline message encountered
     {  // found unexpected message start
sprintf(msgStatus,"\rDEBUG: unexpected start of message found \r\n");
//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
     }

     // mark the read bytes as consumed, i.e. move pased them in buffer
     total_bytes_read+= bytes_read;

     if (total_bytes_read < buffer_size)
     {
sprintf(msgStatus,"\rDEBUG: get next message in buffer\n\r");
//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
     }

   }
   else
   {  // message buffer invalid or reached end of it
     break;
   }
 } while (total_bytes_read<buffer_size);
 for (int i=0;i<sizeof(FinalData);i++){
	  sprintf(msgStatus,"%c", message[i]);
	  //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	  }
 message[sizeof(FinalData)-1]='\0';
 sprintf(msgStatus, "\n\r");
 //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
 memset(FinalData,'\0',sizeof(FinalData));


 return found_value;

}

int GetNumber(const char *str) {
	   while (!(*str >= '0' && *str <= '9') && (*str != '-') && (*str != '+') && *str) str++;
	   int number;
	   if (sscanf(str, "%d", &number) == 1) {
		   return number;
	   }
	   // No int found
	     return -1;
	  }

char* my_strstr(char* str1, char* str2, int num_bytes){
    int i = 0;
    int len2= strlen(str2);

    if (( str2[0] == '\0' ) || (len2 > num_bytes))
      return NULL;


    for (i=0; i< ((num_bytes-len2)+1); i++)
    {
        int val= memcmp(&str1[i], str2, len2);

        if( val == 0 )
            return &str1[i];
    }

    return NULL;
}


char* decode_powerline(char* buffer, int buffer_length, power_line_data_t *pwl_data, char* payload, int *bytes_read)
{
   if (bytes_read != NULL)
     (*bytes_read)= 0;

   // validate parameters
   if ((buffer == NULL) || (buffer_length <= 1))
     return NULL;

#if defined(DEBUG_SEMLTCH_DECODE_BASIC) || defined(DEBUG_SEMLTCH_DECODE)
sprintf(msgStatus,"buffer_length=%d\n\r", buffer_length);
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

#endif

   uint8_t local_buffer[256];
   char* ptr;
   uint8_t found_start= FALSE;
   uint8_t found_end= FALSE;
   uint8_t found_esc= FALSE;
   uint8_t found_payload= FALSE;
   uint8_t found_checksum= FALSE;
   uint8_t unexpected_start= FALSE;
   int count_msg_bytes= 0;
   int total_msg_bytes= 0;

   // get message buffer start
   ptr= buffer;

   // iterate over message buffer looking for semi-tech powerline message
   for (int i=0; i<buffer_length; i++)
   {
#if defined(DEBUG_SEMLTCH_DECODE_BASIC) || defined(DEBUG_SEMLTCH_DECODE)
	   if (*ptr != 0){
		   sprintf(msgStatus,"%d: %c (0x%02X)\r\n",i, *ptr, ((*ptr) & 0xFF));
	   }else{
		   sprintf(msgStatus,"%d:  (0x%02X)\r\n",i, ((*ptr) & 0xFF));
	   }
	   	   //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);


#endif

     // check if found start of message
     if (found_start == FALSE)
     {  // start of message *not* found
#ifdef DEBUG_SEMLTCH_DECODE
sprintf(msgStatus," DEBUG: Hunting for start\n\r");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

       if (*ptr == PWL_START)
       {
#ifdef DEBUG_SEMLTCH_DECODE
sprintf(msgStatus," DEBUG: found start\n\r");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

         found_start= TRUE;
         local_buffer[0]= PWL_START;
         count_msg_bytes= 1;
         total_msg_bytes= 1;
       }
     }
     else
     {  // start of message found
       if (found_esc == FALSE)
       {  // *not* in charater escape mode
         if (*ptr == PWL_ESC)  // check for escape character
         {  // escape charater found
#ifdef DEBUG_SEMLTCH_DECODE
sprintf(msgStatus," DEBUG: found escape character\n\r");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

           found_esc= TRUE;
           total_msg_bytes++;
         }
         else if (*ptr == PWL_END)  // check for end of message
         {  // end of message
#ifdef DEBUG_SEMLTCH_DECODE
sprintf(msgStatus," DEBUG: found end, exiting\n\r");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

           found_end= TRUE;
           local_buffer[count_msg_bytes]= PWL_END;
           count_msg_bytes++;
           total_msg_bytes++;
           ptr++;
           break;
         }
         else if (*ptr == PWL_START)  // check for start of message
         {  // unexpected start of message
#ifdef DEBUG_SEMLTCH_DECODE
sprintf(msgStatus," DEBUG: unexpected start, exiting\n\r");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

           unexpected_start= TRUE;

           break;
         }
         else
         {  // message data
#ifdef DEBUG_SEMLTCH_DECODE
sprintf(msgStatus," DEBUG: general message data\n\r");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

           local_buffer[count_msg_bytes]= *ptr;
           count_msg_bytes++;
           total_msg_bytes++;
         }
       }
       else
       {  // character is escaped
#ifdef DEBUG_SEMLTCH_DECODE
sprintf(msgStatus," DEBUG: in escape mode, invert data\n\r");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

           found_esc= FALSE;

           local_buffer[count_msg_bytes]= ~(*ptr);
           count_msg_bytes++;
           total_msg_bytes++;
       }
     }

     // move message pointer to next character
     ptr++;
   }

#if defined(DEBUG_SEMLTCH_DECODE_BASIC) || defined(DEBUG_SEMLTCH_DECODE)
sprintf(msgStatus,"count_msg_bytes= %d, total_msg_bytes=%d, found_start=%d, found_end=%d\n MSG: \n\r",count_msg_bytes, total_msg_bytes, found_start,found_end);
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
for (int i=0;i<count_msg_bytes;i++)
{
  sprintf(msgStatus,"%02X,",local_buffer[i]);
  HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
}
sprintf(msgStatus,"\n");
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

   // set bytes read from buffer
   if (bytes_read != NULL)
     (*bytes_read)= total_msg_bytes;

   // check if complete message obtained
   if (found_end == TRUE)
   {  // found complete message
     int n, st;

     // popupate powerline message data object and payload array

     // initialise key return values
     pwl_data->msg_len= 0;
     pwl_data->echelon_msg_len= 0;
     pwl_data->semitech_command= 0xFFF;
     memset(pwl_data->semitech_header, '\0', sizeof(pwl_data->semitech_header));
     memset(pwl_data->echelon_header, '\0', sizeof(pwl_data->echelon_header));

     // get data from powerline message
     if (count_msg_bytes >=6)   // check if message data contains the semi-tech message length
     {
       // get the semi-tech message length
       pwl_data->msg_len= ((uint16_t)local_buffer[4] << 8) | (local_buffer[5]);

       pwl_data->semitech_command= ((uint16_t)local_buffer[2] << 8) | (local_buffer[3]);

       // copy the semi-tech header
       st=1;
       n= 5;   // default semi-tech header size
       if (pwl_data->semitech_command == 0x0110)   // check if message is the "received data packet indication"
         n=13;   // sets the header size for the semi-tech "received data packet indication" message
       memcpy(pwl_data->semitech_header,&local_buffer[st], n);

       // get data for Echelon messages
       if (count_msg_bytes >=14)   // check if message data contains the echelon message length
       {
         // get the echelon message length
         pwl_data->echelon_msg_len= ((uint16_t)local_buffer[12] << 8) | (local_buffer[13]);

         // copy the echelon header
         st= 14;
         n= 8;
         memcpy(pwl_data->echelon_header,&local_buffer[st], n);

         // get Echelon payload
         if (payload != NULL)
         {
           st= 22;
           n= pwl_data->echelon_msg_len - 8;   // NB: remove size of Echelon header
           if ( (st + (n-1)) > (count_msg_bytes - 2))
           {
#if defined(DEBUG_SEMLTCH_DECODE_BASIC) || defined(DEBUG_SEMLTCH_DECODE)
sprintf(msgStatus,"Limits %d > %d\n\r",(st + (n-1)), (count_msg_bytes - 2));
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
sprintf(msgStatus,"Echelon payload length too long, %d (%d)\n\r", n, ((count_msg_bytes - 2) - st));
HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
#endif

            n= (count_msg_bytes - 2) - st;
           }

           memcpy(payload, &local_buffer[st], n);
         }
       }
       else if (payload != NULL)
       {
         if (pwl_data->semitech_command == 0x0101)   // check if message is "Data Confirm"
         {  // copy the whole semi-tech payload as the "payload"
           st= 6;
           n= pwl_data->msg_len;
           memcpy(payload, &local_buffer[st], n);
         }
       }
     }

     // get the checksum
     if (count_msg_bytes >=3)
     {
       pwl_data->checksum= local_buffer[count_msg_bytes-2];
     }

     return ptr;
   }
   else if (unexpected_start == TRUE)   // check if an unexpected start message was found
   {  // unexpected start message found, (NB: throw away any data seen so far)
     pwl_data->msg_len= -1;
     pwl_data->echelon_msg_len= -1;
     pwl_data->checksum= 0;
     return ptr;
   }

   // message was not complete, (NB: throw away any data seen so far)
   pwl_data->msg_len= -1;
   pwl_data->echelon_msg_len= -1;
   pwl_data->checksum= 0;

   return NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if(huart->Instance == USART1){ /* Do everything in this block */

		++int_ctr;
		memcpy(RxData+indx, temp, 1);
		if (++indx >= 100 ) indx = 0;
		 start_time = HAL_GetTick();  // Capture current tick value
		HAL_UART_Receive_IT(&huart1, temp, 1);





	} else {/* Do nothing */

	}



}

//Transmit callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){


	sprintf(msgStatus,"Transmission finished\n\r");
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	//HAL_UART_Transmit_IT(&huart1,(uint8_t*)combined_message,total_length);
}



int UART1_Busy(void) {
    // Check if UART1 is busy
    if (__HAL_UART_GET_FLAG(&huart1, HAL_UART_STATE_BUSY_RX) != RESET) {
        return 1; // UART is busy
    } else {
        return 0; // UART is not busy
    }
}
//Transmit callback
/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	sprintf(msgStatus,"Transmission finished\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	//HAL_UART_Transmit_IT(&huart1,(uint8_t*)combined_message,total_length);
}*/
/*
}
*/

// HAL GPIO EXTI callback

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {

    	//if (!is_spi_receiving) {


    	        is_spi_receiving = true;
    	        sprintf(msgStatus, "\r\n Host request interruption \r\n");
    	        HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    	        // Set SS (PB6) Low to select the slave
    	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    	        //HAL_SPI_Receive_DMA(&hspi2, rxData_getProtocol, sizeof(rxData_getProtocol));
    	        //HAL_SPI_Receive_DMA(&hspi2, rxData_getProtocol, sizeof(rxData_getProtocol));//chnage dma to it
    	        //HAL_SPI_Receive_IT(&hspi2, rxData_getProtocol, sizeof(act_rx_packet)); //change of the size of packet
    	        HAL_SPI_Receive_DMA(&hspi2, rxData_getProtocol, sizeof(act_rx_packet));

    	//}
    }
}

// Callback function called when SPI reception is complete
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	static uint8_t packetBuffer[150];   // Buffer to hold individual packets
		    static int packetIndex = 0;        // Index for packet buffer
		    static bool inPacket = false;      // Flag to indicate if we are inside a packet
    if (hspi->Instance == SPI2) {

    	is_spi_receiving = false;
    	lastRX_time = HAL_GetTick();  // Update last reception time

    	sprintf(msgStatus, "\r\n SPI Rx Callback \r\n");
    	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

    	sprintf(msgStatus, "\r\n Received Packet RX: ");
    	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

    	for (int j = 0; j < sizeof(rxData_getProtocol); j++) {
    		sprintf(msgStatus, "%02X", rxData_getProtocol[j]);
    		HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    		}
    	sprintf(msgStatus, "\r\n");
    	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);



    	for (int i=0; i<sizeof(rxData_getProtocol); i++){
    		uint8_t byte = rxData_getProtocol[i];
    		if (byte == 0x3C) {
    			// Start of a new packet
    			inPacket = true;
    			packetIndex = 0; // Reset packet index
    		}if (inPacket) {
    		    if (packetIndex < sizeof(packetBuffer)) {  // Ensure we do NOT exceed buffer
    		        packetBuffer[packetIndex++] = byte;
    		    } else {
    		        // Optional: Handle buffer overflow (e.g., stop processing)
    		        inPacket = false;
    		        packetIndex = 0;
    		        sprintf(msgStatus, "\r\n Buffer Overflow Detected! Packet too large.\r\n");
    		        HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    		    }
    		}if (byte == 0x3E && inPacket) {
    			// End of packet detected, process the packet
    			// Check if packet contains at least the command position
    			if (packetIndex > 4 && packetBuffer[1] == 0x98) {
    				uint8_t command1 = packetBuffer[2];  // First byte of command
    				uint8_t command2 = packetBuffer[3];  // Second byte of command

    				// Check which command is present
    				if (command1 == 0x01 && command2 == 0x01) {
    					// Command 0101 - Transmit Confirmation
    					confirmationReceived = 1;  // Set confirmation flag
    					//sprintf(msgStatus, "\r\n Command: %02X%02X - Transmit Confirmation\r\n", command1, command2);
    					//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    					// Extract and check the result byte of packet confirm
    					uint8_t resultPC = packetBuffer[6];  // Result byte is at index 6 (after the length bytes)
    					switch (resultPC) {
    					    case 0x00:
    					        result0CountPC++;
    					        break;
    					    case 0x01:
    					        result1CountPC++;
    					        break;
    					    case 0x10:
    					        result2CountPC++;
    					        break;
    					    default:
    					        // Handle unexpected result values
    					    	sprintf(msgStatus, "\r\n Unexpected resultPC: %02X\r\n", resultPC);
    					    	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus),HAL_MAX_DELAY);
    					        break;
    					}

    				}else if (command1 == 0x01 && command2 == 0x10) {
    					sprintf(msgStatus, "\r\n Command: %02X%02X - RX Packet\r\n", command1, command2);
    					//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    					// Extract and check the result byte of packet indication
    					uint8_t resultPI = packetBuffer[12]; //Result byte is at index 13 (after SNR)

    					switch (resultPI){
    					case 0x00:
    						validPacketCount++;
    						break;
    					case 0x01:
    						crcErrorCount++;
    						break;
    					case 0x02:
    						framingErrorCount++;
    						break;
    					case 0x04:
    						parityErrorCount++;
    						break;
    					case 0x08:
    						receiveTimeoutCount++;
    						break;
    					case 0x10:
    						signalLossCount++;
    						break;
    					case 0x20:
    						transmitBreakCount++;
    						break;
    					default:
    						// Handle unexpected result values
    						sprintf(msgStatus, "\r\n Unexpected resultPI: %02X\r\n", resultPI);
    						HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    						break;
    					}


    					memcpy(FinalData, packetBuffer, sizeof(packetBuffer));//copy to buffer where it's going to be decoded
    					packetReady_2Dec = true; // Set flag for main loop processing
    					//result_dec = decode_finalData(FinalData);

    					//sprintf(msgStatus,"\r\n Result decoded en RX Callback %d \r\n",result_dec);
    					//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    					//Hasta aqui, se ha decodificado si la data viene del modulo uno, dos, etc

    	    			sprintf(msgStatus, "\r\n Received Packet: ");
    	    			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

    	    			for (int j = 0; j < packetIndex; j++) {
    	    				sprintf(msgStatus, "%02X", packetBuffer[j]);
    	    				//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    	    			}
    	    			sprintf(msgStatus, "\r\n");
    	    			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

    	    			memset(packetBuffer,'\0',sizeof(packetBuffer));



    				}

    			}



    			/*sprintf(msgStatus, "\r\n Received Packet: ");
    			HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

    			for (int j = 0; j < sizeof(FinalData); j++) {
    				sprintf(msgStatus, "%02X", FinalData[j]);
    				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    			}
    			sprintf(msgStatus, "\r\n");
    			HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);*/

    			// display the result counts for debugging
    			//Packet confirmation
    			sprintf(msgStatus, "\r\n Result Packet Conf: Valid = %d, Channelbusy = %d, Transmit Error = %d\r\n",result0CountPC,result1CountPC,result2CountPC);
    			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    			//Packet indication
    			sprintf(msgStatus, "\r\n Result Packet Ind: Valid = %d, CRC Error = %d, Framing Error = %d, Parity Error = %d, Receive Timeout = %d, Signal Loss = %d, Transmit Break = %d\r\n", validPacketCount, crcErrorCount, framingErrorCount, parityErrorCount, receiveTimeoutCount, signalLossCount, transmitBreakCount);
    			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

    			inPacket = false;  // Reset for the next packet
    			packetIndex = 0;



    	        // Set SS (PB6) High to deselect the slave
    	       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    	       //Clear buffers
    	       memset(rxData_getProtocol, 0, sizeof(rxData_getProtocol)); // Clear the transmit buffer
    		}
    	}
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	if (hspi ->Instance ==SPI2){


		sprintf(msgStatus, "\r\n SPI Tx Complete Callback \r\n");
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	for (int i=0; i<sizeof(combined_message); i++){
			sprintf(msgStatus, "%c",combined_message[i]);
			HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
			}
	sprintf(msgStatus, "\r\n ");
			HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);

		//HAL_SPI_Transmit_IT(&hspi2, (uint8_t *)combined_message, total_length);
		 // Set SS (PB6) High to deselect the slave
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		//Clear buffers
		memset(combined_message, 0, sizeof(combined_message)); // Clear the transmit buffer




	}
}



void AppSlaveBoard(){

	sprintf(msgStatus, "\r\n Wait slave mode on \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	//HAL_SPI_Receive_IT(&hspi2, rxData_getProtocol, sizeof(rxData_getProtocol));
	//memset(rxData_getProtocol, 0, sizeof(rxData_getProtocol));

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	//HAL_SPI_Receive_IT(&hspi2, rxData_getProtocol, sizeof(rxData_getProtocol));

}


void AppMasterBoard(){



	sprintf(msgStatus, "\r\n Master mode on \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	//HAL_SPI_Receive_IT(&hspi2, rxData_getProtocol, sizeof(act_rx_packet)); //change of the size of packet
	HAL_SPI_Receive_DMA(&hspi2, rxData_getProtocol, sizeof(act_rx_packet));


	uint32_t startTime = HAL_GetTick(); // Track the start time
	// Wait for pin trigger before proceeding
	while (!operationStarted && (HAL_GetTick() - startTime < TIMEOUT_MS)) {
		currentPinState = HAL_GPIO_ReadPin(GPIO_PORT_ENTRY, GPIO_PIN_ENTRY);
		if (currentPinState == GPIO_PIN_SET && lastPinState == GPIO_PIN_RESET) {
			HAL_Delay(1); // Debounce delay
			if (HAL_GPIO_ReadPin(GPIO_PORT_ENTRY, GPIO_PIN_ENTRY) == GPIO_PIN_SET) {
				start_transmission();
				sprintf(msgStatus,"\r\n Transmit one \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				operationStarted = 1;
			}
	    }
		lastPinState = currentPinState;
	}
	int last_transmission_state = 0;

	while(1){

		if (transmission_state != last_transmission_state){

		    sprintf(msgStatus, "\r\n Result decoded: %d, Current state: %d, Next mod: %d Module one RS: %d \r\n", result_dec, transmission_state, next, loss_trans_count);
		    HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		}
		last_transmission_state = transmission_state;

		if (packetReady_2Dec == true){
			result_dec = decode_finalData(FinalData);
			packetReady_2Dec = false;
		}

		switch (transmission_state) {

		case INITIAL_STATE:
			if (module_x == 1){
				if(result_dec == 4){
					result_dec=254;
					transmission_state = READY_FOR_TRANSMISSION;
					sprintf(msgStatus, "\r\n Ready for transmission \r\n");
					// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				}
		   	}else if (result_dec == module_x - 1) {

		   		result_dec=254;
		   		transmission_state = READY_FOR_TRANSMISSION;
		   		sprintf(msgStatus, "\r\n Ready for transmission \r\n");
		   		// HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		   	 }
		break;
		case READY_FOR_TRANSMISSION:
			transmission_timestamp = HAL_GetTick();  // Start the 1-second delay timer
			HAL_TIM_Base_Start_IT(&htim3);

			// start_transmission();
			//transmission_state = TRANSMISSION_DONE;
			sprintf(msgStatus, "\r\n Transmission done state started \r\n");
			//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

		break;
		case TRANSMISSION_DONE:
			uint32_t spiError = HAL_SPI_GetError(&hspi2);
			HAL_TIM_Base_Stop_IT(&htim3);

			if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_READY) {//HAL_GetTick() - transmission_timestamp >= transmission_delay timerFlag

				transmission_state = WAITING_FOR_PREV_MODULE;
				sprintf(msgStatus, "\r\n Transmission executed \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

			}else if(spiError != HAL_SPI_ERROR_NONE){
				sprintf(msgStatus, "\r\n SPI Error: 0x%08lx \r\n", spiError);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		    }
		break;
		case WAITING_FOR_PREV_MODULE:
			transmission_state = MONITOR_ORDER_MODULES;
			sprintf(msgStatus, "\r\n Checking module order \r\n");
		    //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		break;
		case MONITOR_ORDER_MODULES:
			if (module_x == 1){

			//Module 1 sample
			//I can't decode one as I send module 1 packet, however,
			if (result_dec == 2){
				next = next_module(result_dec);//next is equal to 2
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				sprintf(msgStatus, "\r\n My packet went through cause I see %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

			}
			if (result_dec == 3){
				next = next_module(result_dec);//next is equal to 3
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

				}
			if (result_dec == 4){
				next = next_module(result_dec);//next is equal to 1
				sprintf(msgStatus, "\r\n Next module %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				if (next == module_x) {
					result_dec=254;
					sprintf(msgStatus, "\r\n I'm next! Rd= %d \r\n",result_dec);
					HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
					transmission_state = READY_FOR_TRANSMISSION;
					}
				}
			if (result_dec == 254){
				sprintf(msgStatus, "\r\n reset decoded value %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				}

			if ((HAL_GetTick() - lastRX_time) >= TIMEOUT_MT) {  // 2 seconds passed?
				sprintf(msgStatus, "\r\n Havn't received in a while \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				lastRX_time = HAL_GetTick();  // Reset timer to prevent continuous printing
				loss_trans_count++;
				transmission_state = READY_FOR_TRANSMISSION;
			}
			}
			if(module_x == 2){


			//Module 2 sample
			if (result_dec == 1){
				next = next_module(result_dec);//next is equal to 2
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				if (next == module_x) {
					result_dec=254;
					sprintf(msgStatus, "\r\n I'm next! Rd= %d \r\n",result_dec);
					HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
					transmission_state = READY_FOR_TRANSMISSION;
					}
				}
			//I can't decode two as I send module 2 packet, however,
			if (result_dec == 3){
				next = next_module(result_dec);//next is equal to 3
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				sprintf(msgStatus, "\r\n My packet went through cause I see %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				}
			if (result_dec == 4){
				next = next_module(result_dec);//next is equal to 1
				sprintf(msgStatus, "\r\n Next module %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				}
			if (result_dec == 254){
				sprintf(msgStatus, "\r\n reset decoded value %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				}

			}
			if (module_x == 3){


			//Module 3 sample
			if (result_dec == 1){
				next = next_module(result_dec);//next is equal to 2
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
			}
			if (result_dec == 2){
				next = next_module(result_dec);//next is equal to 3
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				if (next == module_x) {
					result_dec=254;
					sprintf(msgStatus, "\r\n I'm next! Rd= %d \r\n",result_dec);
					HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
					transmission_state = READY_FOR_TRANSMISSION;
					}
			}
			//I can't decode three as I send module 3 packet, however,
			if (result_dec == 4){
				next = next_module(result_dec);//next is equal to 4
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				sprintf(msgStatus, "\r\n My packet went through cause I see %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

			}
			if (result_dec == 254){
				sprintf(msgStatus, "\r\n reset decoded value %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

			}
			}

			if (module_x == 4){
			//Module 4 sample
			if (result_dec == 1){
				next = next_module(result_dec);//next is equal to 2
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				sprintf(msgStatus, "\r\n My packet went through cause I see %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
			}
			if (result_dec == 2){
				next = next_module(result_dec);//next is equal to 3
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

			}

			if (result_dec == 3){
				next = next_module(result_dec);//next is equal to 4
				sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				if (next == module_x) {
					result_dec=254;
					sprintf(msgStatus, "\r\n I'm next! Rd= %d \r\n",result_dec);
					HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
					transmission_state = READY_FOR_TRANSMISSION;
					}

			}
			//I can't decode four as I send module 4 packet, however,
			if (result_dec == 254){
				sprintf(msgStatus, "\r\n reset decoded value %d \r\n",result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

			}
			}




			/*

			uint8_t last_decoded = 0;  // Stores last successfully decoded module
			uint8_t expected = (module_x == 1) ? 4 : module_x - 1;

			if (result_dec == expected) {
				sprintf(msgStatus,"\r\n Received expected module %d, processing...\n", result_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				last_decoded = result_dec;

				if (next == module_x) {
					result_dec=254;
					sprintf(msgStatus, "\r\n I'm next! Rd= %d \r\n",result_dec);
					HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
					transmission_state = READY_FOR_TRANSMISSION;
				}
			} else if (result_dec == next_module(module_x)) {
				sprintf(msgStatus,"\r\n Received module %d, confirming my message was sent correctly. Next expected: %d\r\n", result_dec, next_module(module_x));
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
			} else {
			    	sprintf(msgStatus,"\r\n Unexpected module %d received. Possible missing packet!\r\n", result_dec);
			    	HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

			    }
			if ((HAL_GetTick() - lastRX_time) >= TIMEOUT_MT) {  // 2 seconds passed?
				sprintf(msgStatus, "\r\n Havn't received in a while \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
				lastRX_time = HAL_GetTick();  // Reset timer to prevent continuous printing
			}*/


/*



					//if (packetReady_2Dec) {
					//packetReady_2Dec = false; // Reset the flag
					//
					//result_dec = decode_finalData(FinalData);
					next = next_module(result_dec);
					sprintf(msgStatus, "\r\n Next module %d \r\n", next);
				    //HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
					following_mod_x=next_module(module_x);

					if ((is_spi_receiving == false) && (HAL_GetTick() - lastRX_time) >= TIMEOUT_MT) {  // 2 seconds passed?
						sprintf(msgStatus, "\r\n Havn't received in a while \r\n");
						HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
						lastRX_time = HAL_GetTick();  // Reset timer to prevent continuous printing
					    }

					// If the next value is the one before my position, print the message
					if (next == module_x ) {
						result_dec=254;
						sprintf(msgStatus, "\r\n I'm next! Rd= %d \r\n",result_dec);
						//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

						transmission_state = READY_FOR_TRANSMISSION;

						// Check if 4 is the next in the sequence
						if(following_mod_x == result_dec){
							sprintf(msgStatus, "\r\n Transmitted successfully %d \r\n", following_mod_x);
							HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
						}
						/*else if ((is_spi_receiving == false) && (HAL_GetTick() - lastRX_time) >= TIMEOUT_MT) {  // 2 seconds passed?
							sprintf(msgStatus, "\r\n Havn't received in a while \r\n");
							HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
							lastRX_time = HAL_GetTick();  // Reset timer to prevent continuous printing
							transmission_state = READY_FOR_TRANSMISSION;
						    }*/


					/*}else {
						sprintf(msgStatus, "\r\n Result decoded  %d\n", result_dec);
						//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
						//transmission_state = WAITING_FOR_PREV_MODULE;
					}*/

				break;


		}

		HAL_Delay(1); // Add a small delay to yield control




	}
}






void P2P_Init(void){




	/* Check User Button state */
	if (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	{
		/* User Button released */
		currentMode = MASTER_MODE;
		 //Turn LED ON

		sprintf(msgStatus, "MASTER Mode");
		HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		for (int i=0;i<2;i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(100);//Blink twice
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(100);
			}



		AppMasterBoard();

	}
	else
	{
		/* User Button pressed */
		currentMode = SLAVE_MODE;
		sprintf(msgStatus, "Slave Mode");
		HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		for (int i=0;i<1;i++){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);//Blink twice
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(100);
		}
		AppSlaveBoard();

	}

	return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim -> Instance == TIM3 ){


		sprintf(msgStatus, "\r\n Timer callback executed \r\n");
		//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		// Check if SPI is ready before starting new transmission
		 if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_READY) {
			 start_transmission();
			 transmission_state = TRANSMISSION_DONE;
		 } else {
			 sprintf(msgStatus, "\r\n SPI is busy, skipping transmission \r\n");
			 HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
		}




	}
}


void start_transmission(void){

	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	//led_state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);

	sprintf(msgStatus, "\r\n start transmits fun \r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);

	// Set SS (PB6) Low to select the slave
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	sprintf(buffer2,test_msg_danv5,counterPacket);
	uint8_t* r2s = send_custom_text(buffer2);
	memset(buffer2,'\0',sizeof(buffer2));

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
	confirmationReceived = 0;
	counterPacket++;

	//HAL_SPI_Transmit_IT(&hspi2, (uint8_t *)combined_message, total_length);//Change IT into DMA
	HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)combined_message, total_length);

	HAL_GPIO_TogglePin(Timer_GPIO_Port, Timer_Pin);
}

uint8_t next_module(uint8_t x) {
    return ((x == 4) ? 1 : x + 1);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {

        uint32_t errorCode = HAL_SPI_GetError(hspi);
        sprintf(msgStatus, "\r\n SPI Error: 0x%08lx \r\n", errorCode);
        HAL_UART_Transmit(&huart2, (uint8_t *)msgStatus, strlen(msgStatus), HAL_MAX_DELAY);
    }
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
