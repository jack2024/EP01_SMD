/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#define PLED_port   GPIOA
#define Pled        GPIO_PIN_15

#define BTN_port   GPIOB
#define Pack       GPIO_PIN_0
#define Preset     GPIO_PIN_1
#define Ptest      GPIO_PIN_2

#define HC595EN_port   GPIOB
#define HC595EN_PIN    GPIO_PIN_6

#define Pack2_port   GPIOA
#define Pack2       GPIO_PIN_7

#define rd(k)  HAL_GPIO_ReadPin(BTN_port, k)

#define BELL_BUZZER_port   	GPIOB
#define Pbell       				GPIO_PIN_7
#define Pbuzzer     				GPIO_PIN_4

#define CTRL485_port   	GPIOA
#define P485ctrl    GPIO_PIN_8

#define Sync_port   	GPIOC
#define PsyncR      GPIO_PIN_14
#define PsyncS      GPIO_PIN_13

//23S17 Define
#define IO_MULTIPLE_DEVICES

#define IO_DEVICE_ADDRESS_READ   0x41
#define IO_DEVICE_ADDRESS_WRITE  0x40

#ifdef IO_MULTIPLE_DEVICES
#define IO_DEVICE_0   0x00
#define IO_DEVICE_1   0x02
#define IO_DEVICE_2   004
#define IO_DEVICE_3   0x06
#define IO_DEVICE_4   0x08
#define IO_DEVICE_5   0x0A
#define IO_DEVICE_6   0x0C
#define IO_DEVICE_7   0x0E
#endif

#define IODIRA      0x00     //0x00   Data Direction Register for PORTA
#define IPOLA     0x01     //0x02   Input Polarity Register for PORTA
#define GPINTENA    0x02     //0x04     Interrupt-on-change enable Register for PORTA                        
#define DEFVALA   0x03     //0x06   Default Value Register for PORTA
#define INTCONA   0x04     //0x08   Interrupt-on-change control Register for PORTA                          
#define IOCON      0x05     //0x0A   Configuration register for device                      
#define GPPUA      0x06     //0x0C   100kOhm pullup resistor register for PORTA (sets pin to input when set)                           
#define INTFA      0x07     //0x0E   Interrupt flag Register for PORTA                             
#define INTCAPA   0x08     //0x10   Interrupt captured value Register for PORTA                  
#define GPIO_A      0x09     //0x12   General purpose I/O Register for PORTA                            
#define OLATA      0x0A     //0x14   Output latch Register for PORTA

#define IODIRB    0x10     //0x01   Data Direction Register for PORTB
#define IPOLB     0x11     //0x03   Input Polarity Register for PORTB
#define GPINTENB    0x12     //0x05     Interrupt-on-change enable Register for PORTB
#define DEFVALB   0x13     //0x07   Default Value Register for PORTB
#define INTCONB   0x14     //0x09   Interrupt-on-change control Register for PORTB
//#define IOCON      0x15     //0x0B   //IOCON has 2 different addresses, both write to same register                              
#define GPPUB      0x16     //0x0D   100kOhm pullup resistor register for PORTB (sets pin to input when set)
#define INTFB      0x17     //0x0F   Interrupt flag Register for PORTB 
#define INTCAPB   0x18     //0x11   Interrupt captured value Register for PORTB
#define GPIO_B      0x19     //0x13   General purpose I/O Register for PORTB
#define OLATB      0x1A     //0x15   Output latch Register for PORTB

//End 23S17 Define

#define APP_RX_DATA_SIZE  150
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint16_t Byte_Count=0;
volatile uint16_t USB_State=0;
volatile uint16_t USB_Count=0;
volatile int32_t Time_RxCount =0;
unsigned char USB_buff[150];
volatile uint32_t loopcount =0;
volatile char USB_Rx0,USB_Rx1,USB_Rx2,USB_Rx3,USB_Rx4;
volatile char USB_Rx5,USB_Rx6,USB_Rx7,USB_Rx8,USB_Rx9;
volatile char USB_Rx10,USB_Rx11,USB_Rx12,USB_Rx13,USB_Rx14;
volatile char USB_Rx15,USB_Rx16,USB_Rx17,USB_Rx18,USB_Rx19;
volatile char USB_Rx20,USB_Rx21,USB_Rx22,USB_Rx23,USB_Rx24;

unsigned char ReadSet_flag=0;
unsigned char WritedSet_flag=0;

volatile uint8_t SPI_TXbuf[64];
volatile uint8_t SPI2_TXbuf[64];
volatile uint8_t USART_RXbuf[64];
volatile uint8_t USART_TXbuf[64];
volatile char UART_Rx;
volatile char Rx_data[2] ;
volatile uint8_t timer10ms =0;

HAL_StatusTypeDef SPI_Status;


volatile unsigned char RingbackEn =0;

volatile unsigned char ReFaultEn =0;
volatile int32_t RefaultTime;
volatile int32_t RingbackCount;
volatile int32_t RingbackDelayTime[65];

volatile uint16_t LoopCountRun=0;
volatile uint16_t LoopCountStore;

//volatile uint8_t test_fault;

//////////////////////////////////////
unsigned char const Input1_8_Wr_addr = 0x40;
unsigned char const Input1_8_Rd_addr = 0x41;
unsigned char const Input9_16_Wr_addr = 0x42;
unsigned char const Input9_16_Rd_addr = 0x43;
unsigned char const Input17_24_Wr_addr = 0x44;
unsigned char const Input17_24_Rd_addr = 0x45;
unsigned char const Input25_32_Wr_addr = 0x46;
unsigned char const Input25_32_Rd_addr = 0x47;
unsigned char const Input33_40_Wr_addr = 0x48;
unsigned char const Input33_40_Rd_addr = 0x49;
unsigned char const Input41_48_Wr_addr = 0x4A;
unsigned char const Input41_48_Rd_addr = 0x4B;
unsigned char const Input49_56_Wr_addr = 0x4C;
unsigned char const Input49_56_Rd_addr = 0x4D;
unsigned char const Input57_64_Wr_addr = 0x4E;
unsigned char const Input57_64_Rd_addr = 0x4F;

unsigned char StatusTime = 0;         //Hearth beat LED Timer
unsigned char HearthbeatInd = 0;      //Hearth beat Indicator
int32_t FlashingRateTime = 25;
unsigned char FlashingFlag = 0;
volatile int32_t RingbackTime = 50;
volatile int32_t RingbackCount = 50;
volatile uint8_t RingbackFlag=0;

unsigned char TimeBase1s = 100;
unsigned char AutoAckDelayTime = 0;

unsigned char AutoAckFlag = 0;
unsigned char AutoResetFlag = 0;
unsigned char AutoTestFlag = 0;

char SBUF = 0x00;

unsigned char Test_fault = 0;
unsigned char T_test = 0x00;

unsigned char Test = 0;
unsigned char Ack_F = 0;
unsigned char Ack2_F = 0;
unsigned char Reset_F = 0;

unsigned char SyncStatus = 0;   //"0" -> No Sync signal
unsigned char SyncFlag = 1;     //use for debounce sync signal
uint32_t Synctimer = 0x00; //use for debounce sync signal

unsigned char Input1_8;
unsigned char Input9_16;
unsigned char Input17_24;
unsigned char Input25_32;
unsigned char Input33_40;
unsigned char Input41_48;
unsigned char Input49_56;
unsigned char Input57_64;

unsigned char Input1_8_Send;
unsigned char Input9_16_Send;
unsigned char Input17_24_Send;
unsigned char Input25_32_Send;
unsigned char Input33_40_Send;
unsigned char Input41_48_Send;
unsigned char Input49_56_Send;
unsigned char Input57_64_Send;

unsigned char Ack1_8_Send;
unsigned char Ack9_16_Send;
unsigned char Ack17_24_Send;
unsigned char Ack25_32_Send;
unsigned char Ack33_40_Send;
unsigned char Ack41_48_Send;
unsigned char Ack49_56_Send;
unsigned char Ack57_64_Send;

unsigned char Output1_8;
unsigned char Output9_16;
unsigned char Output17_24;
unsigned char Output25_32;
unsigned char Output33_40;
unsigned char Output41_48;
unsigned char Output49_56;
unsigned char Output57_64;

unsigned char PCF8575_Ip_dat;
unsigned char PCF8575_Op_dat;

unsigned char Output595[9];

///////// Data from EEProm ///////
//int Device_Addr;

///////// Data from EEProm ///////
//int Device_Addr;

unsigned char InputType1_8;
unsigned char InputType9_16;
unsigned char InputType17_24;
unsigned char InputType25_32;
unsigned char InputType33_40;
unsigned char InputType41_48;
unsigned char InputType49_56;
unsigned char InputType57_64;

unsigned char FaultType1_8;
unsigned char FaultType9_16;
unsigned char FaultType17_24;
unsigned char FaultType25_32;
unsigned char FaultType33_40;
unsigned char FaultType41_48;
unsigned char FaultType49_56;
unsigned char FaultType57_64;

unsigned char OutputType1_8;
unsigned char OutputType9_16;
unsigned char OutputType17_24;
unsigned char OutputType25_32;
unsigned char OutputType33_40;
unsigned char OutputType41_48;
unsigned char OutputType49_56;
unsigned char OutputType57_64;

unsigned char OutputBoth1_8;
unsigned char OutputBoth9_16;
unsigned char OutputBoth17_24;
unsigned char OutputBoth25_32;
unsigned char OutputBoth33_40;
unsigned char OutputBoth41_48;
unsigned char OutputBoth49_56;
unsigned char OutputBoth57_64;

unsigned char Alarm_Indicator1_8;
unsigned char Alarm_Indicator9_16;
unsigned char Alarm_Indicator17_24;
unsigned char Alarm_Indicator25_32;
unsigned char Alarm_Indicator33_40;
unsigned char Alarm_Indicator41_48;
unsigned char Alarm_Indicator49_56;
unsigned char Alarm_Indicator57_64;

unsigned char AutoAck;
uint32_t AutoAckTime;
unsigned char FlashingRate;
unsigned char NoOfPoint;
unsigned char MasterSlaveSync;

unsigned char const addr_sq = 0x10,end_sq = 0x11,code_sq = 0x12,start_addr_hi_sq = 0x13,start_addr_lo_sq = 0x14;         //serial sequnce
unsigned char const ubyte_hi_sq = 0x15,ubyte_lo_sq = 0x16,crc_hi_sq = 0x17,byte_count_sq = 0x19,data_sq = 0x20;      //serial sequnce

unsigned char recieve_completed = 0;
unsigned char sequence  = end_sq;  // 
unsigned char Address =1;
unsigned char RxD_DataLen = 0x00;
unsigned char TxD_Buff[60];
unsigned char RxD_Buff[60];
unsigned char CRC_Lo;
unsigned char CRC_Hi;

uint16_t Send_check_Time = 500; //if no send reset buffer every 5 second

uint16_t Start_Address = 0x0000;
uint16_t No_PointCount = 0x0000;

unsigned char Data_ByteCount = 0x00;
unsigned char Data_Buff[30];
//unsigned char DataTemp;
//unsigned char TxD_DataLen;

volatile unsigned char count1 =0,count2 =0,count3 =0,count4 =0,count5 =0;

uint16_t T_timeout = 20;   //use for calculate RxD timeout
unsigned char Rxindex = 0x00; //index old ---jj

unsigned char var_debug1,var_debug2,var_debug3,var_debug4;

///////////// Define Bit register ////////////////////////////

struct Bit64    // Input
{
   unsigned char B1,B2,B3,B4,B5,B6,B7,B8,B9,B10;
   unsigned char B11,B12,B13,B14,B15,B16,B17,B18,B19,B20;
   unsigned char B21,B22,B23,B24,B25,B26,B27,B28,B29,B30;
   unsigned char B31,B32,B33,B34,B35,B36,B37,B38,B39,B40;
   unsigned char B41,B42,B43,B44,B45,B46,B47,B48,B49,B50;
   unsigned char B51,B52,B53,B54,B55,B56,B57,B58,B59,B60;
   unsigned char B61,B62,B63,B64;
};
   //Output   (LED Lit = "0")
   //Input Type NO/NC (NO = "1" , NC = "0")
   //Fault Type Manual/Auto (Manual = "1" , Auto = "0")
   //Output Type Buz/Bell (Buzzer = "1" , Bell = "0")
   //Output Both (Normal = "1" , Both = "0")
   //Acknowledge flag (Acked = "1" , non Ack = "0")

struct Bit64 Input,Output,InputType,FaultType,OutputType,OutputBoth;
struct Bit64 AlarmIndicator,Ack,Ack2,AckSend,In,In2;
struct Bit64 bit_faultHold,Ringback;

unsigned char FaultAgo[65];
unsigned char FaultNow[65];
uint32_t ReleaseTime[65];
uint32_t FaultDelayTime[65];

unsigned char FaultNCNO[65];

unsigned char man_rst[65];
unsigned char auto_rst[65];
unsigned char analfault_index[65];

#define NO 1
#define NC 0

typedef struct {
	uint8_t Input ;
	uint8_t Output ;
	uint8_t InputType ;
	uint8_t FaultType ;
	uint8_t OutputType ;
	uint8_t OutputBoth ;
	uint8_t AlarmIndicator ;
	uint8_t Ack ;
	uint8_t Ack2 ;
	uint8_t AckSend;
	uint8_t In ;
	uint8_t In2 ;
	uint8_t bit_faultHold ;
	uint8_t Ringback ;
	uint8_t FaultAgo ;
	uint8_t FaultNow ;
	int16_t ReleaseTime ;
	uint8_t FaultDelayTime ;
	uint32_t RingbackDelayTime;
	uint8_t FaultNCNO ;
	
} FAULT_t;

FAULT_t fault[64] = {0};

unsigned char const CRC_Table_Hi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ; 

unsigned  char const CRC_Table_Lo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
} ;


/*        fLASH           */
char Flashdata[200];
uint16_t Flashindex=0;
#define FLASH_PAGE_START_ADDRESS    0x0801F800
#define FLASH_PAGE_END_ADDRESS      0x0801FFFF
//#define FLASH_PAGE_SIZE             2048
#define FLASH_PAGE_size             2048

uint8_t FlashErase(void);
uint8_t FlashWrite(uint32_t Addr, uint8_t *Data, uint32_t Length);
//++++++++++++++++++++++++++++++++++++++++++++++

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void delay_us(uint32_t us);
void CRC_Cal(unsigned char *puchMsg , unsigned char usDataLen);
void checkCommand(void);
void checkSetting(void);
void Modbus_Function(void);
void StoreReleaseFault(void);
void Read_input(void);
void Send_Ouput(void);
void ForceAllAlarm(void);
void Alarmtosend(void);
void ReadSetting(void);
void check_test(void);
void check_ack(void);
void check_ack2(void);
void check_reset(void);
uint8_t CheckAutoReset(unsigned char DatType);
void Anal_Function(void);
void Read_Config(void);	
void Write_Flash(void);
void Driver595(void);
void Cal_Data1(void);
void Cal_Data2(void);
void Cal_Data3(void);
void check_Ringback(void);

void check_Refault(void);

void refaultindex(void);

void MCP23S17_INIT(void);
void IO_WRITE_REGISTER(uint8_t address, uint8_t reg, uint8_t data);
uint8_t IO_READ_REGISTER(uint8_t address, uint8_t reg);
void IO_SET_TRIS_A(uint8_t address, uint8_t data);
void IO_SET_TRIS_B(uint8_t address, uint8_t data);
void IO_OUTPUT_A(uint8_t address, uint8_t data);
void IO_OUTPUT_B(uint8_t address, uint8_t data);
uint8_t IO_INPUT_A(uint8_t address);
uint8_t IO_INPUT_B(uint8_t address);


void refaultindex(void)
{
	int8_t mancount =0, autocount =0 , count =0;
	for(unsigned char i = 0;i < NoOfPoint; i++)
	{
		if(fault[i].FaultType == 1)  // 1 = manual reset
		{
			 man_rst[mancount] = i ;
			 mancount++ ;
		}
		else // 0 = auto reset
		{
			auto_rst[autocount] = i ;
			autocount++ ;
		}
		
	}
	
	for(unsigned char j = 0;j < autocount; j++)
	{
		analfault_index[count] = auto_rst[j];
		count++ ;
	}
	
	for(unsigned char k = 0;k < mancount; k++)
	{
		analfault_index[count] = man_rst[k];
		count++ ;
	}
}


//Interrupt callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		//Byte_Count++;
		//HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data, 1);   //activate UART receive interrupt every time
		//SBUF = Rx_data[0];    //add data to Rx_Buffer
		//checkCommand();
}

void HAL_SYSTICK_Callback()
{
	if(++timer10ms >=10)
	{
		timer10ms =0;
		
		StatusTime++;
		
		LoopCountStore = LoopCountRun;
		LoopCountRun =0;

		 if(T_timeout != 0)
		 {
				T_timeout--;
				if(T_timeout == 0)
				{
					 //sequence = stop_sq;         //timeout
					sequence = end_sq;         //timeout    
					HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);

				}
		 }
		 
		 ////////// Time Base 1 Second /////////////
		 if(TimeBase1s != 0x00)
		 {
				TimeBase1s--;
				if(TimeBase1s == 0x00)      // <====== code in time base 1 second
				{
					 if(AutoAckDelayTime != 0x00)
					 {
							AutoAckDelayTime--;
							if(AutoAckDelayTime == 0x00)
							{
								 AutoAckFlag = 1 ;
							}
					 }
					 //////////////////////////////
					 TimeBase1s = 100;
					 
				}

		 }
		 ///////////// End Time base 1 Second ///////



		 if(StatusTime == 40)    //500mS
		 {
				StatusTime = 0;
				if(HearthbeatInd == 0)
				{
					 HearthbeatInd = 1;
					 //output_bit(Pled,1);
					//HAL_GPIO_WritePin(PLED_port,Pled,GPIO_PIN_SET);
				}
				else
				{
					 HearthbeatInd = 0;
					 //HAL_GPIO_WritePin(PLED_port,Pled,GPIO_PIN_RESET);
				}
				/////////////////////////////////////////// time base 500 ms
				if(Test == 1)            //Test function
				{
						if(T_test > 0x00) T_test --;
						if(T_test == 0x00) Test_fault = 1;
				}
				else
				{
						//Test = 0;
						T_test = 0x00;
				}
		 }
		 ///////////////////////////////
		 if(FlashingRateTime != 0)
		 {
				FlashingRateTime--;
				if(FlashingRateTime == 0)
				{
					 if(SyncStatus == 0)
					 {
							if(FlashingFlag == 1)
							{
									if(SyncStatus){
										FlashingFlag = 0; //sleve
									}
									else{
										//RingbackFlag = FlashingFlag = 0; //master
										FlashingFlag = 0; //master
									}
								  
								  HAL_GPIO_WritePin(Sync_port,PsyncS,GPIO_PIN_RESET);
							}
							else
							{

								 if(SyncStatus){
										FlashingFlag = 1; //sleve
									}
									else{
										//RingbackFlag = FlashingFlag = 1; //master
										FlashingFlag = 1; //master
									}
								 HAL_GPIO_WritePin(Sync_port,PsyncS,GPIO_PIN_SET);
							}
					 }
					 
					 FlashingRateTime = FlashingRate;
					 
					 // Ringback
//					if(SyncStatus == 0){
//						 RingbackTime++;
//						 if((RingbackTime %2) == 0)
//						 {
//							 if(RingbackFlag)
//							 {
//								 RingbackFlag = 0;
//							 }
//							 else
//							 {
//								 RingbackFlag=1;
//							 }
//						 }
//					 }
//					else
//					{
//						//RingbackFlag = HAL_GPIO_ReadPin(Sync_port, PsyncR);
//					}
					 //FlashingRateTime = FlashingRate; //reload value
					 
					// jj modify 64
//				  if(Ringback.B1 == 1 || Ringback.B2 == 1 || Ringback.B3 == 1 || Ringback.B4 == 1 || Ringback.B5 == 1 || Ringback.B6 == 1 || Ringback.B7 == 1 || Ringback.B8 == 1 || Ringback.B9 == 1 || Ringback.B10 == 1 
//					|| Ringback.B11 == 1 || Ringback.B12 == 1 || Ringback.B13 == 1 || Ringback.B14 == 1 || Ringback.B15 == 1 || Ringback.B16 == 1 || Ringback.B17 == 1 || Ringback.B18 == 1 || Ringback.B19 == 1 || Ringback.B20 == 1 					
//					|| Ringback.B21 == 1 || Ringback.B22 == 1 || Ringback.B23 == 1 || Ringback.B24 == 1 || Ringback.B25 == 1 || Ringback.B26 == 1 || Ringback.B27 == 1 || Ringback.B28 == 1 || Ringback.B29 == 1 || Ringback.B30 == 1 
//					|| Ringback.B31 == 1 || Ringback.B32 == 1 || Ringback.B33 == 1 || Ringback.B34 == 1 || Ringback.B35 == 1 || Ringback.B36 == 1 || Ringback.B37 == 1 || Ringback.B38 == 1 || Ringback.B39 == 1 || Ringback.B40 == 1 
//					|| Ringback.B41 == 1 || Ringback.B42 == 1 || Ringback.B43 == 1 || Ringback.B44 == 1 || Ringback.B45 == 1 || Ringback.B46 == 1 || Ringback.B47 == 1 || Ringback.B48 == 1 || Ringback.B49 == 1 || Ringback.B50 == 1 
//					|| Ringback.B51 == 1 || Ringback.B52 == 1 || Ringback.B53 == 1 || Ringback.B54 == 1 || Ringback.B55 == 1 || Ringback.B56 == 1 || Ringback.B57 == 1 || Ringback.B58 == 1 || Ringback.B59 == 1 || Ringback.B60 == 1 
//					|| Ringback.B61 == 1 || Ringback.B62 == 1 || Ringback.B63 == 1 || Ringback.B64 == 1)  
//					{
//						FlashingRateTime = (FlashingRate *2);
//					}
//					else
//					{
//						FlashingRateTime = FlashingRate; //reload value
//					}


//						uint8_t ringcount = 0;
//						for(uint8_t i = 0; i < NoOfPoint; i++)
//						{	
//							if(fault[i].Ringback == 1){
//								ringcount++;
//							}
//						}
//						if(ringcount)FlashingRateTime = (FlashingRate *2);
//						else FlashingRateTime = FlashingRate; //reload value
				//////////////////////////////////////////////
					 				 
				}
				
				
				

				if(Synctimer != 0x00)
				{
					 Synctimer--;
					 if(Synctimer == 0x00)
					 {
							SyncStatus = 0;
					 }
				}
		 }

		// Ringback
		if(RingbackTime != 0)
		{
			RingbackTime--;
			if(RingbackTime == 0)
			{
				RingbackTime = FlashingRate * 2 ;
				if(RingbackFlag){
					RingbackFlag = 0;
				}
				else{
					RingbackFlag = 1 ;
				}
			}
			
		}

		 
	}
}








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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //jj
	
	HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);// OFF BELL
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);// OFF BUZZER

	char Set =*(uint8_t *)0x801F800;
	if(Set != 0x0F)
	{
		for(unsigned char i = 0;i < 16; i++)
		{
			fault[i].InputType = 1 ;
			fault[i].FaultNCNO = 1 ;
			fault[i].FaultType = 1 ;
			fault[i].OutputType = 1 ;
			fault[i].OutputBoth = 0 ;
			fault[i].AlarmIndicator = 1 ;
			fault[i].FaultDelayTime = 0 ;
			//HAL_IWDG_Refresh(&hiwdg);
		}
		
		ReFaultEn = 0;
		RefaultTime = 0;
			
		AutoAck = 0xF0; //not use auto ack
		AutoAckTime = 5;
		FlashingRate = 25;
		NoOfPoint = 16;
		MasterSlaveSync = 0x0F;
		Address = 1;
	}
	else
	{
		Read_Config();
	}
	
	 FlashingFlag = 1;
	 sequence = end_sq;
	
	for(unsigned char i = 0; i < 63; i++)
	{
		fault[i].Output = 1 ;
		fault[i].In = 0 ;
		fault[i].In2 = 0;
		fault[i].Ack = 0;
		//HAL_IWDG_Refresh(&hiwdg);
	}
	
	refaultindex();
	 
	 MCP23S17_INIT();   //initializes the MCP23S17 chip.//----------jj
	
	if(NoOfPoint >= 8)
	{
		IO_SET_TRIS_A(IO_DEVICE_0, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_0, 0xFF); //addr.0 Set PortA As Output
		IO_WRITE_REGISTER(IO_DEVICE_0, GPPUB, 0xFF); // Input Pullup
	}
	if(NoOfPoint >= 16)
	{
		
		IO_SET_TRIS_A(IO_DEVICE_1, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_1, 0xFF); //addr.0 Set PortA As Output
		
		IO_WRITE_REGISTER(IO_DEVICE_1, GPPUB, 0xFF); // Input Pullup
	}
	 if(NoOfPoint >= 24)
	 {
		IO_SET_TRIS_A(IO_DEVICE_2, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_2, 0xFF); //addr.0 Set PortA As Output		
		IO_WRITE_REGISTER(IO_DEVICE_2, GPPUB, 0xFF); // Input Pullup
	 }
	 if(NoOfPoint >= 32)
	 {
		IO_SET_TRIS_A(IO_DEVICE_3, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_3, 0xFF); //addr.0 Set PortA As Output
		IO_WRITE_REGISTER(IO_DEVICE_3, GPPUB, 0xFF); // Input Pullup
	 }
	 if(NoOfPoint >= 40)
	 {
		IO_SET_TRIS_A(IO_DEVICE_4, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_4, 0xFF); //addr.0 Set PortA As Output
		IO_WRITE_REGISTER(IO_DEVICE_4, GPPUB, 0xFF); // Input Pullup		
	 }
	 if(NoOfPoint >= 48)
	 {
		IO_SET_TRIS_A(IO_DEVICE_5, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_5, 0xFF); //addr.0 Set PortA As Output
		IO_WRITE_REGISTER(IO_DEVICE_5, GPPUB, 0xFF); // Input Pullup		
	 }
	 if(NoOfPoint >= 56)
	 {
		IO_SET_TRIS_A(IO_DEVICE_6, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_6, 0xFF); //addr.0 Set PortA As Output
		IO_WRITE_REGISTER(IO_DEVICE_6, GPPUB, 0xFF); // Input Pullup		
	 }
	 if(NoOfPoint >= 64)
	 {
		IO_SET_TRIS_A(IO_DEVICE_7, 0x00); //addr.0 Set PortB As Input
		IO_SET_TRIS_B(IO_DEVICE_7, 0xFF); //addr.0 Set PortA As Output
		IO_WRITE_REGISTER(IO_DEVICE_7, GPPUB, 0xFF); // Input Pullup		
	 }
	 
	 if(NoOfPoint >= 8)
	 {
		IO_OUTPUT_A(IO_DEVICE_0, 0x00);	
	 }
	 if(NoOfPoint >= 16)
	 {
		
		IO_OUTPUT_A(IO_DEVICE_1, 0x00);
	 }
	 if(NoOfPoint >= 24)
	 {
		IO_OUTPUT_A(IO_DEVICE_2, 0x00);
	 }
	 if(NoOfPoint >= 32)
	 {
		IO_OUTPUT_A(IO_DEVICE_3, 0x00);
	 }
	 if(NoOfPoint >= 40)
	 {
		IO_OUTPUT_A(IO_DEVICE_4, 0x00);
	 }
	 if(NoOfPoint >= 48)
	 {
		IO_OUTPUT_A(IO_DEVICE_5, 0x00);
	 }
	 if(NoOfPoint >= 56)
	 {
		IO_OUTPUT_A(IO_DEVICE_6, 0x00);
	 }
	 if(NoOfPoint >= 64)
	 {
		IO_OUTPUT_A(IO_DEVICE_7, 0x00);
	 }
	 
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 HAL_GPIO_TogglePin(PLED_port,Pled);	
	 HAL_Delay(100); //HAL_IWDG_Refresh(&hiwdg);
	 
	if(NoOfPoint >= 8)
	{
		IO_OUTPUT_A(IO_DEVICE_0, 0xFF);	
	}
	 if(NoOfPoint >= 16)
	 {
			
			IO_OUTPUT_A(IO_DEVICE_1, 0xFF);
	 }
	 if(NoOfPoint >= 24)
	 {
			IO_OUTPUT_A(IO_DEVICE_2, 0xFF);
	 }
	 if(NoOfPoint >= 32)
	 {
			IO_OUTPUT_A(IO_DEVICE_3, 0xFF);
	 }
	 if(NoOfPoint >= 40)
	 {
			IO_OUTPUT_A(IO_DEVICE_4, 0xFF);
	 }
	 if(NoOfPoint >= 48)
	 {
			IO_OUTPUT_A(IO_DEVICE_5, 0xFF);
	 }
	 if(NoOfPoint >= 56)
	 {
			IO_OUTPUT_A(IO_DEVICE_6, 0xFF);
	 }
	 if(NoOfPoint >= 64)
	 {
			IO_OUTPUT_A(IO_DEVICE_7, 0xFF);
	 }
	 
	  //  JJ 
	 if((FlashingRateTime == 0)||(FlashingRateTime == 0xFF))
	 {
		 FlashingRateTime =25;
	 }
	 
	 SyncFlag = HAL_GPIO_ReadPin(Sync_port, PsyncR); // read pin sync receive init.
	 
	//HAL_UART_Receive_IT(&huart1,(uint8_t*)USART_RXbuf, 1);	//activate UART receive interrupt every time
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_3);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_IWDG_Refresh(&hiwdg);
		// Refresh MCP23S17
		
		if(NoOfPoint >= 16)
		{
			IO_OUTPUT_B(IO_DEVICE_0, 0xFF); //jj 
			IO_OUTPUT_B(IO_DEVICE_1, 0xFF); //jj 
		}
		if(NoOfPoint >= 24)
		{
			IO_OUTPUT_B(IO_DEVICE_2, 0xFF); //jj 
		}
		if(NoOfPoint >= 32)
		{
			IO_OUTPUT_B(IO_DEVICE_3, 0xFF); //jj 
		}
		if(NoOfPoint >= 40)
		{
			IO_OUTPUT_B(IO_DEVICE_4, 0xFF); //jj 
		}
		if(NoOfPoint >= 48)
		{
			IO_OUTPUT_B(IO_DEVICE_5, 0xFF); //jj 
		}
		if(NoOfPoint >= 56)
		{
			IO_OUTPUT_B(IO_DEVICE_6, 0xFF); //jj 	
		}
		if(NoOfPoint >= 64)
		{
			IO_OUTPUT_B(IO_DEVICE_7, 0xFF); //jj 	
		}
		
		//&&&&&&&&&&&&&&&&&&&&&&&&&

		if(recieve_completed == 1)
		{
			Modbus_Function();
			recieve_completed = 0 ;
		}

		if(HAL_GPIO_ReadPin(Sync_port, PsyncR) != SyncFlag)      //Check Sync
		{
			RingbackFlag = SyncFlag = FlashingFlag = HAL_GPIO_ReadPin(Sync_port, PsyncR);
			//SyncFlag = HAL_GPIO_ReadPin(Sync_port, PsyncR);
			
			
			//RingbackFlag = SyncFlag ; //JJ
			if(SyncFlag)
			{
				//FlashingRateTime = FlashingRate; //reload value
				HAL_GPIO_WritePin(Sync_port,PsyncS,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(Sync_port,PsyncS,GPIO_PIN_RESET);
				//FlashingRateTime = FlashingRate; //reload value
			}
			SyncStatus = 1;
			Synctimer = 25;
			
			//RingbackTime = 50;
		}
		
		check_ack();
		check_ack2();
		check_reset();
		check_test();
		//HAL_IWDG_Refresh(&hiwdg);
		
		if(RingbackEn){
				check_Ringback(); 
				//HAL_IWDG_Refresh(&hiwdg);
		}
		
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_3);	// Check time loop 400uSec/loop(16 fault) 
		//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
		if((loopcount % 100) == 0){
			
			if(ReFaultEn){
				check_Refault(); 
				//HAL_IWDG_Refresh(&hiwdg);
			}
		}
		//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);	
		
		Read_input(); //HAL_IWDG_Refresh(&hiwdg);
		Anal_Function(); //HAL_IWDG_Refresh(&hiwdg);	
		Send_Ouput(); //HAL_IWDG_Refresh(&hiwdg);
				
		//if(++loopcount >= 2500)
		if(++loopcount >= 1000)
		{
			//refaultindex();
			loopcount =0;
			HAL_GPIO_TogglePin(PLED_port,Pled);	
			
			RingbackCount = RefaultTime*(LoopCountStore*100*60); //LoopCountStore = 10 ms * 100 = 1 Sec *60 = 1 Minute
			
				// Refresh MCP23S17
			if(NoOfPoint >= 16)
			{
				IO_SET_TRIS_A(IO_DEVICE_0, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_0, 0xFF); //addr.0 Set PortA As Output
				IO_SET_TRIS_A(IO_DEVICE_1, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_1, 0xFF); //addr.0 Set PortA As Output
				IO_WRITE_REGISTER(IO_DEVICE_0, GPPUB, 0xFF); // Input Pullup
				IO_WRITE_REGISTER(IO_DEVICE_1, GPPUB, 0xFF); // Input Pullup
			}
			if(NoOfPoint >= 24)
			{
				IO_SET_TRIS_A(IO_DEVICE_2, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_2, 0xFF); //addr.0 Set PortA As Output		
				IO_WRITE_REGISTER(IO_DEVICE_2, GPPUB, 0xFF); // Input Pullup
			}
			if(NoOfPoint >= 32)
			{
				IO_SET_TRIS_A(IO_DEVICE_3, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_3, 0xFF); //addr.0 Set PortA As Output
				IO_WRITE_REGISTER(IO_DEVICE_3, GPPUB, 0xFF); // Input Pullup
			}
			if(NoOfPoint >= 40)
			{
				IO_SET_TRIS_A(IO_DEVICE_4, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_4, 0xFF); //addr.0 Set PortA As Output
				IO_WRITE_REGISTER(IO_DEVICE_4, GPPUB, 0xFF); // Input Pullup		
			}
			if(NoOfPoint >= 48)
			{
				IO_SET_TRIS_A(IO_DEVICE_5, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_5, 0xFF); //addr.0 Set PortA As Output
				IO_WRITE_REGISTER(IO_DEVICE_5, GPPUB, 0xFF); // Input Pullup		
			}
			if(NoOfPoint >= 56)
			{
				IO_SET_TRIS_A(IO_DEVICE_6, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_6, 0xFF); //addr.0 Set PortA As Output
				IO_WRITE_REGISTER(IO_DEVICE_6, GPPUB, 0xFF); // Input Pullup		
			}
			if(NoOfPoint >= 64)
			{
				IO_SET_TRIS_A(IO_DEVICE_7, 0x00); //addr.0 Set PortB As Input
				IO_SET_TRIS_B(IO_DEVICE_7, 0xFF); //addr.0 Set PortA As Output
				IO_WRITE_REGISTER(IO_DEVICE_7, GPPUB, 0xFF); // Input Pullup		
			}
		}
		if((loopcount % 100) ==0)
		{
			Driver595(); 
		}
		LoopCountRun++;
		
  }																				// Check time loop 1.4 uSec/loop(64 fault) 
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB4 PB6 PB7
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Function Begin $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$	
void Read_input(void)
{
	uint8_t MCP23s17_Ip_dat;
	uint8_t j = 0 ;

	if(NoOfPoint >= 8)
	{
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_0);	
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)	
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
		}	
	}
	if(NoOfPoint >= 16)
	{
			
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_1);
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
			
		}	
	}
	
	if(NoOfPoint >= 24)
	{
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_2);
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
		}
	}
	
	if(NoOfPoint >= 32)
	{
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_3);
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
		}
	}
		
	if(NoOfPoint >= 40)
	{
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_4);
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~ MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
		
		}
	}
	
	if(NoOfPoint >= 48)
	{
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_5);
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
		
		}
	}
	
	if(NoOfPoint >= 56)
	{
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_6);
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
			
		}
	}
	
	if(NoOfPoint >= 64)
	{
		MCP23s17_Ip_dat = IO_INPUT_B(IO_DEVICE_7);
		if (AutoTestFlag)  // TEST FROM MODBUS
		{
			MCP23s17_Ip_dat = ~MCP23s17_Ip_dat;
			//  jj
			if(MCP23s17_Ip_dat == 0)MCP23s17_Ip_dat = 0xFF;
		}
		if(SPI_Status == HAL_OK)
		{
			for(int8_t i = 7; i >= 0; --i)
			{
				fault[j].FaultNow = MCP23s17_Ip_dat >> i;  fault[j].FaultNow = fault[j].FaultNow & 0x01;
				j++;
			}
			
		}
	}
	
	///////////////---Anal Fault---//////////////////
	
	for(uint8_t i = 0; i <= NoOfPoint; i++)
	{
		if(fault[i].InputType == NO)
		{
			if(fault[i].FaultNow == 0)
			{	
				if(fault[i].FaultAgo == 0)
				{
					fault[i].ReleaseTime++;
					if(ReFaultEn){
						fault[i].RingbackDelayTime++;
					}
					if(fault[i].ReleaseTime >= ((fault[i].FaultDelayTime*(LoopCountStore*20))+10)) // LoopCountStore in 10 ms.LoopCountStore*20 = 200 ms 
					{	
						fault[i].ReleaseTime = 0;
						fault[i].Input = 0; 
					}
				}
			}
			else
			{
				fault[i].ReleaseTime = 0;
				fault[i].RingbackDelayTime = 0;
				
				fault[i].Input = 1;
				if(fault[i].bit_faultHold)
				{
					fault[i].bit_faultHold = 0;
					fault[i].Ringback =1;
				}	
			}
		}
		else if(fault[i].InputType == NC)
		{
			if(fault[i].FaultNow == 1)
			{
				if(fault[i].FaultAgo == 1)
				{
					fault[i].ReleaseTime++;
					
					if(ReFaultEn){
						fault[i].RingbackDelayTime++;
					}
					 
					if(fault[i].ReleaseTime >= ((fault[i].FaultDelayTime*(LoopCountStore*20))+10)) // LoopCountStore in 10 ms.LoopCountStore*20 = 200 ms
					{	
						fault[i].ReleaseTime = 0;
						fault[i].Input = 1 ;										
					}
				}
			}
			else
			{
				fault[i].ReleaseTime = 0;
				fault[i].RingbackDelayTime = 0;
				
				fault[i].Input = 0;
				if(fault[i].bit_faultHold){
					fault[i].bit_faultHold = 0;
					fault[i].Ringback =1;
				}
				
			}
		}
	}
	
	StoreReleaseFault();
	
//	if(NoOfPoint >= 16)
//  {
//		Input1_8 = 0x00;	
//		Input1_8 = Input1_8 | Input.B8;
//		Input1_8 = (Input1_8 << 1) | Input.B7;
//		Input1_8 = (Input1_8 << 1) | Input.B6;
//		Input1_8 = (Input1_8 << 1) | Input.B5;
//		Input1_8 = (Input1_8 << 1) | Input.B4;
//		Input1_8 = (Input1_8 << 1) | Input.B3;
//		Input1_8 = (Input1_8 << 1) | Input.B2;
//		Input1_8 = (Input1_8 << 1) | Input.B1;
//		Input1_8 = ~Input1_8;
//		
//		
//		Input9_16 = 0x00;
//		Input9_16 = Input9_16 | Input.B16;
//		Input9_16 = (Input9_16 << 1) | Input.B15;
//		Input9_16 = (Input9_16 << 1) | Input.B14;
//		Input9_16 = (Input9_16 << 1) | Input.B13;
//		Input9_16 = (Input9_16 << 1) | Input.B12;
//		Input9_16 = (Input9_16 << 1) | Input.B11;
//		Input9_16 = (Input9_16 << 1) | Input.B10;
//		Input9_16 = (Input9_16 << 1) | Input.B9;
//		Input9_16 = ~Input9_16;
//	}
//	if(NoOfPoint >= 24)
//  {
//		Input17_24 = 0x00;
//		Input17_24 = Input17_24 | Input.B24;
//		Input17_24 = (Input17_24 << 1) | Input.B23;
//		Input17_24 = (Input17_24 << 1) | Input.B22;
//		Input17_24 = (Input17_24 << 1) | Input.B21;
//		Input17_24 = (Input17_24 << 1) | Input.B20;
//		Input17_24 = (Input17_24 << 1) | Input.B19;
//		Input17_24 = (Input17_24 << 1) | Input.B18;
//		Input17_24 = (Input17_24 << 1) | Input.B17;
//		Input17_24 = ~Input17_24;
//	}
//	if(NoOfPoint >= 32)
//  {
//		Input25_32 = 0x00;
//		Input25_32 = Input25_32 | Input.B32;
//		Input25_32 = (Input25_32 << 1) | Input.B31;
//		Input25_32 = (Input25_32 << 1) | Input.B30;
//		Input25_32 = (Input25_32 << 1) | Input.B29;
//		Input25_32 = (Input25_32 << 1) | Input.B28;
//		Input25_32 = (Input25_32 << 1) | Input.B27;
//		Input25_32 = (Input25_32 << 1) | Input.B26;
//		Input25_32 = (Input25_32 << 1) | Input.B25;
//		Input25_32 = ~Input25_32;
//	}
//	if(NoOfPoint >= 40)
//  {
//		Input33_40 = 0x00;
//		Input33_40 = Input33_40 | Input.B40;
//		Input33_40 = (Input33_40 << 1) | Input.B39;
//		Input33_40 = (Input33_40 << 1) | Input.B38;
//		Input33_40 = (Input33_40 << 1) | Input.B37;
//		Input33_40 = (Input33_40 << 1) | Input.B36;
//		Input33_40 = (Input33_40 << 1) | Input.B35;
//		Input33_40 = (Input33_40 << 1) | Input.B34;
//		Input33_40 = (Input33_40 << 1) | Input.B33;
//		Input33_40 = ~Input33_40;
//	}
//	if(NoOfPoint >= 48)
//  {
//		Input41_48 = 0x00;
//		Input41_48 = Input41_48 | Input.B48;
//		Input41_48 = (Input41_48 << 1) | Input.B47;
//		Input41_48 = (Input41_48 << 1) | Input.B46;
//		Input41_48 = (Input41_48 << 1) | Input.B45;
//		Input41_48 = (Input41_48 << 1) | Input.B44;
//		Input41_48 = (Input41_48 << 1) | Input.B43;
//		Input41_48 = (Input41_48 << 1) | Input.B42;
//		Input41_48 = (Input41_48 << 1) | Input.B41;
//		Input41_48 = ~Input41_48;
//	}
//	if(NoOfPoint >= 56)
//  {
//		Input49_56 = 0x00;
//		Input49_56 = Input49_56 | Input.B56;
//		Input49_56 = (Input49_56 << 1) | Input.B55;
//		Input49_56 = (Input49_56 << 1) | Input.B54;
//		Input49_56 = (Input49_56 << 1) | Input.B53;
//		Input49_56 = (Input49_56 << 1) | Input.B52;
//		Input49_56 = (Input49_56 << 1) | Input.B51;
//		Input49_56 = (Input49_56 << 1) | Input.B50;
//		Input49_56 = (Input49_56 << 1) | Input.B49;
//		Input49_56 = ~Input49_56;
//	}
//	if(NoOfPoint >= 64)
//  {
//		Input57_64 = 0x00;
//		Input57_64 = Input57_64 | Input.B64;
//		Input57_64 = (Input57_64 << 1) | Input.B63;
//		Input57_64 = (Input57_64 << 1) | Input.B62;
//		Input57_64 = (Input57_64 << 1) | Input.B61;
//		Input57_64 = (Input57_64 << 1) | Input.B60;
//		Input57_64 = (Input57_64 << 1) | Input.B59;
//		Input57_64 = (Input57_64 << 1) | Input.B58;
//		Input57_64 = (Input57_64 << 1) | Input.B57;
//		Input57_64 = ~Input57_64;
//	}
	
	if(NoOfPoint >= 8)
	{
		for(int8_t i = 7; i >= 0; --i)
		{
			Input1_8 = Input1_8 | fault[i].Input;
			if(i > 0){
				Input1_8 = Input1_8 << 1;
			}
		}
		Input1_8 = ~Input1_8; 
		
	}
	if(NoOfPoint >= 16)
    {
		for(int8_t i = 15; i >= 8; --i)
		{
			Input9_16 = Input9_16 | fault[i].Input;
			if(i > 8){
				Input9_16 = Input9_16 << 1;
			}
		}
		Input9_16 = ~Input9_16; 
		

	}
	if(NoOfPoint >= 24)
    {
		for(int8_t i = 23; i >= 16; --i)
		{
			Input17_24 = Input17_24 | fault[i].Input;
			if(i > 16){
				Input17_24 = Input17_24 << 1;
			}
		}
		Input17_24 = ~Input17_24; 

	}
	if(NoOfPoint >= 32)
    {
		for(int8_t i = 31; i >= 24; --i)
		{
			Input25_32 = Input25_32 | fault[i].Input;
			if(i > 24){
				Input25_32 = Input25_32 << 1;
			}
		}
		Input25_32 = ~Input25_32; 

	}
	if(NoOfPoint >= 40)
    {
		for(int8_t i = 39; i >= 32; --i)
		{
			Input33_40 = Input33_40 | fault[i].Input;
			if(i > 32){
				Input33_40 = Input33_40 << 1;
			}
		}
		Input25_32 = ~Input25_32; 

	}
	if(NoOfPoint >= 48)
    {
		for(int8_t i = 47; i >= 40; --i)
		{
			Input41_48 = Input41_48 | fault[i].Input;
			if(i > 40){
				Input41_48 = Input41_48 << 1;
			}
		}
		Input41_48 = ~Input41_48; 
	}
	if(NoOfPoint >= 56)
    {
		for(int8_t i = 55; i >= 48; --i)
		{
			Input49_56 = Input49_56 | fault[i].Input;
			if(i > 48){
				Input49_56 = Input49_56 << 1;
			}
		}
		Input49_56 = ~Input49_56; 
	}
	if(NoOfPoint >= 64)
    {
		for(int8_t i = 63; i >= 56; --i)
		{
			Input57_64 = Input57_64 | fault[i].Input;
			if(i > 56){
				Input57_64 = Input57_64 << 1;
			}
		}
		Input57_64 = ~Input57_64; 

	}
}

void Alarmtosend(void)
{
	Input1_8_Send = 0x00;
	Input9_16_Send = 0x00;
	Input17_24_Send = 0x00;
	Input25_32_Send = 0x00;
	Input33_40_Send = 0x00;
	Input41_48_Send = 0x00;
	Input49_56_Send = 0x00;
	Input57_64_Send = 0x00;
   
	Ack1_8_Send = 0x00;
	Ack9_16_Send = 0x00;
	Ack17_24_Send = 0x00;
	Ack25_32_Send = 0x00;
	Ack33_40_Send = 0x00;
	Ack41_48_Send = 0x00;
	Ack49_56_Send = 0x00;
	Ack57_64_Send = 0x00;
   
	for(uint8_t i = 0; i < NoOfPoint; i++)
	{
		if((fault[i].In == 0) && (fault[i].Output ==0))
		  fault[i].AckSend = 1;
		else
		  fault[i].AckSend = 0;
	}
	
	Ack1_8_Send = Ack1_8_Send | fault[7].AckSend;
	Ack1_8_Send = (Ack1_8_Send << 1) | fault[6].AckSend;
	Ack1_8_Send = (Ack1_8_Send << 1) | fault[5].AckSend;
	Ack1_8_Send = (Ack1_8_Send << 1) | fault[4].AckSend;
	Ack1_8_Send = (Ack1_8_Send << 1) | fault[3].AckSend;
	Ack1_8_Send = (Ack1_8_Send << 1) | fault[2].AckSend;
	Ack1_8_Send = (Ack1_8_Send << 1) | fault[1].AckSend;
	Ack1_8_Send = (Ack1_8_Send << 1) | fault[0].AckSend;
	
	Ack9_16_Send = Ack9_16_Send | fault[15].AckSend;
	Ack9_16_Send = (Ack9_16_Send << 1) | fault[14].AckSend;
	Ack9_16_Send = (Ack9_16_Send << 1) | fault[13].AckSend;
	Ack9_16_Send = (Ack9_16_Send << 1) | fault[12].AckSend;
	Ack9_16_Send = (Ack9_16_Send << 1) | fault[11].AckSend;
	Ack9_16_Send = (Ack9_16_Send << 1) | fault[10].AckSend;
	Ack9_16_Send = (Ack9_16_Send << 1) | fault[9].AckSend;
	Ack9_16_Send = (Ack9_16_Send << 1) | fault[8].AckSend;
	
	Ack17_24_Send = Ack17_24_Send | fault[23].AckSend;
   Ack17_24_Send = (Ack17_24_Send << 1) | fault[22].AckSend;
   Ack17_24_Send = (Ack17_24_Send << 1) | fault[21].AckSend;
   Ack17_24_Send = (Ack17_24_Send << 1) | fault[20].AckSend;
   Ack17_24_Send = (Ack17_24_Send << 1) | fault[19].AckSend;
   Ack17_24_Send = (Ack17_24_Send << 1) | fault[18].AckSend;
   Ack17_24_Send = (Ack17_24_Send << 1) | fault[17].AckSend;
   Ack17_24_Send = (Ack17_24_Send << 1) | fault[16].AckSend;
   
   Ack25_32_Send = Ack25_32_Send | fault[31].AckSend;
   Ack25_32_Send = (Ack25_32_Send << 1) | fault[30].AckSend;
   Ack25_32_Send = (Ack25_32_Send << 1) | fault[29].AckSend;
   Ack25_32_Send = (Ack25_32_Send << 1) | fault[28].AckSend;
   Ack25_32_Send = (Ack25_32_Send << 1) | fault[27].AckSend;
   Ack25_32_Send = (Ack25_32_Send << 1) | fault[26].AckSend;
   Ack25_32_Send = (Ack25_32_Send << 1) | fault[25].AckSend;
   Ack25_32_Send = (Ack25_32_Send << 1) | fault[24].AckSend;
   
   Ack33_40_Send = Ack33_40_Send | fault[39].AckSend;
   Ack33_40_Send = (Ack33_40_Send << 1) | fault[38].AckSend;
   Ack33_40_Send = (Ack33_40_Send << 1) | fault[37].AckSend;
   Ack33_40_Send = (Ack33_40_Send << 1) | fault[36].AckSend;
   Ack33_40_Send = (Ack33_40_Send << 1) | fault[35].AckSend;
   Ack33_40_Send = (Ack33_40_Send << 1) | fault[34].AckSend;
   Ack33_40_Send = (Ack33_40_Send << 1) | fault[33].AckSend;
   Ack33_40_Send = (Ack33_40_Send << 1) | fault[32].AckSend;
   
   Ack41_48_Send = Ack41_48_Send | fault[47].AckSend;
   Ack41_48_Send = (Ack41_48_Send << 1) | fault[46].AckSend;
   Ack41_48_Send = (Ack41_48_Send << 1) | fault[45].AckSend;
   Ack41_48_Send = (Ack41_48_Send << 1) | fault[44].AckSend;
   Ack41_48_Send = (Ack41_48_Send << 1) | fault[43].AckSend;
   Ack41_48_Send = (Ack41_48_Send << 1) | fault[42].AckSend;
   Ack41_48_Send = (Ack41_48_Send << 1) | fault[41].AckSend;
   Ack41_48_Send = (Ack41_48_Send << 1) | fault[40].AckSend;
   
   Ack49_56_Send = Ack49_56_Send | fault[55].AckSend;
   Ack49_56_Send = (Ack49_56_Send << 1) | fault[54].AckSend;
   Ack49_56_Send = (Ack49_56_Send << 1) | fault[53].AckSend;
   Ack49_56_Send = (Ack49_56_Send << 1) | fault[52].AckSend;
   Ack49_56_Send = (Ack49_56_Send << 1) | fault[51].AckSend;
   Ack49_56_Send = (Ack49_56_Send << 1) | fault[50].AckSend;
   Ack49_56_Send = (Ack49_56_Send << 1) | fault[49].AckSend;
   Ack49_56_Send = (Ack49_56_Send << 1) | fault[48].AckSend;
   
   Ack57_64_Send = Ack57_64_Send | fault[63].AckSend;
   Ack57_64_Send = (Ack57_64_Send << 1) | fault[62].AckSend;
   Ack57_64_Send = (Ack57_64_Send << 1) | fault[61].AckSend;
   Ack57_64_Send = (Ack57_64_Send << 1) | fault[60].AckSend;
   Ack57_64_Send = (Ack57_64_Send << 1) | fault[59].AckSend;
   Ack57_64_Send = (Ack57_64_Send << 1) | fault[58].AckSend;
   Ack57_64_Send = (Ack57_64_Send << 1) | fault[57].AckSend;
   Ack57_64_Send = (Ack57_64_Send << 1) | fault[56].AckSend;
   
   ///////////////////Fault/////////////////////////////////
   Input1_8_Send = Input1_8_Send | fault[7].In;
   Input1_8_Send = (Input1_8_Send << 1) | fault[6].In;
   Input1_8_Send = (Input1_8_Send << 1) | fault[5].In;
   Input1_8_Send = (Input1_8_Send << 1) | fault[4].In;
   Input1_8_Send = (Input1_8_Send << 1) | fault[3].In;
   Input1_8_Send = (Input1_8_Send << 1) | fault[2].In;
   Input1_8_Send = (Input1_8_Send << 1) | fault[1].In;
   Input1_8_Send = (Input1_8_Send << 1) | fault[0].In;
   
   Input9_16_Send = Input9_16_Send | fault[15].In;
   Input9_16_Send = (Input9_16_Send << 1) | fault[14].In;
   Input9_16_Send = (Input9_16_Send << 1) | fault[13].In;
   Input9_16_Send = (Input9_16_Send << 1) | fault[12].In;
   Input9_16_Send = (Input9_16_Send << 1) | fault[11].In;
   Input9_16_Send = (Input9_16_Send << 1) | fault[10].In;
   Input9_16_Send = (Input9_16_Send << 1) | fault[9].In;
   Input9_16_Send = (Input9_16_Send << 1) | fault[8].In;
   
   Input17_24_Send = Input17_24_Send | fault[23].In;
   Input17_24_Send = (Input17_24_Send << 1) | fault[22].In;
   Input17_24_Send = (Input17_24_Send << 1) | fault[21].In;
   Input17_24_Send = (Input17_24_Send << 1) | fault[20].In;
   Input17_24_Send = (Input17_24_Send << 1) | fault[19].In;
   Input17_24_Send = (Input17_24_Send << 1) | fault[18].In;
   Input17_24_Send = (Input17_24_Send << 1) | fault[17].In;
   Input17_24_Send = (Input17_24_Send << 1) | fault[16].In;
   
   Input25_32_Send = Input25_32_Send | fault[31].In;
   Input25_32_Send = (Input25_32_Send << 1) | fault[30].In;
   Input25_32_Send = (Input25_32_Send << 1) | fault[29].In;
   Input25_32_Send = (Input25_32_Send << 1) | fault[28].In;
   Input25_32_Send = (Input25_32_Send << 1) | fault[27].In;
   Input25_32_Send = (Input25_32_Send << 1) | fault[26].In;
   Input25_32_Send = (Input25_32_Send << 1) | fault[25].In;
   Input25_32_Send = (Input25_32_Send << 1) | fault[24].In;

   Input33_40_Send = Input33_40_Send | fault[39].In;
   Input33_40_Send = (Input33_40_Send << 1) | fault[38].In;
   Input33_40_Send = (Input33_40_Send << 1) | fault[37].In;
   Input33_40_Send = (Input33_40_Send << 1) | fault[36].In;
   Input33_40_Send = (Input33_40_Send << 1) | fault[35].In;
   Input33_40_Send = (Input33_40_Send << 1) | fault[34].In;
   Input33_40_Send = (Input33_40_Send << 1) | fault[33].In;
   Input33_40_Send = (Input33_40_Send << 1) | fault[32].In;
   
   Input41_48_Send = Input41_48_Send | fault[47].In;
   Input41_48_Send = (Input41_48_Send << 1) | fault[46].In;
   Input41_48_Send = (Input41_48_Send << 1) | fault[45].In;
   Input41_48_Send = (Input41_48_Send << 1) | fault[44].In;
   Input41_48_Send = (Input41_48_Send << 1) | fault[43].In;
   Input41_48_Send = (Input41_48_Send << 1) | fault[42].In;
   Input41_48_Send = (Input41_48_Send << 1) | fault[41].In;
   Input41_48_Send = (Input41_48_Send << 1) | fault[40].In;
   
   Input49_56_Send = Input49_56_Send | fault[55].In;
   Input49_56_Send = (Input49_56_Send << 1) | fault[54].In;
   Input49_56_Send = (Input49_56_Send << 1) | fault[53].In;
   Input49_56_Send = (Input49_56_Send << 1) | fault[52].In;
   Input49_56_Send = (Input49_56_Send << 1) | fault[51].In;
   Input49_56_Send = (Input49_56_Send << 1) | fault[50].In;
   Input49_56_Send = (Input49_56_Send << 1) | fault[49].In;
   Input49_56_Send = (Input49_56_Send << 1) | fault[48].In;
   
   Input57_64_Send = Input57_64_Send | fault[63].In;
   Input57_64_Send = (Input57_64_Send << 1) | fault[62].In;
   Input57_64_Send = (Input57_64_Send << 1) | fault[61].In;
   Input57_64_Send = (Input57_64_Send << 1) | fault[60].In;
   Input57_64_Send = (Input57_64_Send << 1) | fault[59].In;
   Input57_64_Send = (Input57_64_Send << 1) | fault[58].In;
   Input57_64_Send = (Input57_64_Send << 1) | fault[57].In;
   Input57_64_Send = (Input57_64_Send << 1) | fault[56].In;
	
}
void Send_Ouput(void)
{

	if(T_test == 0x00)
	{
		
		if(NoOfPoint >= 8)
		{
			Output1_8 = 0x00;
			Output1_8 = Output1_8 | fault[7].Output;
			Output1_8 = (Output1_8 << 1) | fault[6].Output;
			Output1_8 = (Output1_8 << 1) | fault[5].Output;
			Output1_8 = (Output1_8 << 1) | fault[4].Output;
			Output1_8 = (Output1_8 << 1) | fault[3].Output;
			Output1_8 = (Output1_8 << 1) | fault[2].Output;
			Output1_8 = (Output1_8 << 1) | fault[1].Output;
			Output1_8 = (Output1_8 << 1) | fault[0].Output;

			IO_OUTPUT_A(IO_DEVICE_0, Output1_8);	
		}
		if(NoOfPoint >= 16)
		{
			Output9_16 = 0x00;
			Output9_16 = Output9_16 | fault[15].Output;
			Output9_16 = (Output9_16 << 1) | fault[14].Output;
			Output9_16 = (Output9_16 << 1) | fault[13].Output;
			Output9_16 = (Output9_16 << 1) | fault[12].Output;
			Output9_16 = (Output9_16 << 1) | fault[11].Output;
			Output9_16 = (Output9_16 << 1) | fault[10].Output;
			Output9_16 = (Output9_16 << 1) | fault[9].Output;
			Output9_16 = (Output9_16 << 1) | fault[8].Output;

			IO_OUTPUT_A(IO_DEVICE_1, Output9_16);
		}

		 if(NoOfPoint >= 24)
		 {
			Output17_24 = 0x00;
			Output17_24 = Output17_24 | fault[23].Output;
			Output17_24 = (Output17_24 << 1) | fault[22].Output;
			Output17_24 = (Output17_24 << 1) | fault[21].Output;
			Output17_24 = (Output17_24 << 1) | fault[20].Output;
			Output17_24 = (Output17_24 << 1) | fault[19].Output;
			Output17_24 = (Output17_24 << 1) | fault[18].Output;
			Output17_24 = (Output17_24 << 1) | fault[17].Output;
			Output17_24 = (Output17_24 << 1) | fault[16].Output;

			IO_OUTPUT_A(IO_DEVICE_2, Output17_24);
		 }

		 if(NoOfPoint >= 32)
		 {
				Output25_32 = 0x00;
				Output25_32 = Output25_32 | fault[31].Output;
				Output25_32 = (Output25_32 << 1) | fault[30].Output;
				Output25_32 = (Output25_32 << 1) | fault[29].Output;
				Output25_32 = (Output25_32 << 1) | fault[28].Output;
				Output25_32 = (Output25_32 << 1) | fault[27].Output;
				Output25_32 = (Output25_32 << 1) | fault[26].Output;
				Output25_32 = (Output25_32 << 1) | fault[25].Output;
				Output25_32 = (Output25_32 << 1) | fault[24].Output;

				IO_OUTPUT_A(IO_DEVICE_3, Output25_32);
		 }
		 if(NoOfPoint >= 40)
		 {
				Output33_40 = 0x00;
				Output33_40 = Output33_40 | fault[39].Output;
				Output33_40 = (Output33_40 << 1) | fault[38].Output;
				Output33_40 = (Output33_40 << 1) | fault[37].Output;
				Output33_40 = (Output33_40 << 1) | fault[36].Output;
				Output33_40 = (Output33_40 << 1) | fault[35].Output;
				Output33_40 = (Output33_40 << 1) | fault[34].Output;
				Output33_40 = (Output33_40 << 1) | fault[33].Output;
				Output33_40 = (Output33_40 << 1) | fault[32].Output;

				IO_OUTPUT_A(IO_DEVICE_4, Output33_40);
		 }
		 if(NoOfPoint >= 48)
		 {
				Output41_48 = 0x00;
				Output41_48 = Output41_48 | fault[47].Output;
				Output41_48 = (Output41_48 << 1) | fault[46].Output;
				Output41_48 = (Output41_48 << 1) | fault[45].Output;
				Output41_48 = (Output41_48 << 1) | fault[44].Output;
				Output41_48 = (Output41_48 << 1) | fault[43].Output;
				Output41_48 = (Output41_48 << 1) | fault[42].Output;
				Output41_48 = (Output41_48 << 1) | fault[41].Output;
				Output41_48 = (Output41_48 << 1) | fault[40].Output;

				IO_OUTPUT_A(IO_DEVICE_5, Output41_48);
		 }
		 if(NoOfPoint >= 56)
		 {
				Output49_56 = 0x00;
				Output49_56 = Output49_56 | fault[55].Output;
				Output49_56 = (Output49_56 << 1) | fault[54].Output;
				Output49_56 = (Output49_56 << 1) | fault[53].Output;
				Output49_56 = (Output49_56 << 1) | fault[52].Output;
				Output49_56 = (Output49_56 << 1) | fault[51].Output;
				Output49_56 = (Output49_56 << 1) | fault[50].Output;
				Output49_56 = (Output49_56 << 1) | fault[19].Output;
				Output49_56 = (Output49_56 << 1) | fault[48].Output;

				IO_OUTPUT_A(IO_DEVICE_6, Output49_56);
		 }
		 if(NoOfPoint >= 64)
		 {
				Output57_64 = 0x00;
				Output57_64 = Output57_64 | fault[63].Output;
				Output57_64 = (Output57_64 << 1) | fault[62].Output;
				Output57_64 = (Output57_64 << 1) | fault[61].Output;
				Output57_64 = (Output57_64 << 1) | fault[60].Output;
				Output57_64 = (Output57_64 << 1) | fault[59].Output;
				Output57_64 = (Output57_64 << 1) | fault[58].Output;
				Output57_64 = (Output57_64 << 1) | fault[57].Output;
				Output57_64 = (Output57_64 << 1) | fault[56].Output;

				IO_OUTPUT_A(IO_DEVICE_7, Output57_64);
		 }
		 
	}

}
///////////////////////////////////////////////////////////////////
void ForceAllAlarm(void)
{
	for(uint8_t i = 0; i < NoOfPoint; i++)
	{
		fault[i].Input = ~fault[i].InputType;
		fault[i].Ack = 0 ;
	}
}
void StoreReleaseFault(void)
{
	
	
   for(unsigned char i = 0;i < NoOfPoint; i++)
   {
		 ///////   for refault when acksound only and fault return.
		 if(fault[i].FaultAgo == 1 && fault[i].FaultNow == 0 && fault[i].Ack == 0 && fault[i].Ack2 == 1 && fault[i].In == 1 )
		 {
			 fault[i].Ack2 = 0;
		 }
		 
		 
		  //////////  store fault   ///////////////
      fault[i].FaultAgo = fault[i].FaultNow;
   }
}

void delay_us(uint32_t us) { //about 800 nanosec
	volatile uint32_t counter = 3*us;
	while(counter--);
}

void CRC_Cal(unsigned char *puchMsg , unsigned char usDataLen)
{ 
   unsigned char uIndex ;                   /* fill index into CRC lookup table */
   unsigned char i ;

   CRC_Hi = 0xFF ;                      /* high byte of CRC initialized */
   CRC_Lo = 0xFF ;                      /* low byte of CRC initialized */

   for(i = 0;i < usDataLen;i++)
   {
      //restart_wdt();
      uIndex = CRC_Hi ^ (unsigned char) puchMsg[i] ;
      CRC_Hi = CRC_Lo ^ CRC_Table_Hi[uIndex] ;
      CRC_Lo = CRC_Table_Lo[uIndex] ;
   }
}
////////////////////////////////////////////////////////////////////////////////
void checkCommand(void)
{
   //restart_wdt();

   //if(sequence == end_sq && Address == SBUF)     //check Address
   if(sequence == end_sq )     //check Address
   {
     RxD_DataLen = 0x00 ;
     RxD_Buff[RxD_DataLen] = SBUF ;      //Byte 1   Address
     //restart_wdt();
     RxD_DataLen ++ ;
     sequence = addr_sq;
     T_timeout = 20; //200ms
   }
   else if(sequence == addr_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;      //Byte 2   Function Code
      //restart_wdt();
      RxD_DataLen ++ ;

       if(RxD_Buff[RxD_DataLen - 1] < 0x17)   //Function Code Must be less than 0x20
       {
           sequence = code_sq;
           T_timeout = 20; //200ms
       }
       else if(RxD_Buff[RxD_DataLen - 1] == 0x20)   /////Read Setting//////
       {
          sequence = ubyte_lo_sq;
          T_timeout = 20; //200ms
       }
       else if(RxD_Buff[RxD_DataLen - 1] == 0x21)   /////Write setting/////
       {
          sequence = byte_count_sq ;
          T_timeout = 20; //200ms
       }
       else                           // Invalid Code
       {
          RxD_DataLen = 0x00;
          sequence = end_sq;
          T_timeout = 0x00; 
          //output_bit(P485ctrl,0);
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);// 485 Controll
       }
   }
   else if(sequence == byte_count_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;      //Byte 3   Data Byte Count
      //restart_wdt();
      RxD_DataLen ++ ;
      Rxindex = RxD_Buff[RxD_DataLen - 1] ;    //Data Byte Count
      T_timeout = 20; //200ms
      sequence = data_sq ;
   }
   else if(sequence == data_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;      //
      //restart_wdt();
      RxD_DataLen ++ ;
      Rxindex -- ;                     //Data Byte Count
      if(Rxindex == 0x00)
      {
         sequence = ubyte_lo_sq ;      //next CRC
      }
      T_timeout = 20; //200ms
   }
   else if(sequence == code_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;      //Byte 3   Start address High Byte
      //restart_wdt();
      RxD_DataLen ++ ;
      sequence = start_addr_hi_sq;
      T_timeout = 20; //200ms

   }
   else if(sequence == start_addr_hi_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;         //Byte 4   Start address Low Byte
      //restart_wdt();
      RxD_DataLen ++ ;
      sequence = start_addr_lo_sq;
      T_timeout = 20; //200ms
   }
   else if(sequence == start_addr_lo_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;         //Byte 5   No. of point/force data High Byte
      //restart_wdt();
      RxD_DataLen ++ ;
      sequence = ubyte_hi_sq;
      T_timeout = 20; //200ms
   }
   else if(sequence == ubyte_hi_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;         //Byte 6   No. of point/force data Low Byte
      //restart_wdt();
      RxD_DataLen ++ ;
      sequence = ubyte_lo_sq;
      T_timeout = 20; //200ms
   }
   else if(sequence == ubyte_lo_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;         //Byte 7   CRC High Byte
      //restart_wdt();
      RxD_DataLen ++ ;
      sequence = crc_hi_sq;
      T_timeout = 20; //200ms
   }
   else if(sequence == crc_hi_sq)
   {
      RxD_Buff[RxD_DataLen] = SBUF ;         //Byte 8   CRC Low Byte
      //restart_wdt();
      sequence = end_sq;
      T_timeout = 0x20;
      //T_timeout = 0x14; //200ms
      recieve_completed = 1 ;            //Recieve completed then translate
      //output_bit(P485ctrl,0);
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);// 485 Controll
		 
		  USB_Count++;
   }
}


////////////////////////////////////////////////////////////////////////////////
///////////////////// MODBUS FUNCTION //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void Modbus_Function(void)
{
	//restart_wdt();
  CRC_Cal(RxD_Buff , RxD_DataLen - 1);

  if(CRC_Hi == RxD_Buff[RxD_DataLen - 1] && CRC_Lo == RxD_Buff[RxD_DataLen])
  {
		///////////////////////////////////////////////////////////////
    if(RxD_Buff[0] == Address)
    {
         /*-------------jack----*/
      if(RxD_Buff[1] == 0x07)///////////// READ COIL (esp code)////////////
      {
         Alarmtosend();
         //disable_interrupts(INT_TIMER2);
         //----------------------------------jj----------------------------//        
					Data_Buff[0] = Input1_8_Send ; //>> Start_Address ;
					Data_Buff[1] = Input9_16_Send ;// >> Start_Address ;
					Data_Buff[2] = Input17_24_Send ;// >> Start_Address ;
					Data_Buff[3] = Input25_32_Send ;
					Data_Buff[4] = Input33_40_Send ;
					Data_Buff[5] = Input41_48_Send ;
					Data_Buff[6] = Input49_56_Send ;
					Data_Buff[7] = Input57_64_Send ;
						
					Data_Buff[8] = Ack1_8_Send ;
					Data_Buff[9] = Ack9_16_Send ;
					Data_Buff[10] = Ack17_24_Send ;
					Data_Buff[11] = Ack25_32_Send ;
					Data_Buff[12] = Ack33_40_Send ;
					Data_Buff[13] = Ack41_48_Send ;
					Data_Buff[14] = Ack49_56_Send ;
					Data_Buff[15] = Ack57_64_Send ;

					TxD_Buff[0] = Address ;         //Address
					TxD_Buff[1] = 0x07 ;         //Function Code
					TxD_Buff[2] = Data_ByteCount=0x10;   //Byte Count
					TxD_Buff[3] = Data_Buff[0] ;      //first byte Data
					TxD_Buff[4] = Data_Buff[1] ;      //second byte Data
					TxD_Buff[5] = Data_Buff[2] ;  
					TxD_Buff[6] = Data_Buff[3] ;
					TxD_Buff[7] = Data_Buff[4] ;
					TxD_Buff[8] = Data_Buff[5] ;
					TxD_Buff[9] = Data_Buff[6] ;
					TxD_Buff[10] = Data_Buff[7] ;
						
				 TxD_Buff[11] = Data_Buff[8] ; //Ack1-8
				 TxD_Buff[12] = Data_Buff[9] ;
				 TxD_Buff[13] = Data_Buff[10] ;
				 TxD_Buff[14] = Data_Buff[11] ;
				 TxD_Buff[15] = Data_Buff[12] ;
				 TxD_Buff[16] = Data_Buff[13] ;
				 TxD_Buff[17] = Data_Buff[14] ;
				 TxD_Buff[18] = Data_Buff[15] ; //Ack57-64

				 CRC_Cal(TxD_Buff,19);            //Cal CRC 5 Byte

         TxD_Buff[19] = CRC_Hi ;
         TxD_Buff[20] = CRC_Lo ;
        
         HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
         delay_us(4);;

         HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 21,30);

         delay_us(3);
         HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
              
        //enable_interrupts(INT_TIMER2);        
         
       }

      else if(RxD_Buff[1] == 0x03)///////////// READ HOLDING REGISTER /////////////////////
      {
         //Do Read Input
         Start_Address = RxD_Buff[2] ;
         Start_Address = (Start_Address << 8) | RxD_Buff[3] ;   //Start Address 16 bit
         No_PointCount = RxD_Buff[4] ;
         No_PointCount = (No_PointCount << 8) | RxD_Buff[5] ;      //No. of Point 16 bit
				
				
				if(No_PointCount ==1)
				{
					 TxD_Buff[0] = Address ;         //Address
					 TxD_Buff[1] = 0x03 ;         //Function Code
					 TxD_Buff[2] = 2;   //Byte Count
					 TxD_Buff[3] = Input9_16 ;   
					 TxD_Buff[4] = Input1_8 ; 

					 CRC_Cal(TxD_Buff,5)   ;            //Cal CRC 4 Byte
				 
				
					 TxD_Buff[5] = CRC_Hi ;
					 TxD_Buff[6] = CRC_Lo ;
					 
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
					 delay_us(4);

					 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 7,30);
					 
					 delay_us(3);
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
				}
				else if(No_PointCount ==2)
				{
					 TxD_Buff[0] = Address ;         //Address
					 TxD_Buff[1] = 0x03 ;         //Function Code
					 TxD_Buff[2] = 4;   //Byte Count
					 TxD_Buff[3] = Input9_16 ;   
					 TxD_Buff[4] = Input1_8 ; 
					 TxD_Buff[5] = Input25_32 ;   
					 TxD_Buff[6] = Input17_24 ; 					

					 CRC_Cal(TxD_Buff,7)   ;            //Cal CRC 4 Byte
				 
				
					 TxD_Buff[7] = CRC_Hi ;
					 TxD_Buff[8] = CRC_Lo ;
					 
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
					 delay_us(4);

					 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 9,30);
					 
					 delay_us(3);
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
				}
				else if(No_PointCount ==3)
				{
					 TxD_Buff[0] = Address ;         //Address
					 TxD_Buff[1] = 0x03 ;         //Function Code
					 TxD_Buff[2] = 6;   //Byte Count
					 TxD_Buff[3] = Input9_16 ;   
					 TxD_Buff[4] = Input1_8 ; 
					 TxD_Buff[5] = Input25_32 ;   
					 TxD_Buff[6] = Input17_24 ;  		
					 TxD_Buff[7] = Input41_48 ;   
					 TxD_Buff[8] = Input33_40; 				

					 CRC_Cal(TxD_Buff,9)   ;            //Cal CRC 4 Byte
				 
				
					 TxD_Buff[9] = CRC_Hi ;
					 TxD_Buff[10] = CRC_Lo ;
					 
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
					 delay_us(4);

					 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 11,30);
					 
					 delay_us(3);
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
				}
				else if(No_PointCount ==4)
				{
					 TxD_Buff[0] = Address ;         //Address
					 TxD_Buff[1] = 0x03 ;         //Function Code
					 TxD_Buff[2] = 8;   //Byte Count
					 TxD_Buff[3] = Input9_16 ;   
					 TxD_Buff[4] = Input1_8 ; 
					 TxD_Buff[5] = Input25_32 ;   
					 TxD_Buff[6] = Input17_24 ;  		
					 TxD_Buff[7] = Input41_48 ;   
					 TxD_Buff[8] = Input33_40; 	
					 TxD_Buff[9] = Input57_64 ;   
					 TxD_Buff[10] = Input49_56 ; 			

					 CRC_Cal(TxD_Buff,11)   ;            //Cal CRC 4 Byte
				 
				
					 TxD_Buff[11] = CRC_Hi ;
					 TxD_Buff[12] = CRC_Lo ;
					 
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
					 delay_us(4);

					 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 13,30);
					 
					 delay_us(3);
					 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
				}
				else
         {
            //invalid parameter
            TxD_Buff[0] = Address ;         //Address
            TxD_Buff[1] = 0x81 ;         //Function Code
            TxD_Buff[2] = 0x02 ;         //illegal data address

            CRC_Cal(TxD_Buff,3)   ;            //Cal CRC 3 Byte

            TxD_Buff[3] = CRC_Hi ;
            TxD_Buff[4] = CRC_Lo ;
                
            HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
            delay_us(4);

            HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 5,30);

            delay_us(3);
            HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
 
         }
				/*
         if(Start_Address < 0x01 && (Start_Address + No_PointCount) < 0x02)  //Valid point 0
         {
            if(No_PointCount < 2) Data_ByteCount = 0x02 ;

               if(Start_Address < 0x01)
               {
                  
                 Data_Buff[1] = Input1_8;
                 Data_Buff[0] = Input9_16;
                                  
               }

               TxD_Buff[0] = Address ;         //Address
               TxD_Buff[1] = 0x03 ;         //Function Code
               TxD_Buff[2] = Data_ByteCount ;   //Byte Count
               TxD_Buff[3] = Data_Buff[0] ;   
               TxD_Buff[4] = Data_Buff[1] ; 

               CRC_Cal(TxD_Buff,5)   ;            //Cal CRC 4 Byte
             
            
               TxD_Buff[5] = CRC_Hi ;
               TxD_Buff[6] = CRC_Lo ;
               
               HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
               delay_us(4);

               HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 7,30);
							 
               delay_us(3);
               HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0

         }
				 
         else
         {
            //invalid parameter
            TxD_Buff[0] = Address ;         //Address
            TxD_Buff[1] = 0x81 ;         //Function Code
            TxD_Buff[2] = 0x02 ;         //illegal data address

            CRC_Cal(TxD_Buff,3)   ;            //Cal CRC 3 Byte

            TxD_Buff[3] = CRC_Hi ;
            TxD_Buff[4] = CRC_Lo ;
                
            HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
            delay_us(4);

            HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 5,30);

            delay_us(3);
            HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
 
         }
					*/
      }
			//-----------------------------jj-----------------------------------------//
       else if(RxD_Buff[1] == 0x01)///////////// FORCE COIL /////////////////////
			 {
					Start_Address = RxD_Buff[2] ;
          Start_Address = (Start_Address << 8) | RxD_Buff[3] ;   //Start Address 16 bit
          No_PointCount = RxD_Buff[4] ;
          No_PointCount = (No_PointCount << 8) | RxD_Buff[5] ;      //No. of Point 16 bit
				 if(No_PointCount <= 64)//check valid data
				 {
					 if(No_PointCount < 9)
					{
						 Data_ByteCount = 0x01 ;
						
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;   
						 

						 CRC_Cal(TxD_Buff,4)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[4] = CRC_Hi ;
						 TxD_Buff[5] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 6,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
						 
					 }
           else if(No_PointCount < 17)
					 {
						 Data_ByteCount = 0x02 ;
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;  
						 TxD_Buff[4] = Input9_16 ;  						 
						 

						 CRC_Cal(TxD_Buff,5)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[5] = CRC_Hi ;
						 TxD_Buff[6] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 7,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
					 }
           else if(No_PointCount < 25) 
					 {
						 Data_ByteCount = 0x03 ;
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;  
						 TxD_Buff[4] = Input9_16 ; 
						 TxD_Buff[5] = Input17_24 ;  						 
						 

						 CRC_Cal(TxD_Buff,6)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[6] = CRC_Hi ;
						 TxD_Buff[7] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
					 }
					 else if(No_PointCount < 33)
					 {
						 Data_ByteCount = 0x04 ;
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;  
						 TxD_Buff[4] = Input9_16 ; 
						 TxD_Buff[5] = Input17_24 ;  
						 TxD_Buff[6] = Input25_32 ; 						 
						 

						 CRC_Cal(TxD_Buff,7)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[7] = CRC_Hi ;
						 TxD_Buff[8] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 9,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
					 }
					 else if(No_PointCount < 41)
					 {
						 Data_ByteCount = 0x05 ;
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;  
						 TxD_Buff[4] = Input9_16 ; 
						 TxD_Buff[5] = Input17_24 ;  
						 TxD_Buff[6] = Input25_32 ; 
						 TxD_Buff[7] = Input33_40 ; 
						 

						 CRC_Cal(TxD_Buff,8)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[8] = CRC_Hi ;
						 TxD_Buff[9] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 10,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
					 }
					 else if(No_PointCount < 49)
					 {
						 Data_ByteCount = 0x06 ;
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;  
						 TxD_Buff[4] = Input9_16 ; 
						 TxD_Buff[5] = Input17_24 ;  
						 TxD_Buff[6] = Input25_32 ; 
						 TxD_Buff[7] = Input33_40 ; 
						 TxD_Buff[8] = Input41_48 ; 
						 

						 CRC_Cal(TxD_Buff,9)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[9] = CRC_Hi ;
						 TxD_Buff[10] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 11,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
					 }
					 else if(No_PointCount < 57)
					 {
							Data_ByteCount = 0x07 ;
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;  
						 TxD_Buff[4] = Input9_16 ; 
						 TxD_Buff[5] = Input17_24 ;  
						 TxD_Buff[6] = Input25_32 ; 
						 TxD_Buff[7] = Input33_40 ; 
						 TxD_Buff[8] = Input41_48 ; 
						 TxD_Buff[9] = Input49_56 ; 
						 

						 CRC_Cal(TxD_Buff,10)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[10] = CRC_Hi ;
						 TxD_Buff[11] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 12,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
					 }
					 else if(No_PointCount < 65)
					 {
						 Data_ByteCount = 0x08 ;
						 TxD_Buff[0] = Address ;         //Address
						 TxD_Buff[1] = 0x01 ;         //Function Code
						 TxD_Buff[2] = Data_ByteCount;   //Byte Count
						 TxD_Buff[3] = Input1_8 ;  
						 TxD_Buff[4] = Input9_16 ; 
						 TxD_Buff[5] = Input17_24 ;  
						 TxD_Buff[6] = Input25_32 ; 
						 TxD_Buff[7] = Input33_40 ; 
						 TxD_Buff[8] = Input41_48 ; 
						 TxD_Buff[9] = Input49_56 ; 
						 TxD_Buff[10] = Input57_64 ; 

						 CRC_Cal(TxD_Buff,11)   ;            //Cal CRC 4 Byte
					 
					
						 TxD_Buff[11] = CRC_Hi ;
						 TxD_Buff[12] = CRC_Lo ;
						 
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
						 delay_us(4);

						 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 13,30);
						 
						 delay_us(3);
						 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
					 }
				 }
				 else
         {
            //invalid parameter
            TxD_Buff[0] = Address ;         //Address
            TxD_Buff[1] = 0x81 ;         //Function Code
            TxD_Buff[2] = 0x02 ;         //illegal data address

            CRC_Cal(TxD_Buff,3)   ;            //Cal CRC 3 Byte

            TxD_Buff[3] = CRC_Hi ;
            TxD_Buff[4] = CRC_Lo ;
                
            HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
            delay_us(4);

            HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 5,30);

            delay_us(3);
            HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
 
         }
				 
				 
				 
			 }
      //-----------------------------jj-----------------------------------------//
      //-----------------------------jj-----------------------------------------//
      //-----------------------------jj-----------------------------------------//
       else if(RxD_Buff[1] == 0x05)///////////// FORCE COIL /////////////////////
       {
            //Do Force Coil
            Start_Address = RxD_Buff[2] ;
            Start_Address = (Start_Address << 8) | RxD_Buff[3] ;   //Coil Address 16 bit
            No_PointCount = RxD_Buff[4] ;
            No_PointCount = (No_PointCount << 8) | RxD_Buff[5] ;   //Force Data 16 bit FF00 = ON, 00FF = OFF

            if(Start_Address == 0x00)   ////// Acknowlegde //////
            {
               if(No_PointCount == 0xFF00)   //ON
               {
									//Modbus_ACK = 1 ;
									AutoAckFlag = 1;

									TxD_Buff[0] = Address ;         //Address
									TxD_Buff[1] = 0x05 ;         //Function Code
									TxD_Buff[2] = RxD_Buff[2] ;      //Coil Address Hi
									TxD_Buff[3] = RxD_Buff[3] ;      //Coil Address Lo
									TxD_Buff[4] = RxD_Buff[4] ;      //Force Data Hi
									TxD_Buff[5] = RxD_Buff[5] ;      //Force Data Lo

									CRC_Cal(TxD_Buff,6);            //Cal CRC 6 Byte

									TxD_Buff[6] = CRC_Hi ;
									TxD_Buff[7] = CRC_Lo ;

								 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
								 delay_us(4);
		 
								 HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);
		 
								 delay_us(3);
								 HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
              
               }
            }
            else if(Start_Address == 0x01)   ///// Reset //////
            {
               if(No_PointCount == 0xFF00)   //ON
               {
                  //Modbus_RET = 1 ;
                  AutoResetFlag = 1;
                  
                  TxD_Buff[0] = Address ;         //Address
                  TxD_Buff[1] = 0x05 ;         //Function Code
                  TxD_Buff[2] = RxD_Buff[2] ;      //Coil Address Hi
                  TxD_Buff[3] = RxD_Buff[3] ;      //Coil Address Lo
                  TxD_Buff[4] = RxD_Buff[4] ;      //Force Data Hi
                  TxD_Buff[5] = RxD_Buff[5] ;      //Force Data Lo

                  CRC_Cal(TxD_Buff,6)   ;            //Cal CRC 6 Byte

                  TxD_Buff[6] = CRC_Hi ;
                  TxD_Buff[7] = CRC_Lo ;

									HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
                  delay_us(4);

                  HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);

                  delay_us(3);
                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
              
               }
            }
            else if(Start_Address == 0x02)   ///// Test //////
            {
               if(No_PointCount == 0xFF00)   //ON
               {
                  //Modbus_Lamp_Test = 1 ;
                  AutoTestFlag = 1;

                  TxD_Buff[0] = Address ;         //Address
                  TxD_Buff[1] = 0x05 ;         //Function Code
                  TxD_Buff[2] = RxD_Buff[2] ;      //Coil Address Hi
                  TxD_Buff[3] = RxD_Buff[3] ;      //Coil Address Lo
                  TxD_Buff[4] = RxD_Buff[4] ;      //Force Data Hi
                  TxD_Buff[5] = RxD_Buff[5] ;      //Force Data Lo

                  CRC_Cal(TxD_Buff,6)   ;            //Cal CRC 6 Byte

                  TxD_Buff[6] = CRC_Hi ;
                  TxD_Buff[7] = CRC_Lo ;

                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
                  HAL_Delay(4);

                  HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);
									
                  HAL_Delay(3);
                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
             
               }
               else if(No_PointCount == 0x00)   //OFF
               {
                  //Modbus_Lamp_Test = 0 ;
                  AutoTestFlag = 0;
                  Read_input();
                  AutoAckFlag = 1;
                  check_ack();
								  check_ack2();
                  AutoResetFlag = 1;
                  check_reset();

                  TxD_Buff[0] = Address ;         //Address
                  TxD_Buff[1] = 0x05 ;         //Function Code
                  TxD_Buff[2] = RxD_Buff[2] ;      //Coil Address Hi
                  TxD_Buff[3] = RxD_Buff[3] ;      //Coil Address Lo
                  TxD_Buff[4] = RxD_Buff[4] ;      //Force Data Hi
                  TxD_Buff[5] = RxD_Buff[5] ;      //Force Data Lo

                  CRC_Cal(TxD_Buff,6)   ;            //Cal CRC 6 Byte

                  TxD_Buff[6] = CRC_Hi ;
                  TxD_Buff[7] = CRC_Lo ;

                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
                  delay_us(4);

                  HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);

                  delay_us(3);
                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
                  
               }
            }
            else if(Start_Address == 0x03)   ///// Function Test //////
            {
               if(No_PointCount == 0xFF00)   //ON
               {
                  //Modbus_Lamp_Test = 1 ;
                  Test_fault = 1 ;

                  TxD_Buff[0] = Address ;         //Address
                  TxD_Buff[1] = 0x05 ;         //Function Code
                  TxD_Buff[2] = RxD_Buff[2] ;      //Coil Address Hi
                  TxD_Buff[3] = RxD_Buff[3] ;      //Coil Address Lo
                  TxD_Buff[4] = RxD_Buff[4] ;      //Force Data Hi
                  TxD_Buff[5] = RxD_Buff[5] ;      //Force Data Lo

                  CRC_Cal(TxD_Buff,6)   ;            //Cal CRC 6 Byte

                  TxD_Buff[6] = CRC_Hi ;
                  TxD_Buff[7] = CRC_Lo ;
                  
                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
                  delay_us(4);

                  HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);

                  delay_us(10);
                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0

               }
               else if(No_PointCount == 0x00)   //OFF
               {
                  //Modbus_Lamp_Test = 0 ;
                  Test_fault = 0;
                  Read_input();
                  AutoAckFlag = 1;
                  check_ack();
								  check_ack2();
                  AutoResetFlag = 1;
                  check_reset();

                  TxD_Buff[0] = Address ;         //Address
                  TxD_Buff[1] = 0x05 ;         //Function Code
                  TxD_Buff[2] = RxD_Buff[2] ;      //Coil Address Hi
                  TxD_Buff[3] = RxD_Buff[3] ;      //Coil Address Lo
                  TxD_Buff[4] = RxD_Buff[4] ;      //Force Data Hi
                  TxD_Buff[5] = RxD_Buff[5] ;      //Force Data Lo

                  CRC_Cal(TxD_Buff,6)   ;            //Cal CRC 6 Byte

                  TxD_Buff[6] = CRC_Hi ;
                  TxD_Buff[7] = CRC_Lo ;

                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
                  delay_us(4);

                  HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);

                  delay_us(3);
                  HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0
                  
               }
            }
            else if(Start_Address == 0x64)   ///// Change Modbus Addr //////
            {
               Address = No_PointCount;
               //write_eeprom(0x60,Address);   //Communication Address
							 Write_Flash();
               
               TxD_Buff[0] = Address ;         //Address
               TxD_Buff[1] = 0x05 ;         //Function Code
               TxD_Buff[2] = RxD_Buff[2] ;      //Coil Address Hi
               TxD_Buff[3] = RxD_Buff[3] ;      //Coil Address Lo
               TxD_Buff[4] = RxD_Buff[4] ;      //Force Data Hi
               TxD_Buff[5] = RxD_Buff[5] ;      //Force Data Lo

               CRC_Cal(TxD_Buff,6)   ;            //Cal CRC 6 Byte

               TxD_Buff[6] = CRC_Hi ;
               TxD_Buff[7] = CRC_Lo ;

               HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_SET);// 485 Controll 1
               delay_us(4);

               HAL_UART_Transmit(&huart1,(uint8_t*)TxD_Buff, 8,30);

               delay_us(3);
               HAL_GPIO_WritePin(CTRL485_port,P485ctrl,GPIO_PIN_RESET);// 485 Controll 0

            }
         }
      
     }
   
		Send_check_Time = 500; //5 Second
	}
	recieve_completed = 0 ;
	sequence = end_sq ;
  T_timeout = 0x00;
  RxD_DataLen = 0x00 ;
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);// 485 Control	
}

void checkSetting(void)
{
	if((UserRxBufferFS[0] == 0xAA || UserRxBufferFS[0] == Address)&&(UserRxBufferFS[1] == 0x20))//Read Setting
	{
		T_timeout = 100;
		RxD_DataLen = 3;//4 byte
		CRC_Cal(UserRxBufferFS , RxD_DataLen - 1);
		if((CRC_Hi == UserRxBufferFS[RxD_DataLen - 1]) && (CRC_Lo == UserRxBufferFS[RxD_DataLen]))
    {			
			USB_buff[0] = Address ;         //Address
			USB_buff[1] = 0x20 ;            //function code
			USB_buff[2] = 0x2D ;//45 Byte   //Data Byte count
			USB_buff[3] = InputType1_8 ;
			USB_buff[4] = InputType9_16 ;
			USB_buff[5] = InputType17_24 ;
			USB_buff[6] = InputType25_32 ;
			USB_buff[7] = InputType33_40 ;
			USB_buff[8] = InputType41_48 ;
			USB_buff[9] = InputType49_56 ;
			USB_buff[10] = InputType57_64 ;
			USB_buff[11] = FaultType1_8 ;
			USB_buff[12] = FaultType9_16 ;
			USB_buff[13] = FaultType17_24 ;
			USB_buff[14] = FaultType25_32 ;
			USB_buff[15] = FaultType33_40 ;
			USB_buff[16] = FaultType41_48 ;
			USB_buff[17] = FaultType49_56 ;
			USB_buff[18] = FaultType57_64 ;
			USB_buff[19] = OutputType1_8 ;
			USB_buff[20] = OutputType9_16 ;
			USB_buff[21] = OutputType17_24 ;
			USB_buff[22] = OutputType25_32 ;
			USB_buff[23] = OutputType33_40 ;
			USB_buff[24] = OutputType41_48 ;
			USB_buff[25] = OutputType49_56 ;
			USB_buff[26] = OutputType57_64 ;
			USB_buff[27] = OutputBoth1_8 ;
			USB_buff[28] = OutputBoth9_16 ;
			USB_buff[29] = OutputBoth17_24 ;
			USB_buff[30] = OutputBoth25_32 ;
			USB_buff[31] = OutputBoth33_40 ;
			USB_buff[32] = OutputBoth41_48 ;
			USB_buff[33] = OutputBoth49_56 ;
			USB_buff[34] = OutputBoth57_64 ;
			USB_buff[35] = Alarm_Indicator1_8 ;
			USB_buff[36] = Alarm_Indicator9_16 ;
			USB_buff[37] = Alarm_Indicator17_24 ;
			USB_buff[38] = Alarm_Indicator25_32 ;
			USB_buff[39] = Alarm_Indicator33_40 ;
			USB_buff[40] = Alarm_Indicator41_48 ;
			USB_buff[41] = Alarm_Indicator49_56 ;
			USB_buff[42] = Alarm_Indicator57_64 ;
			USB_buff[43] = AutoAck ; //41
			USB_buff[44] = AutoAckTime ; //42
			USB_buff[45] = FlashingRate ; //43
			USB_buff[46] = NoOfPoint ; //44
			USB_buff[47] = Address ; //45
			
			for(uint8_t i = 0; i < 64; i++)
			{
				USB_buff[48 + i] = fault[i].FaultDelayTime  ;
			}
			
			/*
			USB_buff[48] = FaultDelayTime[1];
			USB_buff[49] = FaultDelayTime[2];
			USB_buff[50] = FaultDelayTime[3];
			USB_buff[51] = FaultDelayTime[4];
			USB_buff[52] = FaultDelayTime[5];
			USB_buff[53] = FaultDelayTime[6];
			USB_buff[54] = FaultDelayTime[7];
			USB_buff[55] = FaultDelayTime[8];
			USB_buff[56] = FaultDelayTime[9];
			USB_buff[57] = FaultDelayTime[10];
			
			USB_buff[58] = FaultDelayTime[11];
			USB_buff[59] = FaultDelayTime[12];
			USB_buff[60] = FaultDelayTime[13];
			USB_buff[61] = FaultDelayTime[14];
			USB_buff[62] = FaultDelayTime[15];
			USB_buff[63] = FaultDelayTime[16];
			USB_buff[64] = FaultDelayTime[17];
			USB_buff[65] = FaultDelayTime[18];
			USB_buff[66] = FaultDelayTime[19];
			USB_buff[67] = FaultDelayTime[20];
			
			USB_buff[68] = FaultDelayTime[21];
			USB_buff[69] = FaultDelayTime[22];
			USB_buff[70] = FaultDelayTime[23];
			USB_buff[71] = FaultDelayTime[24];
			USB_buff[72] = FaultDelayTime[25];
			USB_buff[73] = FaultDelayTime[26];
			USB_buff[74] = FaultDelayTime[27];
			USB_buff[75] = FaultDelayTime[28];
			USB_buff[76] = FaultDelayTime[29];
			USB_buff[77] = FaultDelayTime[30];
			
			USB_buff[78] = FaultDelayTime[31];
			USB_buff[79] = FaultDelayTime[32];
			USB_buff[80] = FaultDelayTime[33];
			USB_buff[81] = FaultDelayTime[34];
			USB_buff[82] = FaultDelayTime[35];
			USB_buff[83] = FaultDelayTime[36];
			USB_buff[84] = FaultDelayTime[37];
			USB_buff[85] = FaultDelayTime[38];
			USB_buff[86] = FaultDelayTime[39];
			USB_buff[87] = FaultDelayTime[40];
			
			USB_buff[88] = FaultDelayTime[41];
			USB_buff[89] = FaultDelayTime[42];
			USB_buff[90] = FaultDelayTime[43];
			USB_buff[91] = FaultDelayTime[44];
			USB_buff[92] = FaultDelayTime[45];
			USB_buff[93] = FaultDelayTime[46];
			USB_buff[94] = FaultDelayTime[47];
			USB_buff[95] = FaultDelayTime[48];
			USB_buff[96] = FaultDelayTime[49];
			USB_buff[97] = FaultDelayTime[50];
			
			USB_buff[98] = FaultDelayTime[51];
			USB_buff[99] = FaultDelayTime[52];
			USB_buff[100] = FaultDelayTime[53];
			USB_buff[101] = FaultDelayTime[54];
			USB_buff[102] = FaultDelayTime[55];
			USB_buff[103] = FaultDelayTime[56];
			USB_buff[104] = FaultDelayTime[57];
			USB_buff[105] = FaultDelayTime[58];
			USB_buff[106] = FaultDelayTime[59];
			USB_buff[107] = FaultDelayTime[60];
			
			USB_buff[108] = FaultDelayTime[61];
			USB_buff[109] = FaultDelayTime[62];
			USB_buff[110] = FaultDelayTime[63];
			USB_buff[111] = FaultDelayTime[64];
			*/
			
			USB_buff[112] = ReFaultEn;
			USB_buff[113] = RefaultTime;


//			CRC_Cal(USB_buff,114) ;            //Cal CRC 113 byte

//			USB_buff[114] = CRC_Hi ;
//			USB_buff[115] = CRC_Lo ;
//			
//			CDC_Transmit_FS((uint8_t*)USB_buff, 117); //117 CHAR SEND

			USB_buff[114] = RingbackEn;
			
			CRC_Cal(USB_buff,115) ;            //Cal CRC 113 byte

			USB_buff[115] = CRC_Hi ;
			USB_buff[116] = CRC_Lo ;
			
			CDC_Transmit_FS((uint8_t*)USB_buff, 118); //117 CHAR SEND
			
			memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS)); //clear array data
						 
		}
	}
	else if((UserRxBufferFS[0] == 0xAA || UserRxBufferFS[0] == Address) && (UserRxBufferFS[1] == 0x21))//Write Setting State1
	{
		RxD_DataLen = 50;//115 byte

		CRC_Cal(UserRxBufferFS , RxD_DataLen - 1);
		if((CRC_Hi == UserRxBufferFS[RxD_DataLen - 1]) && (CRC_Lo == UserRxBufferFS[RxD_DataLen]))
    {
			USB_State =1;	
			Cal_Data1();
				
			memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS)); //clear array data
		}
	}
	else if((UserRxBufferFS[0] == 0xAA || UserRxBufferFS[0] == Address) && (UserRxBufferFS[1] == 0x22))//Write Setting State2
	{
		USB_State =4;
		RxD_DataLen = 34;//115 byte

		CRC_Cal(UserRxBufferFS , RxD_DataLen - 1);
		if((CRC_Hi == UserRxBufferFS[RxD_DataLen - 1]) && (CRC_Lo == UserRxBufferFS[RxD_DataLen]))
    {
			USB_State =2;
			Cal_Data2();	
				
			memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS)); //clear array data
		}
	}
	else if((UserRxBufferFS[0] == 0xAA || UserRxBufferFS[0] == Address) && (UserRxBufferFS[1] == 0x23))//Write Setting State3
	{
		USB_State =6;
		//RxD_DataLen = 40;//115 byte
		RxD_DataLen = 41;//115 byte

		CRC_Cal(UserRxBufferFS , RxD_DataLen - 1);
		if((CRC_Hi == UserRxBufferFS[RxD_DataLen - 1]) && (CRC_Lo == UserRxBufferFS[RxD_DataLen]))
    {
			USB_buff[0] = Address ;
			USB_buff[1] = 0x21 ;
			CRC_Cal(USB_buff,2);
			USB_buff[2] = CRC_Hi ;
			USB_buff[3] = CRC_Lo ;
			
			CDC_Transmit_FS((uint8_t*)USB_buff, 4); //4 CHAR SEND
			
			
			Cal_Data3();
			
			Write_Flash();
			Read_Config();
			USB_State =3;
			refaultindex();
			//USB_State =3;		
			memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS)); //clear array data
		}
	}
	else if((UserRxBufferFS[0] == 0xAA || UserRxBufferFS[0] == Address) && (UserRxBufferFS[1] == 0x05))//Chang Address
	{
		if((UserRxBufferFS[2] == 0x00) && (UserRxBufferFS[3] == 0x64)&& (UserRxBufferFS[4] == 0x00))
		{
			Address = UserRxBufferFS[5];
			Write_Flash();
			
			USB_buff[0] = Address ;
			USB_buff[1] = 0x05 ;
			USB_buff[2] = UserRxBufferFS[2];
			USB_buff[3] = UserRxBufferFS[3];
			USB_buff[4] = UserRxBufferFS[4];
			USB_buff[5] = UserRxBufferFS[5];
			
			CRC_Cal(USB_buff,6);
			
			USB_buff[6] = CRC_Hi ;
			USB_buff[7] = CRC_Lo ;
			
			CDC_Transmit_FS((uint8_t*)USB_buff, 8); //8 CHAR SEND
			
			memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS)); //clear array data
		}	
	}
	
}

void Cal_Data1 (void)
{
		Flashindex =0;
				
		Flashdata[Flashindex] = 0x0F; Flashindex++; // 800
		
		Flashdata[Flashindex] = (InputType1_8 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 801
		Flashdata[Flashindex] = (InputType9_16 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 802
		Flashdata[Flashindex] = (InputType17_24 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 803
		Flashdata[Flashindex] = (InputType25_32 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 804
		Flashdata[Flashindex] = (InputType33_40 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 805
		Flashdata[Flashindex] = (InputType41_48 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 806
		Flashdata[Flashindex] = (InputType49_56 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 807
		Flashdata[Flashindex] = (InputType57_64 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 808
		
		Flashdata[Flashindex] = (FaultType1_8 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 809
		Flashdata[Flashindex] = (FaultType9_16 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 80A
		Flashdata[Flashindex] = (FaultType17_24 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 80B
		Flashdata[Flashindex] = (FaultType25_32 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 80C
		Flashdata[Flashindex] = (FaultType33_40 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 80D
		Flashdata[Flashindex] = (FaultType41_48 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 80E
		Flashdata[Flashindex] = (FaultType49_56 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 80F
		Flashdata[Flashindex] = (FaultType57_64 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 810
		
		Flashdata[Flashindex] = (OutputType1_8 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 811
		Flashdata[Flashindex] = (OutputType9_16 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 812
		Flashdata[Flashindex] = (OutputType17_24 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 813
		Flashdata[Flashindex] = (OutputType25_32 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 814
		Flashdata[Flashindex] = (OutputType33_40 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 815
		Flashdata[Flashindex] = (OutputType41_48 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 816
		Flashdata[Flashindex] = (OutputType49_56 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 817
		Flashdata[Flashindex] = (OutputType57_64 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 818
		
		Flashdata[Flashindex] = (OutputBoth1_8 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 819
		Flashdata[Flashindex] = (OutputBoth9_16 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 81A
		Flashdata[Flashindex] = (OutputBoth17_24 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 81B
		Flashdata[Flashindex] = (OutputBoth25_32 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 81C
		Flashdata[Flashindex] = (OutputBoth33_40 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 81D
		Flashdata[Flashindex] = (OutputBoth41_48 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 81E
		Flashdata[Flashindex] = (OutputBoth49_56 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 81F
		Flashdata[Flashindex] = (OutputBoth57_64 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 820
		
		Flashdata[Flashindex] = (Alarm_Indicator1_8 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 821
		Flashdata[Flashindex] = (Alarm_Indicator9_16 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 822
		Flashdata[Flashindex] = (Alarm_Indicator17_24 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 823
		Flashdata[Flashindex] = (Alarm_Indicator25_32 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 824
		Flashdata[Flashindex] = (Alarm_Indicator33_40 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 825
		Flashdata[Flashindex] = (Alarm_Indicator41_48 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 826
		Flashdata[Flashindex] = (Alarm_Indicator49_56 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 827
		Flashdata[Flashindex] = (Alarm_Indicator57_64 = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 828
		
		Flashdata[Flashindex] = (AutoAck = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 829
		Flashdata[Flashindex] = (AutoAckTime = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 82A
		Flashdata[Flashindex] = (FlashingRate = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 82B
		RingbackTime = FlashingRate*2;
		Flashdata[Flashindex] = (NoOfPoint = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 82C
		Flashdata[Flashindex] = (MasterSlaveSync = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 82D
		Flashdata[Flashindex] = (Address = UserRxBufferFS[2+Flashindex]) ; Flashindex++; // 82E
}

void Cal_Data2 (void)
{
	for(uint8_t i = 0; i < 30; i++)
	{
		//Flashdata[i + 0x2F] = (FaultDelayTime[i + 1] = UserRxBufferFS[3+i]) ;  //fault[i]
		Flashdata[i + 0x2F] = (fault[i].FaultDelayTime = UserRxBufferFS[3+i]) ;
	}	
}
void Cal_Data3 (void)
{
	uint8_t i ;
	for(i = 0; i < 34; i++)
	{
		//Flashdata[i + 0x4D] = (FaultDelayTime[i + 31] = UserRxBufferFS[3+i]) ; //fault[i].
		Flashdata[i + 0x4D] = (fault[i + 31].FaultDelayTime = UserRxBufferFS[3+i]) ;
	}
	Flashdata[i + 0x4D] = (ReFaultEn = UserRxBufferFS[3 + i]) ; i++; // 86F
	Flashdata[i + 0x4D] = (RefaultTime = UserRxBufferFS[3 + i]) ; i++; // 870
	
	Flashdata[i + 0x4D] = (RingbackEn = UserRxBufferFS[3 + i]) ; i++; // 871 //jj 26/11/2562
	
}

void Write_Flash(void)
{
		/* Save data to Flash */
		FlashErase();
		//FlashWrite(FLASH_PAGE_START_ADDRESS, (uint8_t*)Flashdata, 113);
		FlashWrite(FLASH_PAGE_START_ADDRESS, (uint8_t*)Flashdata, 114);
	
}

void Read_Config(void)
{
	uint8_t i,j = 0;
	InputType1_8 = *(uint8_t *)0x801F801;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType1_8 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType; j++;
	}
	
	InputType9_16 = *(uint8_t *)0x801F802;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType9_16 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType; j++;
	}
	
	InputType17_24 = *(uint8_t *)0x801F803;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType17_24 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType; j++;
	}
	
	InputType25_32 = *(uint8_t *)0x801F804;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType25_32 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType; j++;
	}
	
	InputType33_40 = *(uint8_t *)0x801F805;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType33_40 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType; j++;
	}
	
	InputType41_48 = *(uint8_t *)0x801F806;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType41_48 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType ; j++;
	}
	
	InputType49_56 = *(uint8_t *)0x801F807;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType49_56 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType ; j++;
	}
	
	InputType57_64 = *(uint8_t *)0x801F808;
	for(i =0; i < 8; i++)
	{
		fault[j].InputType = InputType57_64 >> i; fault[j].InputType = fault[j].InputType & 0x01; fault[j].FaultNCNO = fault[j].InputType ; j++;
	}
	
	j =0; //Reset 
	
	FaultType1_8 = *(uint8_t *)0x801F809;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType1_8 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	FaultType9_16 = *(uint8_t *)0x801F80A;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType9_16 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	FaultType17_24 = *(uint8_t *)0x801F80B;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType17_24 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	FaultType25_32 = *(uint8_t *)0x801F80C;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType25_32 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	FaultType33_40 = *(uint8_t *)0x801F80D;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType33_40 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	FaultType41_48 = *(uint8_t *)0x801F80E;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType41_48 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	FaultType49_56 = *(uint8_t *)0x801F80F;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType49_56 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	FaultType57_64 = *(uint8_t *)0x801F810;
	for(i =0; i < 8; i++)
	{
		fault[j].FaultType = FaultType57_64 >> i; fault[j].FaultType = fault[j].FaultType & 0x01; j++;
	}
	
	j =0; //Reset 
	OutputType1_8 = *(uint8_t *)0x801F811;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType1_8 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	OutputType9_16 = *(uint8_t *)0x801F812;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType9_16 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	OutputType17_24 = *(uint8_t *)0x801F813;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType17_24 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	OutputType25_32 = *(uint8_t *)0x801F814;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType25_32 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	OutputType33_40 = *(uint8_t *)0x801F815;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType33_40 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	OutputType41_48 = *(uint8_t *)0x801F816;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType41_48 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	OutputType49_56 = *(uint8_t *)0x801F817;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType49_56 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	OutputType57_64 = *(uint8_t *)0x801F818;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputType = OutputType57_64 >> i; fault[j].OutputType = fault[j].OutputType & 0x01; j++;
	}
	
	j =0; //Reset 
	OutputBoth1_8 = *(uint8_t *)0x801F819;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth1_8 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}
	
	OutputBoth9_16 = *(uint8_t *)0x801F81A;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth9_16 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}
	
	OutputBoth17_24 = *(uint8_t *)0x801F81B;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth17_24 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}
	
	OutputBoth25_32 = *(uint8_t *)0x801F81C;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth25_32 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}
	
	OutputBoth33_40 = *(uint8_t *)0x801F81D;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth33_40 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}
	
	OutputBoth41_48 = *(uint8_t *)0x801F81E;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth41_48 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}
	
	OutputBoth49_56 = *(uint8_t *)0x801F81F;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth49_56 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}

	OutputBoth57_64 = *(uint8_t *)0x801F820;
	for(i =0; i < 8; i++)
	{
		fault[j].OutputBoth = OutputBoth57_64 >> i; fault[j].OutputBoth = fault[j].OutputBoth & 0x01; j++;
	}
	
	j =0; //Reset 
	
	Alarm_Indicator1_8 = *(uint8_t *)0x801F821;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator1_8 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}

	Alarm_Indicator9_16 = *(uint8_t *)0x801F822;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator9_16 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}

	Alarm_Indicator17_24 = *(uint8_t *)0x801F823;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator17_24 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}
	
	Alarm_Indicator25_32 = *(uint8_t *)0x801F824;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator25_32 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}

	Alarm_Indicator33_40 = *(uint8_t *)0x801F825;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator33_40 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}

	Alarm_Indicator41_48 = *(uint8_t *)0x801F826;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator41_48 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}
	
	Alarm_Indicator49_56 = *(uint8_t *)0x801F827;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator49_56 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}

	Alarm_Indicator57_64 = *(uint8_t *)0x801F828;
	for(i =0; i < 8; i++)
	{
		fault[j].AlarmIndicator = Alarm_Indicator57_64 >> i; fault[j].AlarmIndicator = fault[j].AlarmIndicator & 0x01; j++;
	}

	AutoAck = *(uint8_t *)0x801F829;
	AutoAckTime = *(uint8_t *)0x801F82A;
	FlashingRate = *(uint8_t *)0x801F82B;
	RingbackTime = FlashingRate*2;
	NoOfPoint = *(uint8_t *)0x801F82C;
	MasterSlaveSync = *(uint8_t *)0x801F82D;
	Address = *(uint8_t *)0x801F82E;
	
	j = 0;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F82F; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F830; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F831; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F832; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F833; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F834; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F835; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F836; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F837; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F838; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F839; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F83A; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F83B; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F83C; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F83D; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F83E; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F83F; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F840; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F841; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F842; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F843; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F844; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F845; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F846; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F847; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F848; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F849; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F84A; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F84B; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F84C; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F84D; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F84E; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F84F; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F850; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F851; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F852; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F853; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F854; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F855; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F856; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F857; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F858; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F859; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F85A; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F85B; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F85C; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F85D; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F85E; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F85F; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F860; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F861; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F862; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F863; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F864; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F865; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F866; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F867; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F868; j++;
	
	fault[j].FaultDelayTime = *(uint8_t *)0x801F869; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F86A; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F86B; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F86C; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F86D; j++;
	fault[j].FaultDelayTime = *(uint8_t *)0x801F86E; j++;
	
	ReFaultEn = *(uint8_t *)0x801F86F;
	RefaultTime = *(uint8_t *)0x801F870;
	
	RingbackEn = *(uint8_t *)0x801F871;
	
	
	if(LoopCountStore ==0)LoopCountStore = 25;
	RingbackCount = RefaultTime*(LoopCountStore*100*60);  
}

///////////////////////////////////////////////////////////////////////////////	
///////////// Check Test ////////////////
void check_test(void)
{
	 uint8_t output_temp[8];
   
   output_temp[0] = Output1_8;
   output_temp[1] = Output9_16;
   output_temp[2] = Output17_24;
   output_temp[3] = Output25_32;
   output_temp[4] = Output33_40;
   output_temp[5] = Output41_48;
   output_temp[6] = Output49_56;
   output_temp[7] = Output57_64;
	
	if((!rd(Ptest) && Test == 0)|| AutoTestFlag == 1)
	{
		HAL_Delay(20);
		if((!rd(Ptest) && Test == 0)|| AutoTestFlag == 1) 
		{

			//restart_wdt();
			if(T_test == 0) T_test = 0x06;    //3 second for time base 500 ms
			Test = 1;
			if(NoOfPoint >= 16)
			{
				IO_OUTPUT_A(IO_DEVICE_0, 0x00);
				IO_OUTPUT_A(IO_DEVICE_1, 0x00);
			}
			if(NoOfPoint >= 24)
			{
				IO_OUTPUT_A(IO_DEVICE_2, 0x00);
			}
			if(NoOfPoint >= 32)
			{
				IO_OUTPUT_A(IO_DEVICE_3, 0x00);
			}
			if(NoOfPoint >= 40)
			{
				IO_OUTPUT_A(IO_DEVICE_4, 0x00);
			}
			if(NoOfPoint >= 48)
			{
				IO_OUTPUT_A(IO_DEVICE_5, 0x00);
			}
			if(NoOfPoint >= 56)
			{
				IO_OUTPUT_A(IO_DEVICE_6, 0x00);
			}
			if(NoOfPoint >= 64)
			{
				IO_OUTPUT_A(IO_DEVICE_7, 0x00);
			}
							
		}
		
	}
	else if(rd(Ptest) && Test == 1)
	{
		T_test = 0x00;
		Test = 0;
		Test_fault = 0;
	}
	
	if(Test_fault == 1)
	{
		for(uint8_t i = 0; i < NoOfPoint; i++)
		{
			fault[i].In = 1;
			
			// jj2392564
			fault[i].Ack = 0;
			fault[i].Ack2 = 0;
		}
		if(rd(Ptest))
		{
			Test_fault = 0;
		}  
	}
	else if(rd(Ptest) && Test == 1)
	{
		if(NoOfPoint >= 16)
		{
			IO_OUTPUT_A(IO_DEVICE_0, output_temp[0]);
			IO_OUTPUT_A(IO_DEVICE_1, output_temp[1]);
		}
		if(NoOfPoint >= 24)
		{
			IO_OUTPUT_A(IO_DEVICE_2, output_temp[2]);
		}
		if(NoOfPoint >= 32)
		{
			IO_OUTPUT_A(IO_DEVICE_3, output_temp[3]);
		}
		if(NoOfPoint >= 40)
		{
			IO_OUTPUT_A(IO_DEVICE_4, output_temp[4]);
		}
		if(NoOfPoint >= 48)
		{
			IO_OUTPUT_A(IO_DEVICE_5, output_temp[5]);
		}
		if(NoOfPoint >= 56)
		{
			IO_OUTPUT_A(IO_DEVICE_6, output_temp[6]);
		}
		if(NoOfPoint >= 64)
		{
			IO_OUTPUT_A(IO_DEVICE_7, output_temp[7]);
		}

		T_test = 0x00;
		Test = 0;
		Test_fault = 0;
	}
  
}


/////////////// Check acknowledge ////////////////////////////
void check_ack(void)
{
	if((!rd(Pack) && Ack_F == 0) || AutoAckFlag == 1)
	{
		HAL_Delay(10);
		if((!rd(Pack) && Ack_F == 0) || AutoAckFlag == 1)
		{
//			if(!rd(Pack) && Ack_F == 0){
//				
//			}
			//AutoAckFlag = 0;
			AutoAckDelayTime = 0x00;
			for(uint8_t i = 0; i < NoOfPoint; i++)
			{
				if((fault[i].In == 1 )||(fault[i].In2 == 1 ))
				{
					fault[i].Ack = 1;
					
					fault[i].In = 0;

					fault[i].In2 = 0;      //for auto reset
					fault[i].Output = 0;				 
					if((fault[i].Input^ fault[i].InputType) == 1)
					{
						fault[i].bit_faultHold = 1;
					}
					else
					{
						fault[i].Ringback =1;
					}			 
				}		
			}
			Ack_F = 1;

			char i;
			for( i=1; i<=NoOfPoint; i++)
			{
				fault[i].RingbackDelayTime = 0;
			}		
		}
	}
	else if(rd(Pack) && Ack_F == 1)
  {
		Ack_F = 0;
  }

}
////////////////////////////////////////////////////////////////////////////////

/////////////// Check2 acknowledge ////////////////////////////
void check_ack2(void)
{
   if((!HAL_GPIO_ReadPin(Pack2_port, Pack2) && Ack2_F == 0) || AutoAckFlag == 1)
	 {	
		HAL_Delay(10); //Acknowledge function
		if((!HAL_GPIO_ReadPin(Pack2_port, Pack2) && Ack2_F == 0) || AutoAckFlag == 1)
		{	
//			if(!HAL_GPIO_ReadPin(Pack2_port, Pack2)){
//				AutoAckFlag = 0;
//			}
			if(AutoAckFlag)
				AutoAckFlag = 0;
			AutoAckDelayTime = 0x00;
			
			for(uint8_t i = 0; i < NoOfPoint; i++)
			{
				if((fault[i].In == 1 )||(fault[i].In2 == 1 )) 
				{
					fault[i].Ack2 = 1;
					if(RingbackEn){
						fault[i].In = 0;
					}
					
					if((fault[i].Input^ fault[i].InputType) == 1)
					{
						fault[i].bit_faultHold = 1;
					}
					else
					{
						fault[i].Ringback =1;
					}
				}
			}
			
			char i;
			for( i=1; i<=NoOfPoint; i++)
			{
				fault[i].RingbackDelayTime = 0;
			}		
			
			
			Ack2_F = 1;
			HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_SET);
			HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_SET);		
		}
	}
	else if(HAL_GPIO_ReadPin(Pack2_port, Pack2) && Ack2_F == 1)
	{
		Ack2_F = 0;
	}

}
///////////////////////////////////////////////

///////////////////// Check Reset ///////////////////////////////////
void check_reset(void)
{
  if((!rd(Preset) && Reset_F == 0)|| AutoResetFlag ==1)
	{		
		HAL_Delay(10);
		if((!rd(Preset) && Reset_F == 0)|| AutoResetFlag ==1) 
		{
			AutoResetFlag = 0;
			
			for(uint8_t i = 0; i < NoOfPoint; i++)
			{

					if(((fault[i].Input ^ fault[i].InputType) == 0) && ((fault[i].In == 0)  || fault[i].Ack2 == 1  ))
					{
						//    JJJJ    ///
						fault[i].In = 0;
						fault[i].In2 = 0;      //for auto reset
						
						fault[i].Output = 1;
						
					}
					if((fault[i].Ringback)&&((fault[i].Input ^ fault[i].InputType) == 0))
					{
						fault[i].bit_faultHold =0;
						fault[i].Ringback =0;
						fault[i].Output = 1;
						 
						HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_SET);//buzzer 0
						HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_SET);//bell 0
					}

			}		
			Reset_F = 1;	   
		}
	}
	else if(rd(Preset) == 1 && Reset_F == 1)
	{
		 Reset_F = 0;
	}
}

///////////////////////Check Auto Reset function ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
/*
uint8_t CheckAutoReset(unsigned char DatType)
{
	uint8_t check;     /// "1" = true  //Other Alarm active or not "ACK"
	check = 0;
	if(DatType == 0x01) // Buzzer&&BELL
  {
		for(uint8_t i = 0; i < NoOfPoint; i++)
		{
			check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01)));
		}
	}
	else if(DatType == 0x02)             //Bell
	{
		for(uint8_t i = 0; i < NoOfPoint; i++)
		{
			check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01)));	
		}
		
	}
	
	return(check);
}
*/
////////////////////////////////////////////////////////////////////////////////////
uint8_t check;
uint8_t CheckAutoReset(unsigned char DatType)
{
   uint8_t i;     /// "1" = true  //Other Alarm active or not "ACK"
	 check = 0;
   if(DatType == 0x01) // Buzzer&&BELL
   {
      if(NoOfPoint >= 16)
      {
				/*
         check = (In.B1 | In2.B1) & ~Ack.B1 & (OutputType.B1 | OutputBoth.B1);
         check = check | ((In.B2 | In2.B2) & ~Ack.B2 & (OutputType.B2 | OutputBoth.B2));
         check = check | ((In.B3 | In2.B3) & ~Ack.B3 & (OutputType.B3 | OutputBoth.B3));
         check = check | ((In.B4 | In2.B4) & ~Ack.B4 & (OutputType.B4 | OutputBoth.B4));
         check = check | ((In.B5 | In2.B5) & ~Ack.B5 & (OutputType.B5 | OutputBoth.B5));
         check = check | ((In.B6 | In2.B6) & ~Ack.B6 & (OutputType.B6 | OutputBoth.B6));
         check = check | ((In.B7 | In2.B7) & ~Ack.B7 & (OutputType.B7 | OutputBoth.B7));
         check = check | ((In.B8 | In2.B8) & ~Ack.B8 & (OutputType.B8 | OutputBoth.B8));
         check = check | ((In.B9 | In2.B9) & ~Ack.B9 & (OutputType.B9 | OutputBoth.B9));
         check = check | ((In.B10 | In2.B10) & ~Ack.B10 & (OutputType.B10 | OutputBoth.B10));
         check = check | ((In.B11 | In2.B11) & ~Ack.B11 & (OutputType.B11 | OutputBoth.B11));
         check = check | ((In.B12 | In2.B12) & ~Ack.B12 & (OutputType.B12 | OutputBoth.B12));
         check = check | ((In.B13 | In2.B13) & ~Ack.B13 & (OutputType.B13 | OutputBoth.B13));
         check = check | ((In.B14 | In2.B14) & ~Ack.B14 & (OutputType.B14 | OutputBoth.B14));
         check = check | ((In.B15 | In2.B15) & ~Ack.B15 & (OutputType.B15 | OutputBoth.B15));
         check = check | ((In.B16 | In2.B16) & ~Ack.B16 & (OutputType.B16 | OutputBoth.B16));
				*/
				
				 i = 0;
				 check = ((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01)); i++;
         check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & ((fault[i].OutputType & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 
 
      }
      if(NoOfPoint >= 24)
      {
         check = check | ((In.B17 | In2.B17) & ~Ack.B17 & (OutputType.B17 | OutputBoth.B17));
         check = check | ((In.B18 | In2.B18) & ~Ack.B18 & (OutputType.B18 | OutputBoth.B18));
         check = check | ((In.B19 | In2.B19) & ~Ack.B19 & (OutputType.B19 | OutputBoth.B19));
         check = check | ((In.B20 | In2.B20) & ~Ack.B20 & (OutputType.B20 | OutputBoth.B20));
         check = check | ((In.B21 | In2.B21) & ~Ack.B21 & (OutputType.B21 | OutputBoth.B21));
         check = check | ((In.B22 | In2.B22) & ~Ack.B22 & (OutputType.B22 | OutputBoth.B22));
         check = check | ((In.B23 | In2.B23) & ~Ack.B23 & (OutputType.B23 | OutputBoth.B23));
         check = check | ((In.B24 | In2.B24) & ~Ack.B24 & (OutputType.B24 | OutputBoth.B24));
      }
      if(NoOfPoint >= 32)
      {
         check = check | ((In.B25 | In2.B25) & ~Ack.B25 & (OutputType.B25 | OutputBoth.B25));
         check = check | ((In.B26 | In2.B26) & ~Ack.B26 & (OutputType.B26 | OutputBoth.B26));
         check = check | ((In.B27 | In2.B27) & ~Ack.B27 & (OutputType.B27 | OutputBoth.B27));
         check = check | ((In.B28 | In2.B28) & ~Ack.B28 & (OutputType.B28 | OutputBoth.B28));
         check = check | ((In.B29 | In2.B29) & ~Ack.B29 & (OutputType.B29 | OutputBoth.B29));
         check = check | ((In.B30 | In2.B30) & ~Ack.B30 & (OutputType.B30 | OutputBoth.B30));
         check = check | ((In.B31 | In2.B31) & ~Ack.B31 & (OutputType.B31 | OutputBoth.B31));
         check = check | ((In.B32 | In2.B32) & ~Ack.B32 & (OutputType.B32 | OutputBoth.B32));
      }
      if(NoOfPoint >= 40)
      {
         check = check | ((In.B33 | In2.B33) & ~Ack.B33 & (OutputType.B33 | OutputBoth.B33));
         check = check | ((In.B34 | In2.B34) & ~Ack.B34 & (OutputType.B34 | OutputBoth.B34));
         check = check | ((In.B35 | In2.B35) & ~Ack.B35 & (OutputType.B35 | OutputBoth.B35));
         check = check | ((In.B36 | In2.B36) & ~Ack.B36 & (OutputType.B36 | OutputBoth.B36));
         check = check | ((In.B37 | In2.B37) & ~Ack.B37 & (OutputType.B37 | OutputBoth.B37));
         check = check | ((In.B38 | In2.B38) & ~Ack.B38 & (OutputType.B38 | OutputBoth.B38));
         check = check | ((In.B39 | In2.B39) & ~Ack.B39 & (OutputType.B39 | OutputBoth.B39));
         check = check | ((In.B40 | In2.B40) & ~Ack.B40 & (OutputType.B40 | OutputBoth.B40));
      }
      if(NoOfPoint >= 48)
      {
         check = check | ((In.B41 | In2.B41) & ~Ack.B41 & (OutputType.B41 | OutputBoth.B41));
         check = check | ((In.B42 | In2.B42) & ~Ack.B42 & (OutputType.B42 | OutputBoth.B42));
         check = check | ((In.B43 | In2.B43) & ~Ack.B43 & (OutputType.B43 | OutputBoth.B43));
         check = check | ((In.B44 | In2.B44) & ~Ack.B44 & (OutputType.B44 | OutputBoth.B44));
         check = check | ((In.B45 | In2.B45) & ~Ack.B45 & (OutputType.B45 | OutputBoth.B45));
         check = check | ((In.B46 | In2.B46) & ~Ack.B46 & (OutputType.B46 | OutputBoth.B46));
         check = check | ((In.B47 | In2.B47) & ~Ack.B47 & (OutputType.B47 | OutputBoth.B47));
         check = check | ((In.B48 | In2.B48) & ~Ack.B48 & (OutputType.B48 | OutputBoth.B48));
      }
      if(NoOfPoint >= 56)
      {
         check = check | ((In.B49 | In2.B49) & ~Ack.B49 & (OutputType.B49 | OutputBoth.B49));
         check = check | ((In.B50 | In2.B50) & ~Ack.B50 & (OutputType.B50 | OutputBoth.B50));
         check = check | ((In.B51 | In2.B51) & ~Ack.B51 & (OutputType.B51 | OutputBoth.B51));
         check = check | ((In.B52 | In2.B52) & ~Ack.B52 & (OutputType.B52 | OutputBoth.B52));
         check = check | ((In.B53 | In2.B53) & ~Ack.B53 & (OutputType.B53 | OutputBoth.B53));
         check = check | ((In.B54 | In2.B54) & ~Ack.B54 & (OutputType.B54 | OutputBoth.B54));
         check = check | ((In.B55 | In2.B55) & ~Ack.B55 & (OutputType.B55 | OutputBoth.B55));
         check = check | ((In.B56 | In2.B56) & ~Ack.B56 & (OutputType.B56 | OutputBoth.B56));
      }
      if(NoOfPoint >= 64)
      {
         check = check | ((In.B57 | In2.B57) & ~Ack.B57 & (OutputType.B57 | OutputBoth.B57));
         check = check | ((In.B58 | In2.B58) & ~Ack.B58 & (OutputType.B58 | OutputBoth.B58));
         check = check | ((In.B59 | In2.B59) & ~Ack.B59 & (OutputType.B59 | OutputBoth.B59));
         check = check | ((In.B60 | In2.B60) & ~Ack.B60 & (OutputType.B60 | OutputBoth.B60));
         check = check | ((In.B61 | In2.B61) & ~Ack.B61 & (OutputType.B61 | OutputBoth.B61));
         check = check | ((In.B62 | In2.B62) & ~Ack.B62 & (OutputType.B62 | OutputBoth.B62));
         check = check | ((In.B63 | In2.B63) & ~Ack.B63 & (OutputType.B63 | OutputBoth.B63));
         check = check | ((In.B64 | In2.B64) & ~Ack.B64 & (OutputType.B64 | OutputBoth.B64));
      }
      
   }
   else if(DatType == 0x02)             //Bell
   {
			if(NoOfPoint >= 16)
      {
				i = 0;
				 check = ((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01)); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 check = check | (((fault[i].In & 0x01) | (fault[i].In2 & 0x01)) & ((~fault[i].Ack) & 0x01) & (((~fault[i].OutputType) & 0x01) | (fault[i].OutputBoth & 0x01))); i++;
				 
				/*
				check = (In.B1 | In2.B1) & ~Ack.B1 & (~OutputType.B1 | OutputBoth.B1);
				check = check | ((In.B2 | In2.B2) & ~Ack.B2 & (~OutputType.B2 | OutputBoth.B2));
				check = check | ((In.B3 | In2.B3) & ~Ack.B3 & (~OutputType.B3 | OutputBoth.B3));
				check = check | ((In.B4 | In2.B4) & ~Ack.B4 & (~OutputType.B4 | OutputBoth.B4));
				check = check | ((In.B5 | In2.B5) & ~Ack.B5 & (~OutputType.B5 | OutputBoth.B5));
				check = check | ((In.B6 | In2.B6) & ~Ack.B6 & (~OutputType.B6 | OutputBoth.B6));
				check = check | ((In.B7 | In2.B7) & ~Ack.B7 & (~OutputType.B7 | OutputBoth.B7));
				check = check | ((In.B8 | In2.B8) & ~Ack.B8 & (~OutputType.B8 | OutputBoth.B8));
				check = check | ((In.B9 | In2.B9) & ~Ack.B9 & (~OutputType.B9 | OutputBoth.B9));
				check = check | ((In.B10 | In2.B10) & ~Ack.B10 & (~OutputType.B10 | OutputBoth.B10));
				check = check | ((In.B11 | In2.B11) & ~Ack.B11 & (~OutputType.B11 | OutputBoth.B11));
				check = check | ((In.B12 | In2.B12) & ~Ack.B12 & (~OutputType.B12 | OutputBoth.B12));
				check = check | ((In.B13 | In2.B13) & ~Ack.B13 & (~OutputType.B13 | OutputBoth.B13));
				check = check | ((In.B14 | In2.B14) & ~Ack.B14 & (~OutputType.B14 | OutputBoth.B14));
				check = check | ((In.B15 | In2.B15) & ~Ack.B15 & (~OutputType.B15 | OutputBoth.B15));
				check = check | ((In.B16 | In2.B16) & ~Ack.B16 & (~OutputType.B16 | OutputBoth.B16));
				*/
      }
      if(NoOfPoint >= 24)
      {
        check = check | ((In.B17 | In2.B17) & ~Ack.B17 & (~OutputType.B17 | OutputBoth.B17));
				check = check | ((In.B18 | In2.B18) & ~Ack.B18 & (~OutputType.B18 | OutputBoth.B18));
				check = check | ((In.B19 | In2.B19) & ~Ack.B19 & (~OutputType.B19 | OutputBoth.B19));
				check = check | ((In.B20 | In2.B20) & ~Ack.B20 & (~OutputType.B20 | OutputBoth.B20));
				check = check | ((In.B21 | In2.B21) & ~Ack.B21 & (~OutputType.B21 | OutputBoth.B21));
				check = check | ((In.B22 | In2.B22) & ~Ack.B22 & (~OutputType.B22 | OutputBoth.B22));
				check = check | ((In.B23 | In2.B23) & ~Ack.B23 & (~OutputType.B23 | OutputBoth.B23));
				check = check | ((In.B24 | In2.B24) & ~Ack.B24 & (~OutputType.B24 | OutputBoth.B24)); 
      }
      if(NoOfPoint >= 32)
      {
        check = check | ((In.B25 | In2.B25) & ~Ack.B25 & (~OutputType.B25 | OutputBoth.B25));
				check = check | ((In.B26 | In2.B26) & ~Ack.B26 & (~OutputType.B26 | OutputBoth.B26));
				check = check | ((In.B27 | In2.B27) & ~Ack.B27 & (~OutputType.B27 | OutputBoth.B27));
				check = check | ((In.B28 | In2.B28) & ~Ack.B28 & (~OutputType.B28 | OutputBoth.B28));
				check = check | ((In.B29 | In2.B29) & ~Ack.B29 & (~OutputType.B29 | OutputBoth.B29));
				check = check | ((In.B30 | In2.B30) & ~Ack.B30 & (~OutputType.B30 | OutputBoth.B30));
				check = check | ((In.B31 | In2.B31) & ~Ack.B31 & (~OutputType.B31 | OutputBoth.B31));
				check = check | ((In.B32 | In2.B32) & ~Ack.B32 & (~OutputType.B32 | OutputBoth.B32));
      }
      if(NoOfPoint >= 40)
      {
        check = check | ((In.B33 | In2.B33) & ~Ack.B33 & (~OutputType.B33 | OutputBoth.B33));
				check = check | ((In.B34 | In2.B34) & ~Ack.B34 & (~OutputType.B34 | OutputBoth.B34));
				check = check | ((In.B35 | In2.B35) & ~Ack.B35 & (~OutputType.B35 | OutputBoth.B35));
				check = check | ((In.B36 | In2.B36) & ~Ack.B36 & (~OutputType.B36 | OutputBoth.B36));
				check = check | ((In.B37 | In2.B37) & ~Ack.B37 & (~OutputType.B37 | OutputBoth.B37));
				check = check | ((In.B38 | In2.B38) & ~Ack.B38 & (~OutputType.B38 | OutputBoth.B38));
				check = check | ((In.B39 | In2.B39) & ~Ack.B39 & (~OutputType.B39 | OutputBoth.B39));
				check = check | ((In.B40 | In2.B40) & ~Ack.B40 & (~OutputType.B40 | OutputBoth.B40));
      }
      if(NoOfPoint >= 48)
      {
        check = check | ((In.B41 | In2.B41) & ~Ack.B41 & (~OutputType.B41 | OutputBoth.B41));
				check = check | ((In.B42 | In2.B42) & ~Ack.B42 & (~OutputType.B42 | OutputBoth.B42));
				check = check | ((In.B43 | In2.B43) & ~Ack.B43 & (~OutputType.B43 | OutputBoth.B43));
				check = check | ((In.B44 | In2.B44) & ~Ack.B44 & (~OutputType.B44 | OutputBoth.B44));
				check = check | ((In.B45 | In2.B45) & ~Ack.B45 & (~OutputType.B45 | OutputBoth.B45));
				check = check | ((In.B46 | In2.B46) & ~Ack.B46 & (~OutputType.B46 | OutputBoth.B46));
				check = check | ((In.B47 | In2.B47) & ~Ack.B47 & (~OutputType.B47 | OutputBoth.B47));
				check = check | ((In.B48 | In2.B48) & ~Ack.B48 & (~OutputType.B48 | OutputBoth.B48));
      }
      if(NoOfPoint >= 56)
      {
        check = check | ((In.B49 | In2.B49) & ~Ack.B49 & (~OutputType.B49 | OutputBoth.B49));
				check = check | ((In.B50 | In2.B50) & ~Ack.B50 & (~OutputType.B50 | OutputBoth.B50));
				check = check | ((In.B51 | In2.B51) & ~Ack.B51 & (~OutputType.B51 | OutputBoth.B51));
				check = check | ((In.B52 | In2.B52) & ~Ack.B52 & (~OutputType.B52 | OutputBoth.B52));
				check = check | ((In.B53 | In2.B53) & ~Ack.B53 & (~OutputType.B53 | OutputBoth.B53));
				check = check | ((In.B54 | In2.B54) & ~Ack.B54 & (~OutputType.B54 | OutputBoth.B54));
				check = check | ((In.B55 | In2.B55) & ~Ack.B55 & (~OutputType.B55 | OutputBoth.B55));
				check = check | ((In.B56 | In2.B56) & ~Ack.B56 & (~OutputType.B56 | OutputBoth.B56));
      }
      if(NoOfPoint >= 64)
      {
        check = check | ((In.B57 | In2.B57) & ~Ack.B57 & (~OutputType.B57 | OutputBoth.B57));
				check = check | ((In.B58 | In2.B58) & ~Ack.B58 & (~OutputType.B58 | OutputBoth.B58));
				check = check | ((In.B59 | In2.B59) & ~Ack.B59 & (~OutputType.B59 | OutputBoth.B59));
				check = check | ((In.B60 | In2.B60) & ~Ack.B60 & (~OutputType.B60 | OutputBoth.B60));
				check = check | ((In.B61 | In2.B61) & ~Ack.B61 & (~OutputType.B61 | OutputBoth.B61));
				check = check | ((In.B62 | In2.B62) & ~Ack.B62 & (~OutputType.B62 | OutputBoth.B62));
				check = check | ((In.B63 | In2.B63) & ~Ack.B63 & (~OutputType.B63 | OutputBoth.B63));
				check = check | ((In.B64 | In2.B64) & ~Ack.B64 & (~OutputType.B64 | OutputBoth.B64));
      }

   }
   return(check);
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void Anal_Function(void)
{
	
	static unsigned char inputflag = 0;

	// Anual function
	for(uint8_t i = 0; i < NoOfPoint; i++)
	{	
		/////////////////// Auto reset //////////////////////////////////////////////////////////////////////
		if((((fault[analfault_index[i]].Input ^ fault[analfault_index[i]].InputType) == 1) && (fault[analfault_index[i]].FaultType == 0)) || Test_fault == 1)    // alarm1 occure and " Non Lock type"
		{
			if(fault[analfault_index[i]].In2 == 0 && AutoAck == 0x0F && fault[analfault_index[i]].Ack == 0)   //use auto acknowlegde
			{
				TimeBase1s = 100 ;            //reload again
				AutoAckDelayTime = AutoAckTime ;
			}
			fault[analfault_index[i]].In2 = 1;
			if(fault[analfault_index[i]].Ack == 0)
			{
				if(FlashingFlag == 0)
				{
					fault[analfault_index[i]].Output = 0;           //Flash output1
				}
				else
				{
					fault[analfault_index[i]].Output = 1;
				}
				if (~AutoTestFlag)
				{
					
					if(!(fault[analfault_index[i]].Ack2))
					{
						if(fault[analfault_index[i]].AlarmIndicator == 1)
						{
							if(fault[analfault_index[i]].OutputBoth == 0)        //Both output
							{
								HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_RESET);//on buzzer 0
								HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_RESET);//on bell 0
							}
							else
							{
								if(fault[analfault_index[i]].OutputType == 1)
								{
									HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_RESET);//on buzzer 0
								}
								else
								{
									HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_RESET);//on bell 0
								}
							}
						}
					}

				}
			}
			inputflag =1;
		}
		else if(fault[analfault_index[i]].FaultType == 0)
		{
			uint8_t  in2count =0 ;
			
			fault[analfault_index[i]].In = 0;//jj412564 
			
			fault[analfault_index[i]].In2 = 0;
			fault[analfault_index[i]].Ack = 0;
			
			fault[analfault_index[i]].Ack2 = 0;
			
			fault[analfault_index[i]].Output = 1;     //Off LED

			if((fault[analfault_index[i]].OutputType == 1 || fault[analfault_index[i]].OutputBoth == 0)&& inputflag ==0)      //If Buzzer or Both
			{
				if(CheckAutoReset(0x01));   //Check other Input "Ack" or not if not,do nothing
				else
				{
					HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_SET);//off buzzer 
				}
			}

			if((fault[analfault_index[i]].OutputType == 0 || fault[analfault_index[i]].OutputBoth == 0)&& inputflag ==0)  //If Bell or Both
			{
				if(CheckAutoReset(0x02));
				else
				{
					HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_SET);//off bell 
				}
			}
		
			//uint8_t  in2count =0 ;
			for(uint8_t i = 0; i < NoOfPoint; i++)
			{	
				if(fault[analfault_index[i]].In2 == 1){
					in2count++;
				}
			}
			if(in2count == 0 ){
				HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_SET);//off bell 
				HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_SET);//off buzzer   
			}
		}
		
		/////////////////// Manual reset //////////////////////////////////////////////////////////////////////
		if(fault[analfault_index[i]].AlarmIndicator) // Alarm
		{
			if((((fault[analfault_index[i]].Input ^ fault[analfault_index[i]].InputType) == 1) && (fault[analfault_index[i]].FaultType == 1)) || fault[analfault_index[i]].In == 1)    // alarm1 occure and "Lock type"
			{
				if(fault[analfault_index[i]].Ack == 0)
				{
					if(fault[analfault_index[i]].In == 0 && AutoAck == 0x0F)   //use auto acknowlegde
					{
						TimeBase1s = 100 ;  //reload again
						AutoAckDelayTime = AutoAckTime ;
					}
					fault[analfault_index[i]].In = 1;     // setbit in1
					if(FlashingFlag == 0)
					{
						fault[analfault_index[i]].Output = 0; //Flash output1
					}
					else
					{
						fault[analfault_index[i]].Output = 1;
					}
					if (~AutoTestFlag)
					{
												
						if(!(fault[analfault_index[i]].Ack2)) 
						{
							if(fault[analfault_index[i]].OutputBoth == 0)                  //Both output
							{
								HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_RESET);//on bell 0
								HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_RESET);//on buzzer 0
							}
							else{
								if(fault[analfault_index[i]].OutputType == 1)
								{ 
									HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_RESET);//on buzzer 0
								}
								else
								{ 
									HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_RESET);//on bell 0
								}
							}
							
						}
						
					}
				}
			}
			else if(fault[analfault_index[i]].FaultType == 1)
			{
				
				fault[analfault_index[i]].Ack = 0;
				fault[analfault_index[i]].Ack2 = 0;
			
			}
		}
		else // indicator
		{
			if((fault[analfault_index[i]].Input ^ fault[analfault_index[i]].InputType) == 1)
			{
				fault[analfault_index[i]].Output = 0;
			}
			else
			{
				fault[analfault_index[i]].Output = 1;
			}
		}
			
	}

///////////////////////////////////////////////////////////////////////////////

}

//END Anal_Function///////////////////////////////////////////////////////////////////////
void check_Ringback(void)
{
	for(uint8_t i = 0; i < NoOfPoint; i++)
	{
		if(fault[i].FaultType) // manual reset only
			if((fault[analfault_index[i]].Input ^ fault[analfault_index[i]].InputType) == 1) 
			{
				if((fault[i].Ack2 == 1) && (fault[i].In == 1)){
					fault[i].In = 0;
				}				
			}
					
			if(((fault[i].Input ^ fault[i].InputType) == 0) && (fault[i].Ringback))
			{
				if(RingbackFlag)
				{
					fault[i].Output = 1;           //Flash output1
					if(fault[i].OutputBoth == 0)        //Both output
					{
						HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_SET);//buzzer 0
						HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_SET);//bell 0
					}
					else
					{
						if(fault[i].OutputType == 1) HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_SET);//buzzer 0
						else HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_SET);//bell 0
					}
				}
				else
				{
					fault[i].Output = 0;
					if(fault[i].OutputBoth == 0)        //Both output
					{
						HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_RESET);//buzzer 0
						HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_RESET);//bell 0
					}
					else
					{
						if(fault[i].OutputType == 1) HAL_GPIO_WritePin(BELL_BUZZER_port,Pbuzzer,GPIO_PIN_RESET);//buzzer 0
						else HAL_GPIO_WritePin(BELL_BUZZER_port,Pbell,GPIO_PIN_RESET);//bell 0
					}
					 
				}
			}
		
	}
	
	
}

void check_Refault(void)
{
	if(ReFaultEn)
	{
		for(uint8_t i = 0; i < NoOfPoint; i++)
		{
			if(fault[i].RingbackDelayTime >= RingbackCount)
			{
				fault[i].RingbackDelayTime =0;
				if(fault[i].FaultType ==1)
				{
					fault[i].In = 1;
					
				}
				else
				{
					fault[i].In2 = 1;
					
				}
				fault[i].Ack = 0;
				fault[i].Ack2 = 0;
				fault[i].bit_faultHold = 0;
			}	
			
		}
	}
	
	
}

//************** 23S17 Function *************************
void IO_WRITE_REGISTER(uint8_t address, uint8_t reg, uint8_t data){
	uint8_t SPIbuf[3];
	
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET); //CS Pin
     
	SPIbuf[0]= IO_DEVICE_ADDRESS_WRITE | address;
	SPIbuf[1]= reg;
  SPIbuf[2]= data;
	HAL_SPI_Transmit(&hspi2, (uint8_t*)SPIbuf, 0x03, 0x05);
   
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET); //CS Pin

}
//***************************************
uint8_t IO_READ_REGISTER(uint8_t address, uint8_t reg){
	
	uint8_t SPIbuf[3];
  uint8_t retVal;
	static uint8_t Old_Result =0;

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET); //CS Pin
      
	SPIbuf[0]= IO_DEVICE_ADDRESS_READ | address;
  
	SPIbuf[1]= reg;
	HAL_SPI_Transmit(&hspi2, (uint8_t*)SPIbuf, 0x02, 0x05);
	
	SPI_Status = HAL_SPI_Receive(&hspi2, &retVal, 0x01, 0x05);  
	
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET); //CS Pin
	
	if(SPI_Status == HAL_OK){
		Old_Result = retVal;
		return retVal;
	}
  else return Old_Result; //if SPI Error Return old data
   
}
//********************************
void MCP23S17_INIT(void){

   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET); //CS Pin
	
   IO_WRITE_REGISTER(IO_DEVICE_0, 0x0A, 0xAA);
   IO_WRITE_REGISTER(IO_DEVICE_1, 0x0A, 0xAA);  //Edit by Jack
   IO_WRITE_REGISTER(IO_DEVICE_2, 0x0A, 0xAA);  //Edit by Jack
   IO_WRITE_REGISTER(IO_DEVICE_3, 0x0A, 0xAA);  //Edit by Jack
   IO_WRITE_REGISTER(IO_DEVICE_4, 0x0A, 0xAA);  //Edit by Jack
   IO_WRITE_REGISTER(IO_DEVICE_5, 0x0A, 0xAA);  //Edit by Jack
   IO_WRITE_REGISTER(IO_DEVICE_6, 0x0A, 0xAA);  //Edit by Jack
   IO_WRITE_REGISTER(IO_DEVICE_7, 0x0A, 0xAA);  //Edit by Jack
   
}
//***************************************
void IO_SET_TRIS_A(uint8_t address, uint8_t data)
{
   IO_WRITE_REGISTER(address, IODIRA, data);
}
//***************************************
void IO_SET_TRIS_B(uint8_t address, uint8_t data)
{
   IO_WRITE_REGISTER(address, IODIRB, data);
}
//***************************************
void IO_OUTPUT_A(uint8_t address, uint8_t data)
{
   IO_WRITE_REGISTER(address, OLATA, data);
}
//***************************************
void IO_OUTPUT_B(uint8_t address, uint8_t data)
{
   IO_WRITE_REGISTER(address, OLATB, data);
}
//***************************************
uint8_t IO_INPUT_A(uint8_t address){
   uint8_t retVal;
  
   retVal = IO_READ_REGISTER(address, GPIO_A);  
   return retVal;
}
//***************************************
uint8_t IO_INPUT_B(uint8_t address){
   uint8_t retVal;
  
   retVal = IO_READ_REGISTER(address, GPIO_B);  
   return retVal;
}

//--------------- FLASH fUNCTION ---------------------------------//
uint8_t FlashErase(void)
{
  uint8_t ret = 1;
  uint32_t Address;
  
  /* Unlock the Flash to enable the flash control register access *************/ 
	HAL_FLASH_Unlock();
  
  /* Erase the user Flash area ***********/

  /* Clear pending flags (if any) */  
  FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
  
  for(Address = FLASH_PAGE_START_ADDRESS; Address < FLASH_PAGE_END_ADDRESS; Address += FLASH_PAGE_size)
  {
    /* Wait for last operation to be completed */
    while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
    
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    {
      /* Write protected error */
      ret = 0;
      break;
    }
    
    if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
    {
      /* Programming error */
      ret = 0;
      break;
    }
    
    /* If the previous operation is completed, proceed to erase the page */
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = Address;
    FLASH->CR |= FLASH_CR_STRT;
      
    /* Wait for last operation to be completed */
    while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
    
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    {
      /* Write protected error */
      ret = 0;
      break;
    }
    
    if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
    {
      /* Programming error */
      ret = 0;
      break;
    }
      
    /* Disable the PER Bit */
    FLASH->CR &= ~FLASH_CR_PER;
  }
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  /* Set the LOCK Bit to lock the FLASH control register and program memory access */
  FLASH->CR |= FLASH_CR_LOCK;
  
  return ret;
}

uint8_t FlashWrite(uint32_t Address, uint8_t *Data, uint32_t Length)
{
  uint8_t ret = 1;
  uint16_t TmpData;
  
  if(Address >= FLASH_PAGE_START_ADDRESS && Address <= FLASH_PAGE_END_ADDRESS)
  {
    /* Unlock the Flash to enable the flash control register access *************/ 
    HAL_FLASH_Unlock();
    
    /* Clear pending flags (if any) */  
    FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    
    while(Length > 0)
    {
      if(Length == 1)
      {
        TmpData = Data[0] | (0x00 << 8 );
        Data = Data + 1;
        Length = Length - 1;
      }
      else
      {
        TmpData = Data[0] | (Data[1] << 8 );
        Data = Data + 2;
        Length = Length - 2;
      }
      
      /* Wait for last operation to be completed */
      while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
      
      if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
      {
        /* Write protected error */
        ret = 0;
        break;
      }
      
      if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
      {
        /* Programming error */
        ret = 0;
        break;
      }
      
      /* If the previous operation is completed, proceed to program the new data */
      FLASH->CR |= FLASH_CR_PG;
      
      *(__IO uint16_t*)Address = TmpData;
      
      /* Wait for last operation to be completed */
      while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
      
      if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
      {
        /* Write protected error */
        ret = 0;
        break;
      }
      
      if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
      {
        /* Programming error */
        ret = 0;
        break;
      }
      
      /* Disable the PG Bit */
      FLASH->CR &= ~FLASH_CR_PG;
      
      /* Next address */
      Address = Address + 2;
    }
    
    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    /* Set the LOCK Bit to lock the FLASH control register and program memory access */
    FLASH->CR |= FLASH_CR_LOCK;
  }
  else
    ret = 0;
  
  return ret;
}

//--------------- FLASH fUNCTION ---------------------------------//

void Driver595(void)
{
	uint8_t SPIbuf[8];
	uint8_t datacount =0;
	
	if(NoOfPoint >= 64)
  {		
		SPIbuf[datacount] = ~Input57_64;
		datacount++;
	}
	if(NoOfPoint >= 56)
  {
		SPIbuf[datacount] = ~Input49_56;
		datacount++;
	}
	if(NoOfPoint >= 48)
  {
		SPIbuf[datacount] = ~Input41_48;
		datacount++;
	}
	if(NoOfPoint >= 40)
  {
		SPIbuf[datacount] = ~Input33_40;
		datacount++;
	}
	if(NoOfPoint >= 32)
  {
		SPIbuf[datacount] = ~Input25_32;
		datacount++;
	}
	if(NoOfPoint >= 24)
  {
		SPIbuf[datacount] = ~Input17_24;
		datacount++;
	}
	if(NoOfPoint >= 16)
  {
		SPIbuf[datacount] = ~Input9_16;
		datacount++;
	}
	if(NoOfPoint >= 8)
	{		
		SPIbuf[datacount] = ~Input1_8;
		datacount++;
	}
	
	HAL_SPI_Transmit(&hspi1, (uint8_t*)SPIbuf, datacount, 0x05);
	
	HAL_GPIO_WritePin(HC595EN_port,HC595EN_PIN,GPIO_PIN_SET); 
	delay_us(1);
	HAL_GPIO_WritePin(HC595EN_port,HC595EN_PIN,GPIO_PIN_RESET); 
}

/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */

//*******************************************
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$	
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$		

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */	
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
