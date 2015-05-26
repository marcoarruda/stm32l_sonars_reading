#include "stm32l1xx_conf.h"
#include "stm32l1xx.h"
#include "stdio.h"
#include "discover_board.h"
#include "stm32l_discovery_lcd.h"
#include "MyUSART.h"

typedef struct
{
  uint16_t VREF;
  uint16_t TS_CAL_1; // low temperature calibration data
  uint16_t reserved;
  uint16_t TS_CAL_2; // high temperature calibration data
} TSCALIB_TypeDef;

typedef enum
{
  Display_TemperatureDegC,
  Display_ADCval
} DisplayState_TypeDef;


#define DEBUG_SWD_PIN  1  /* needs to be set to 1 to enable SWD debug pins, set to 0 for power consumption measurement*/
#define FACTORY_TSCALIB_MD_BASE         ((uint32_t)0x1FF80078)    /*!< Calibration Data Bytes base address for medium density devices*/
#define FACTORY_TSCALIB_MDP_BASE        ((uint32_t)0x1FF800F8)    /*!< Calibration Data Bytes base address for medium density plus devices*/
#define FACTORY_TSCALIB_MD_DATA         ((TSCALIB_TypeDef *) FACTORY_TSCALIB_MD_BASE)
#define FACTORY_TSCALIB_MDP_DATA        ((TSCALIB_TypeDef *) FACTORY_TSCALIB_MDP_BASE)
#define USER_CALIB_BASE           ((uint32_t)0x08080000)    /*!< USER Calibration Data Bytes base address */
#define USER_CALIB_DATA           ((TSCALIB_TypeDef *) USER_CALIB_BASE)
#define TEST_CALIB_DIFF           (int32_t) 50  /* difference of hot-cold calib
data to be considered as valid */

#define HOT_CAL_TEMP 		110
#define COLD_CAL_TEMP  	30

#define DEFAULT_HOT_VAL 0x362
#define DEFAULT_COLD_VAL 0x2A8

#define MAX_TEMP_CHNL 12

#define ADC_CONV_BUFF_SIZE 20

#define VDD_CORRECTION 1  /* definition for correction of VDD if <> 3V */

ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef DMA_InitStructure;

__IO uint16_t 	ADC_ConvertedValue, T_StartupTimeDelay;

uint32_t ADC_Result, INTemperature, refAVG, tempAVG, vdd_ref, Address = 0;
uint32_t t, ultrassomAVG;
int32_t temperature_C;

uint16_t ultrassom1, ultrassom2, ultrassom3, ultrassom4, ultrassom5, predefvalue;

uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE];

char strDisp[20];

DisplayState_TypeDef CurrentlyDisplayed = Display_TemperatureDegC;

TSCALIB_TypeDef calibdata;    /* field storing temp sensor calibration data */

volatile bool flag_ADCDMA_TransferComplete;
volatile bool flag_UserButton;

static volatile uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;


/* Private function prototypes -----------------------------------------------*/
void  RCC_Configuration(void);
void  RTC_Configuration(void);
void  Init_GPIOs(void);
void  acquireTemperatureData(void);
void  configureADC_Temp(void);
void  configureDMA(void);
void  powerDownADC_Temper(void);
uint16_t conversaoADCMilimetros(uint16_t ultrassomADC);
void  processTempData(void);
void  configureWakeup(void);
void insertionSort(uint16_t *numbers, uint32_t array_size);
uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples);
void clearUserButtonFlag(void);
/*******************************************************************************/

int main(void)
{
  RCC_Configuration();
  //RTC_Configuration();
  PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);
  while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET);
  
#ifdef  DEBUG_SWD_PIN
  DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);
#endif
  
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  
  Init_GPIOs();
  
  InitializeUSART3();
  
  configureWakeup();
  configureDMA();
  configureADC_Temp();
  
  t = 0;
  
  int defaultTD = 50;
  TimingDelay = defaultTD;
  while (1) {
    if (TimingDelay > 0) continue;
    TimingDelay = defaultTD;
    acquireTemperatureData();
    __WFI();
    
    /* for DEBUG purpose uncomment the following line and comment the __WFI call to do not enter STOP mode */
    //while (!flag_ADCDMA_TransferComplete);
    
    powerDownADC_Temper();
    processTempData();
    
    /* uint16 > -32768 -- +32767 */
    /* tamanho máximo de %s de uint16 = 5 */
    /* tamanho máximo de string com 5 valores uint16 = 2([[) + 5*5 + 4(;;;;) + 2(]]) = 33 */
    char string[33];
    sprintf(string, "[[%d;%d;%d;%d;%d]]", ultrassom1, ultrassom2, ultrassom3, ultrassom4, ultrassom5);
    USART3SendString(string);
    
    
    GPIO_TOGGLE(GPIOB, GPIO_Pin_6);
  }
  
}


void configureWakeup(void)
{
  /* Declare initialisation structures for (NVIC) and external interupt (EXTI) */
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* Clear IT pending bit from external interrupt Line 20 */
  EXTI_ClearITPendingBit(EXTI_Line20);
  
  /* Initialise EXTI using its init structure */
  EXTI_InitStructure.EXTI_Line = EXTI_Line20;			 // interrupt generated on RTC Wakeup event (Line 20)
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;    // Use EXTI line as interrupt
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Trigg interrupt on rising edge detection
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;				 // Enable EXTI line
  EXTI_Init(&EXTI_InitStructure);
  
  /* Initialise the NVIC interrupts (IRQ) using its init structure */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;        // set IRQ channel to RTC Wakeup Interrupt  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 // set channel Preemption priority to 0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // set channel sub priority to 0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	         // Enable channel
  NVIC_Init(&NVIC_InitStructure);
  
  /* Clear Wake-up flag */
  PWR->CR |= PWR_CR_CWUF;
  
  /* Enable PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  /* Allow access to RTC */
  PWR_RTCAccessCmd(ENABLE);
  
  /* Enable Low Speed External clock */
  RCC_LSEConfig(RCC_LSE_ON);
  
  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
  
  /* Select LSE clock as RCC Clock source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);
  
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();
  
  /* Select 1Hz clock for RTC wake up*/
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  
  /* Set Wakeup auto-reload value to 2 sec */
  RTC_SetWakeUpCounter(1);
  
  /* Clear RTC Interrupt pending bit */
  RTC_ClearITPendingBit(RTC_IT_WUT);
  
  /* Clear EXTI line20 Interrupt pending bit */
  EXTI_ClearITPendingBit(EXTI_Line20);
  
  /* Enable the Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
}

void setUserButtonFlag(void)
{
  flag_UserButton = TRUE;
}

void clearUserButtonFlag(void)
{
  flag_UserButton = FALSE;
}

void setADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = TRUE;
}

void clearADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = FALSE;
}

void acquireTemperatureData(void)
{
  /* Enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Wait until the ADC1 is ready */
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);
  
  /* re-initialize DMA -- is it needed ?*/
  DMA_DeInit(DMA1_Channel1);
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* Enable DMA channel 1 Transmit complete interrupt*/
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  
  /* Disable DMA mode for ADC1 */
  ADC_DMACmd(ADC1, DISABLE);
  
  /* Enable DMA mode for ADC1 */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Clear global flag for DMA transfert complete */
  clearADCDMA_TransferComplete();
  
  /* Start ADC conversion */
  ADC_SoftwareStartConv(ADC1);
}

void powerDownADC_Temper(void)
{
  /* Disable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  /* Disable ADC1 */
  ADC_Cmd(ADC1, DISABLE);
  
  /* Disable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
  /* Disable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
}

void configureADC_Temp(void)
{
  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* Enable the internal connection of Temperature sensor and with the ADC channels*/
  ADC_TempSensorVrefintCmd(ENABLE);
  
  /* Wait until ADC + Temp sensor start */
  T_StartupTimeDelay = 1024;
  while (T_StartupTimeDelay--);
  
  /* Setup ADC common init struct */
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  
  /* Initialise the ADC1 by using its init structure */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	          // Set conversion resolution to 12bit
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          // Enable Scan mode (single conversion for each channel of the group)
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			  // Disable Continuous conversion
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; // Disable external conversion trigger
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  // Set conversion data alignement to right
  ADC_InitStructure.ADC_NbrOfConversion = ADC_CONV_BUFF_SIZE;             // Set conversion data alignement to ADC_CONV_BUFF_SIZE
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 13, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 14, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 15, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 16, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 17, ADC_SampleTime_384Cycles);
}

void configureDMA(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  DMA_DeInit(DMA1_Channel1);
  
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	     // Set DMA channel Peripheral base address to ADC Data register
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValueBuff;  // Set DMA channel Memeory base addr to ADC_ConvertedValueBuff address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                         // Set DMA channel direction to peripheral to memory
  DMA_InitStructure.DMA_BufferSize = ADC_CONV_BUFF_SIZE;                     // Set DMA channel buffersize to peripheral to ADC_CONV_BUFF_SIZE
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	     // Disable DMA channel Peripheral address auto increment
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // Enable Memeory increment (To be verified ....)
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// set Peripheral data size to 8bit 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	     // set Memeory data size to 8bit 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                              // Set DMA in normal mode
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                     // Set DMA channel priority to High
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                               // Disable memory to memory option 
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);								 // Use Init structure to initialise channel1 (channel linked to ADC)
  
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}

void RCC_Configuration(void)
{
  RCC_HSICmd(ENABLE);
  
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {
  }
  
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  
  RCC_MSIRangeConfig(RCC_MSIRange_6);
  
  RCC_HSEConfig(RCC_HSE_OFF);
  if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
  {
    while (1);
  }
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_LCD | RCC_APB1Periph_PWR, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG, ENABLE);
  
}


void RTC_Configuration(void)
{
  PWR_RTCAccessCmd(ENABLE);
  
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);
  
  RCC_LSEConfig(RCC_LSE_ON);
  
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }
  
  RCC_RTCCLKCmd(ENABLE);
  
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  
}

void conf_analog_all_GPIOS(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                          RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, ENABLE);
  
  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  
#if  DEBUG_SWD_PIN == 1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & (~GPIO_Pin_13) & (~GPIO_Pin_14);
#endif
  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Disable GPIOs clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                          RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);
}

void  Init_GPIOs(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  conf_analog_all_GPIOS();   /* configure all GPIOs as analog input */
  
  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(LD_GPIO_PORT_CLK | USERBUTTON_GPIO_CLK, ENABLE);
  
  /* USER button and WakeUP button init: GPIO set in input interrupt active mode */
  
  /* Configure User Button pin as input */
  GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);
  
  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  
  /* Configure User Button and IDD_WakeUP EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;  // PA0 for User button AND IDD_WakeUP
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set User Button and IDD_WakeUP EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  /* Configure the GPIO_LED pins  LD3 & LD4*/
  GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);
  GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
  GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
  
  /* Disable all GPIOs clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                          RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
}

void processTempData(void)
{
  ultrassom1 = ADC_ConvertedValueBuff[13];
  ultrassom2 = ADC_ConvertedValueBuff[14];
  ultrassom3 = ADC_ConvertedValueBuff[15];
  ultrassom4 = ADC_ConvertedValueBuff[16];
  ultrassom5 = ADC_ConvertedValueBuff[17];
}

uint16_t conversaoADCMilimetros(uint16_t ultrassomADC) {
  uint32_t ultrassomMilimetros32 = ((3130 * ultrassomADC) + 22413) / 1000;
  uint16_t ultrassomMilimetros16 = ultrassomMilimetros32;
  return ultrassomMilimetros16;
}

void Delay(uint32_t nTime)
{
  TimingDelay = nTime;
  
  while (TimingDelay != 0);
  
}

void TimingDelay_Decrement(void)
{
  
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
  
}

