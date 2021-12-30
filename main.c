#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "core_cm3.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_HC-SR04.h"
#include "lcd.h"
#include "touch.h"


/* function prototype */

void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);
void TIM_Configure(void);
void EXTI_Configure(void);
void USART1_IRQHandler();
void USART2_IRQHandler();


void sendDataUART1(uint16_t data);
void sendDataUART2(uint16_t data);

#define DHT11_H
#define DHT11_SUCCESS         1
#define DHT11_ERROR_CHECKSUM  2
#define DHT11_ERROR_TIMEOUT   3

int res;
int i;
int count = 0;
int fan_flag = 0;
int flag=1;
int dry_flag = 0;
int previous_flag = 0;

int32_t adc1;
int32_t adc2;   

int32_t dist=0;
int32_t dist2=0;

int color[12] =
{ WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY };

typedef struct DHT11_Dev {
  uint8_t temparature;
  uint8_t humidity;
} DHT11_Dev;

struct DHT11_Dev dev;

int DHT11_read(struct DHT11_Dev* dev);
volatile uint32_t ADC_Value[2];


//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
  // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
  //LED
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
  /* GPIO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  //조도센서
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* USART1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  /* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  //온습도센서
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  //모터
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

//초음파센서
void EnableHCSR04PeriphClock() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

void GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* LED pin setting*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /* UART pin setting */
  //TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*USART2 pin setting */
  //TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //온습도센서
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //조도센서1 channel_5
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //조도센서2 channel_4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //모터
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //버튼 인터럽틉
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}


void USART1_Init(void)
{
  USART_InitTypeDef USART1_InitStructure;
  // Enable the USART1 peripheral
  USART_Cmd(USART1, ENABLE);
  // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
  USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART1_InitStructure.USART_Parity = USART_Parity_No;
  USART1_InitStructure.USART_StopBits = USART_StopBits_1;
  USART1_InitStructure.USART_BaudRate = 9600;
  USART1_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' 
  //      and the argument value 'Receive Data register not empty interrupt'
  USART_Init(USART1, &USART1_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  NVIC_EnableIRQ(USART1_IRQn);
  
}

void USART2_Init(void)
{
  USART_InitTypeDef USART2_InitStructure;
  // Enable the USART1 peripheral
  USART_Cmd(USART3, ENABLE);
  // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART2_InitStructure.USART_Parity = USART_Parity_No;
  USART2_InitStructure.USART_StopBits = USART_StopBits_1;
  USART2_InitStructure.USART_BaudRate = 9600;
  USART2_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' 
  //      and the argument value 'Receive Data register not empty interrupt'
  USART_Init(USART3, &USART2_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  NVIC_EnableIRQ(USART3_IRQn);
}

void TIM_Configure(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //1us count
  TIM_TimeBaseStructure.TIM_Period = 1; 
  TIM_TimeBaseStructure.TIM_Prescaler = 72;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  
  //모터
  TIM_OCInitTypeDef TIM_OCInitStructure; 
  
  TIM_TimeBaseStructure.TIM_Period = 5000; 
  TIM_TimeBaseStructure.TIM_Prescaler = 7200;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;
  
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);
  
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}

void Forward(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Back(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Stop(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_8|GPIO_Pin_9);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Fan_on(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Fan_off(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_6|GPIO_Pin_7);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void EXTI_Configure(void)
{
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  EXTI_InitTypeDef EXTI_Init_Struct;
  EXTI_Init_Struct.EXTI_Line = EXTI_Line0;
  EXTI_Init_Struct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_Init_Struct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_Init_Struct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_Init_Struct);
  
}

void NVIC_Configure(void) {
  
  NVIC_InitTypeDef NVIC_InitStructure;
  // TODO: fill the arg you want
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // TODO: Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'
  // UART1
  // 'NVIC_EnableIRQ' is only required for USART setting
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // UART2
  NVIC_EnableIRQ(USART3_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //fan interrupt
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
}

void DMA_Configure(){
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADC_Value[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC_Configure()
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_239Cycles5);
  ADC_DMACmd(ADC1, ENABLE);
  
  ADC_Cmd(ADC1, ENABLE);
  
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1)){}
  
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1)){}
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void USART1_IRQHandler() {
  uint16_t word;
  if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
    // the most recent received data by the USART1 peripheral
    word = USART_ReceiveData(USART1);
    // TODO implement
    sendDataUART1(word);
    sendDataUART2(word);
    // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
  }
}

void USART3_IRQHandler() {
  uint16_t word;
  if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET){
    // the most recent received data by the USART1 peripheral
    word = USART_ReceiveData(USART3);
    // TODO implement
    sendDataUART1(word);
    //sendDataUART2(word);
    // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART3,USART_IT_RXNE);
  }
}

void sendDataUART1(uint16_t data) {
  USART_SendData(USART1, data);
  /* Wait till TC is set */
  //while ((USART1->SR & USART_SR_TC) == 0);
}

void sendDataUART2(uint16_t data) {
  USART_SendData(USART3, data);
  /* Wait till TC is set */
  //while ((USART1->SR & USART_SR_TC) == 0);
}

//Increase count (1us)
void TIM2_IRQHandler(void){
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    count++;
    if(count>6000000)
      count=0;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

//fan on-off
void EXTI15_10_IRQHandler(void) { // when the user S1 is pressed
  if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) == Bit_RESET) {
      if(fan_flag==0)
      {
        Fan_on();
        fan_flag=1;
      }
      else
      {
        Fan_off();
        fan_flag=0;
      }
      
      GPIO_SetBits(GPIOD, GPIO_Pin_3);
    }
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
}

void EXTI0_IRQHandler(void) { // when the Software Interrupted
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
    GPIO_SetBits(GPIOD,GPIO_Pin_2);
    if(count > 5000000){
      res=DHT11_read(&dev);
      if(dev.humidity<45)
        dry_flag = 1;
      else
        dry_flag = 0;
    }
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

//read 온습도value
int DHT11_read(struct DHT11_Dev* dev) {
  
  //Initialisation
  uint8_t i, j, temp;
  uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  GPIO_InitTypeDef GPIO_InitStructure;
  
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //Put LOW for at least 18ms
  GPIO_ResetBits(GPIOA, GPIO_Pin_0);
  
  //wait 18ms
  count=0;
  while(count <= 18000);
  
  //Put HIGH for 20-40us
  GPIO_SetBits(GPIOA, GPIO_Pin_0);
  
  //wait 40us
  count=0;
  while(count <= 40);
  //End start condition
  
  //io();
  //Input mode to receive data
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //DHT11 ACK
  
  //should be LOW for at least 80us
  //while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
  count = 0;
  while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
    if(count > 100)
      return DHT11_ERROR_TIMEOUT;
  }
  
  
  //should be HIGH for at least 80us
  //while(GPIO_ReadInputDataBit(dev->port, dev->pin));
  count = 0;
  while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
    if(count > 100)
      return DHT11_ERROR_TIMEOUT;
  }
  
  
  //Read 40 bits (8*5)
  
  for(j = 0; j < 5; ++j) {
    for(i = 0; i < 8; ++i) {
      
      //LOW for 50us
      //while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6));
      count = 0;
      while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
        if(count > 60)
          return DHT11_ERROR_TIMEOUT;
      }
      
      //Start counter
      count=0;
      
      //HIGH for 26-28us = 0 / 70us = 1
      while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
      
      //Calc amount of time passed
      temp = count;
      
      //shift 0
      data[j] = data[j] << 1;
      
      //if > 30us it's 1
      if(temp > 30)
        data[j] = data[j]+1;
    }
  }
  
  //verify the Checksum
  if(data[4] != (data[0] + data[1] + data[2] + data[3]))
    return DHT11_ERROR_CHECKSUM;
  
  //set data
  dev->temparature = data[2];
  dev->humidity = data[0];
  
  count=0;
  
  return DHT11_SUCCESS;
}

void delay() {  //delay function
  int i;
  for (i=0; i<10000; i++){}
}

int main(void)
{
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  USART1_Init();
  USART2_Init();
  ADC_Configure();
  DMA_Configure();
  NVIC_Configure();
  TIM_Configure();
  InitHCSR04();
  LCD_Init();
  EXTI_Configure();
  Touch_Configuration();
  LCD_Clear(WHITE);
  
  Stop();
  
  char msg_dry[] ="Dry is done!\r\n";
  
  while (1) {
    dist = HCSR04GetDistance2();              //mm단위    
    dist2 = HCSR04GetDistance();  //mm단위
    adc1=   ADC_Value[0];
    adc2=   ADC_Value[1];           
    LCD_ShowNum(40, 40, dist, 3, MAGENTA, WHITE);
    LCD_ShowNum(40, 60, dist2, 3, MAGENTA, WHITE);
    LCD_ShowNum(40, 120, (uint32_t)ADC_Value[0], 5, MAGENTA, WHITE);
    LCD_ShowNum(40, 140, (uint32_t)ADC_Value[1], 5, MAGENTA, WHITE);
    LCD_ShowNum(40, 80, (uint32_t)dev.temparature, 3, MAGENTA, WHITE);
    LCD_ShowNum(40, 100, (uint32_t)dev.humidity, 3, MAGENTA, WHITE);
    
    LCD_ShowNum(40, 160, count, 10, MAGENTA, WHITE);
    
    if ( adc1 - adc2 < 0) {
      if ( dist > 200 ){
        Stop();
        Forward();
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);
      }
      else {
        Stop();
        EXTI_GenerateSWInterrupt(EXTI_Line0);
      }
    }
    else if ( adc1 - adc2 > 0) {
      if ( dist2 > 200 ){
        Stop();
        Back();
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);
      }
      else {
        Stop();
        EXTI_GenerateSWInterrupt(EXTI_Line0);
        
      }
    }else{
      Stop();
      EXTI_GenerateSWInterrupt(EXTI_Line0);
    }
    
    if(dry_flag==1 && previous_flag ==0){
      for(int i=0; i<14; i++){
        sendDataUART2(msg_dry[i]);
        delay();
      }
    }
    previous_flag = dry_flag;
    
  }
  return 0;
}
