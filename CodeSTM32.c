#include "stm32f4xx.h"
#include "stdio.h"

#define BUFFER_SIZE_RX 1
#define BUFFER_SIZE_TX 1

int8_t rxbuff[BUFFER_SIZE_RX];
char txbuff[BUFFER_SIZE_TX];

/* ENCODER */
float input_capture, pre_capture, captureOnthefly = 0 ;
float pulse_perSecond, time ;
float speed;
int32_t Output_int;
int run = 0;
float Pulse=0, Completed_Circle=0;

///// TypeDef /////
GPIO_InitTypeDef GPIO_InitStruct;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructre;
TIM_ICInitTypeDef TimerIC_Confi;
USART_InitTypeDef USART_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;

///// Void /////
void TimerCaptureConf(void);
void USART_Config(void);
void readEncoder(void);
void PCTransmission(void);
void TIMCounterConfig(void);
void delay( __IO uint32_t nCount);

///// Main /////
int main(){
	SystemInit();
 	TimerCaptureConf();
 	USART_Config();
	TIMCounterConfig();
	
	while(1){
		//readEncoder();	
		if (run == 1){
			PCTransmission();	
		}
		else{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
		}
	}
}

void TIMCounterConfig(void){
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseInitStructre.TIM_ClockDivision = 0;
  	TIM_TimeBaseInitStructre.TIM_Period = 0xFFFF;
  	TIM_TimeBaseInitStructre.TIM_Prescaler = 8400 - 1;

  	TIM_TimeBaseInit (TIM6, & TIM_TimeBaseInitStructre);
  	TIM_Cmd(TIM6, ENABLE);

  	TIM6->CNT = 0;
}
void TimerCaptureConf(void){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15| GPIO_Pin_13| GPIO_Pin_12; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOD, &GPIO_InitStruct); 
	
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
  	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_TIM3);

	// Timer Capture Configuration
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  	TIM_TimeBaseInitStructre.TIM_Period = 10000 - 1 ;
  	TIM_TimeBaseInitStructre.TIM_Prescaler = (8400 - 1);
  	TIM_TimeBaseInitStructre.TIM_ClockDivision = 0;
  	TIM_TimeBaseInitStructre.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit (TIM3, &TIM_TimeBaseInitStructre);

  	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  	// Timer Input Capture Configuration
  	TimerIC_Confi.TIM_Channel = TIM_Channel_1 ;
  	TimerIC_Confi.TIM_ICFilter = 15;
  	TimerIC_Confi.TIM_ICPolarity = TIM_ICPolarity_Rising ;
  	TimerIC_Confi.TIM_ICSelection = TIM_ICSelection_DirectTI;
  	TIM_ICInit(TIM3, &TimerIC_Confi);
  	TIM_Cmd(TIM3, ENABLE);
}

void readEncoder(void){
	if (TIM_GetFlagStatus (TIM3, TIM_FLAG_CC1) == SET){
		//if (GPIO_ReadInputDataBit(GPIOB, Channel_B) == RESET )
		TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
		// Determine direction using Channel_B
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == RESET)
			{
				Pulse++;
			}
		// else
		// 	{
				
		// 		Pulse++;
		// 	}
			
		// SPEED //
			
		input_capture= TIM_GetCapture1(TIM3);					// channel_A 
		time = input_capture - pre_capture + captureOnthefly*10000;
		captureOnthefly = 0;
		pre_capture = input_capture;
		
		if(time != 0) 
			pulse_perSecond = 10000/time ;						// 1 pulse = time
																// ? pulse = 10000 (second)												
		else 
			pulse_perSecond = 0;
			speed = pulse_perSecond*60/858;					// rpm = pulse_perSecond / encoder resolution
		
		
		//  Rounding  //
		if ( (speed - (int)speed ) >= (double)0.5)
				speed = (int)speed + 1;
		
		else	
				speed = (int)speed ;
		
		if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != SET)
			captureOnthefly ++;	
	}				
}
	
void USART_Config(void){
  	RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
  	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB , ENABLE);
  	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2, ENABLE);

  	// GPIO Configuration
  	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP ;
  	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed ;
  	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

  	GPIO_Init(GPIOB, &GPIO_InitStruct);

  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

  	USART_InitStructure.USART_BaudRate = 9600;
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;
  	USART_InitStructure.USART_Parity = USART_Parity_No;
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b ;

  	USART_Init(USART1, &USART_InitStructure);
  	USART_Cmd(USART1, ENABLE);

  	// Enable USART1 DMA
  	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

  	// DMA2 Stream2 Channel4 for USART1 Rx configuration
  	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE_RX;			
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff	;		
		DMA_InitStructure.DMA_MemoryBurst =DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_MemoryDataSize = 	DMA_MemoryDataSize_Byte	;			
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable			;					
		//MemoryInc: specifies whether memory address be incremented or not
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal 	;		
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&USART1->DR	; 
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_PeripheralBurst= DMA_PeripheralBurst_Single;
		
		DMA_Init (DMA2_Stream2,&DMA_InitStructure );
		DMA_Cmd (DMA2_Stream2, ENABLE);
		
		//Cau hinh DMA cho TX
		DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE_TX;			//TX_Buffer_size
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff	;		
		DMA_InitStructure.DMA_MemoryBurst =DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_MemoryDataSize = 	DMA_MemoryDataSize_Byte	;			
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable			;				1
		//MemoryInc: specifies whether memory address be incremented or not
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal 	;	
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&USART1->DR	; 
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;	
		DMA_InitStructure.DMA_PeripheralBurst= DMA_PeripheralBurst_Single;
		
		DMA_Init (DMA2_Stream7,&DMA_InitStructure );
		DMA_Cmd (DMA2_Stream7, ENABLE);
		
		// Enable DMA Interrupt to the highest priority
  	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
  	// Transfer complete interrupt mask
  	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
}

void PCTransmission(void){
	if(TIM6-> CNT >= 100){
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
			
		speed = 21;
		
		txbuff[0]= speed;
		
		
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
		DMA2_Stream7->NDTR=13;
		DMA_Cmd(DMA2_Stream7,ENABLE);
		TIM6->CNT=0;
	}
}

void DMA2_Stream2_IRQHandler(void) {
  	// Clear the DMA2_Stream2 TCIF2 pending bit
  	DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
  	//No new input after finish these package
  	DMA_Cmd(DMA2_Stream2, DISABLE);
	
	if(rxbuff[0]=='S')
		{
			run = 1;
		}
	else run = 0;
	DMA_Cmd(DMA2_Stream2, ENABLE);
}
void delay( __IO uint32_t nCount)
{
while(nCount--)
	{
	}
}

/*
PB6 - RXD of UART
PB7 - TXD of UART
PC6 - CHANNEL A
PC7 - CHANNEL B
*/



