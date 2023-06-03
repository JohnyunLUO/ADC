#include "stm32f10x.h"
#include "GPIO_STM32F10x.h"
#include "string.h"
#include <math.h>

//ADC 腳位設定
static void ADC_GPIO_config(void){
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_1;             /*!< Specifies the GPIO pins to be configured.
                                      This parameter can be any value of @ref GPIO_pins_define */
 
  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;    /*!< Specifies the operating mode for the selected pins.
                                      This parameter can be a value of @ref GPIOMode_TypeDef */
	
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
}

static void ADC_mode_config(void){
	
	ADC_InitTypeDef ADC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
	
	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;  /*!< Configures the ADC to operate in independent or
                                               dual mode. 
                                               This parameter can be a value of @ref ADC_mode */

  ADC_InitStruct.ADC_ScanConvMode=DISABLE;       /*!< Specifies whether the conversion is performed in
                                               Scan (multichannels) or Single (one channel) mode.
                                               This parameter can be set to ENABLE or DISABLE */

  ADC_InitStruct.ADC_ContinuousConvMode=ENABLE; /*!< Specifies whether the conversion is performed in
                                               Continuous or Single mode.
                                               This parameter can be set to ENABLE or DISABLE. */

  ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;          /*!< Defines the external trigger used to start the analog
                                               to digital conversion of regular channels. This parameter
                                               can be a value of @ref ADC_external_trigger_sources_for_regular_channels_conversion */

  ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;                 /*!< Specifies whether the ADC data alignment is left or right.
																																		This parameter can be a value of @ref ADC_data_align */

  ADC_InitStruct.ADC_NbrOfChannel=1;               /*!< Specifies the number of ADC channels that will be converted
                                               using the sequencer for regular channel group.
                                               This parameter must range from 1 to 16. */
	
	ADC_Init(ADC1,&ADC_InitStruct);
	
	//Configures the ADC clock (ADCCLK) 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	//通道轉換(ADC選擇,Channel選擇,優先順續,採樣週期)
	ADC_RegularChannelConfig( ADC1, ADC_Channel_11,1, ADC_SampleTime_239Cycles5);
	
	//ADC轉換結束後進入中斷，在中斷程序中讀取轉換值
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);
	
	//ADC校準初始化Resets the selected ADC calibration registers.
	ADC_ResetCalibration(ADC1);
	
	//檢驗初始化是否完成(完成回傳SET)
	while(ADC_GetResetCalibrationStatus(ADC1)){
	//ADC開始校正
	ADC_StartCalibration(ADC1);
	}
	//檢驗校正是否完成
	while(ADC_GetCalibrationStatus(ADC1)){
	//軟體觸發中斷
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	}
}
//NVIC
void ADC_NVIC_config(){
	NVIC_InitTypeDef NVIC_InitStruct;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	NVIC_InitStruct.NVIC_IRQChannel=ADC1_2_IRQn;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be a value of @ref IRQn_Type 
                                                   (For the complete STM32 Devices IRQ Channels list, please
                                                    refer to stm32f10x.h file) */

  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */

  NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */

  NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init( &NVIC_InitStruct);
}


//USART_configuration
void USART_config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//GPIO TX RX Channel
	GPIO_InitTypeDef GPIO_InitStruct;
	
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10 ;             /*RX !< Specifies the GPIO pins to be configured.
                                      This parameter can be any value of @ref GPIO_pins_define */

  //GPIO_InitStruct.GPIO_Speed;  /*!< Specifies the speed for the selected pins.
                                      //This parameter can be a value of @ref GPIOSpeed_TypeDef */

  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;    /*!< Specifies the operating mode for the selected pins.
                                      This parameter can be a value of @ref GPIOMode_TypeDef */
	
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9 ;             /*RX !< Specifies the GPIO pins to be configured.
                                      This parameter can be any value of @ref GPIO_pins_define */

  GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;  /*!< Specifies the speed for the selected pins.
																								This parameter can be a value of @ref GPIOSpeed_TypeDef */

  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;   
	
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	
	USART_InitTypeDef USART_InitStruct;
	
	USART_InitStruct.USART_BaudRate=115200;            /*!< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

  USART_InitStruct.USART_WordLength=USART_WordLength_8b;          /*!< Specifies the number of data bits transmitted or received in a frame.
																																		This parameter can be a value of @ref USART_Word_Length */

  USART_InitStruct.USART_StopBits=USART_StopBits_1;            /*!< Specifies the number of stop bits transmitted.
																																This parameter can be a value of @ref USART_Stop_Bits */

  USART_InitStruct.USART_Parity=USART_Parity_No;              /*!< Specifies the parity mode.
																															 This parameter can be a value of @ref USART_Parity
																															 @note When parity is enabled, the computed parity is inserted
																																		 at the MSB position of the transmitted data (9th bit when
																																		 the word length is set to 9 data bits; 8th bit when the
																																		 word length is set to 8 data bits). */
 
  USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;                /*!< Specifies wether the Receive or Transmit mode is enabled or disabled.
                                                                     This parameter can be a value of @ref USART_Mode */

  USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None; /*!< Specifies wether the hardware flow control mode is enabled
																													 or disabled.
																													 This parameter can be a value of @ref USART_Hardware_Flow_Control */
	
	
	USART_Init(USART1,&USART_InitStruct);
	
	
	USART_Cmd(USART1,ENABLE);//-
}

void delay(int us){
	while(us--){
		int count=9;
		while(count--){}
	}
}


uint16_t conversionvalue=0;
int ttt=1,i=0;
char buffer[20]={0};

int main(void)
{
	//while(ttt){}
	ADC_GPIO_config();
	ADC_mode_config();
	USART_config();
	ADC_NVIC_config();
	
	//我希望印出中文"開始測試"????????????????????
	
	while(1){
	  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	  sprintf(buffer,"%d\n\r",conversionvalue);	
		for(i=0;i<20;i++){  //-
		USART_SendData(USART1,buffer[i]);
		delay(10);
		}
		delay(0XFFFF);
	}
	
	return 0;
}
void ADC1_2_IRQHandler (void){
			
	    /*ADC_IT_EOC: End of conversion interrupt mask 是指模數轉換器 (ADC) 中的「轉換結束中斷屏蔽位」。
			當 ADC 完成一次模數轉換時，它會生成一個轉換結束中斷，以通知處理器可以讀取轉換結果。
			ADC_IT_EOC 是一個寄存器中的特定位，用於控制是否屏蔽（禁用）轉換結束中斷。當 ADC_IT_EOC 位被設置為 1 時，
			表示允許轉換結束中斷；當 ADC_IT_EOC 位被設置為 0 時，表示禁用轉換結束中斷。
			通過控制 ADC_IT_EOC 位的值，可以靈活地控制是否觸發轉換結束中斷，並根據需要進行處理器的相應操作。
			當 ADC_IT_EOC 位被設置為 0 時，表示禁用轉換結束中斷。這樣的設置通常是為了避免在中斷處理程序尚未完成時發生新的中斷。
			在一些應用中，中斷處理程序可能需要一些時間來完成特定的任務，例如儲存轉換結果、更新狀態等。如果不屏蔽轉換結束中斷，
			並在中斷處理程序中觸發新的中斷，可能會導致中斷重疊和競爭條件。*/
	
	if (ADC_GetITStatus(ADC1,ADC_IT_EOC)==SET){	//-
		
		 conversionvalue=ADC_GetConversionValue(ADC1);
		
	}
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC); //-
}	
	
	


  

void SystemInit(void)
{	
}




/*********************************************END OF FILE**********************/
