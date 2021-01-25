#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void _RCC_Init(void);
void _GPIO_Init(void);
void DisplayTitle(void);

void _ADC_Init(void);
void TIMER3_Init(void);
void TIMER6_Init(void);
void USART1_Init(void);
uint16_t KEY_Scan(void);
void _EXTI_Init(void); 
void USART_BRR_Configuration(uint32_t USART_BaudRate);

void SerialPutChar(uint8_t c);
void Serial_PutString(char *s);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

unsigned short ADC1_Value, ADC2_Value, MODE = 4; //ADC의 data register 값을 저장할 변수들과, PC command를 결정할 모드 저장 변수 정의
char str1[20], str2[20], str_T[20]; //ADC data register 값을 받아온 뒤 키트으이 LCD에 출력 하기 위해 문자열로 반드는데 필요한 변수 str1,2 그리고 PC로 송신할 문자열을 만들기 위한 변수 str_T


int main(void)
{
	_RCC_Init();
	LCD_Init();			// LCD 구동 함수
	DelayMS(10);		// LCD구동 딜레이
    
	DisplayTitle();		//LCD 초기화면구동 함수
	_GPIO_Init();
        _EXTI_Init();
	_ADC_Init();
        USART1_Init();
        TIMER3_Init();
        TIMER6_Init();          //GPIO, ADC, TIMER, USART 설정 함수
    
        ADC2->CR2 |= ADC_CR2_SWSTART; // 초기 상태 : ADC2 ON
	while(1)
	{

 	}
}

void ADC_IRQHandler(void) //ADC 핸들러
{
  if(ADC1->SR & 0x02)
  {
   //ADC1은 250ms마다 AD 변환을 하고, EOC 플래그가 발생한다. DR을 읽는 것으로 플래그를 지워준다.
    ADC1_Value = ADC1->DR;   // Read ADC1 result value from ADC1 Data Reg(ADC1->DR) 
    sprintf(str1,"%3X",ADC1_Value); //string 을 만든다 16진수로 표시 하기 위해 %X(16진수로 표현)을 썼다.
    LCD_DisplayText(1,9,str1); //LCD에 ADC1 값 출력
  }
  
  if(ADC2->SR & 0x02)
  {
    //ADC2은 연속적으로 AD 변환을 하고, EOC 플래그가 발생한다. DR을 읽는 것으로 플래그를 지워준다.
    ADC2_Value = ADC2->DR;  // Read ADC2 result value from ADC2 Data Reg(ADC2->DR) 
    sprintf(str2,"%4d",ADC2_Value); //string 을 만든다
    LCD_DisplayText(2,5,str2); //LCD에 ADC2 값 출력
  }

}
  
void USART1_IRQHandler(void) //수신 인터럽트를 사용하기 위해 USART 핸들러 사용
{       
	if ( (USART1->SR & USART_SR_RXNE) ) //수신 레지스터가 채워지면 인터럽트가 발생한다.
	{
		char ch; // 수신된 값을 받아올 변수
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장

                switch(ch) //수신된 문자에 따라 모드를 설정한다.
                {
                case (0x31) : MODE = 1; break; //1(아스키 코드 0x31) 을 수신하면 모드1
                case (0x32) : MODE = 2; break; //2(아스키 코드 0x32) 을 수신하면 모드2
                case (0x33) : MODE = 3; break; //3(아스키 코드 0x33) 을 수신하면 모드3
                case (0x34) : MODE = 4; break; //4(아스키 코드 0x34) 을 수신하면 모드4
                }

	} 
        // DR 을 읽으면 SR.RXNE bit(flag bit)는 자동으로 clear 된다.
  
}

void _ADC_Init(void) //ADC1 : PA1, ADC2 : PC1
{   
        //ADC1 (PA1)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// 0x00000001  // ENABLE GPIOA CLK
	GPIOA->MODER |= GPIO_MODER_MODER1;       // 0x0000000C	// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// 0x00000100  // ENABLE ADC1 CLK

        ADC->CCR &= ~0X0000001F;	// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= 0x00010000;		// ADCPRE: ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
	ADC->CCR &= ~0x0000C000;	// DMA: Disable adc 공통된 설정

        ADC1->CR1 &= ~(3<<24);		// RES[1:0]: 12bit Resolution ( res - resolution )
        ADC1->CR1 &= ~(1<<8);		// SCAN: ADC_ScanCovMode Disable
        ADC1->CR1 |= ( 1 << 5);            // EOC 인터럽트 발생 허용
        
        ADC1->CR2 |= (3<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(RISING FALLING 둘다 설정, 토글 될때마다 동작)
	ADC1->CR2 |= (7 << 24);	        // EXTSEL[3:0]: ADC_ExternalTrig (TIM3_CC1)
        
        ADC1->CR2 &= ~(1<<1);		// CONT: ADC_ContinuousConvMode Disable
        ADC1->CR2 &= ~(1<<11);		// ALIGN: ADC_DataAlign_Right
        ADC1->CR2 |= (1<<10);		// EOCS: The EOC bit is set at the end of each regular conversion 250ms마다 EOC 비트 set.
        
        ADC1->SQR1 &= ~0x00F00000;	// L[3:0]: ADC Regular channel sequece length = 1 conversion
        
        ADC1->SMPR2	|= 0x07 << (3*1);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1) 샘플링타임. 크게 중요한건 아님
        
                //Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
        ADC1->SQR3 |= 0x01<<0;
        
        ADC1->CR2 |= (1<<0);		// ADON: ADC1 ON
        
         //ADC2 (PC1)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // ENABLE GPIOC CLK
        GPIOC->MODER |= GPIO_MODER_MODER1;  // CONFIG GPIOC PIN1(PC1) TO ANALOG IN MODE
        RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// ENABLE ADC2 CLK

        ADC->CCR &= ~0X0000001F;	// MULTI[4:0]: ADC_Mode_Independent
        ADC->CCR |= 0x00010000;		// ADCPRE: ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        ADC->CCR &= ~0x0000C000;	// DMA: Disable adc 공통된 설정

        ADC2->CR1 &= ~(3<<24);		// RES[1:0]: 12bit Resolution ( res - resolution )
        ADC2->CR1 &= ~(1<<8);		// SCAN: ADC_ScanCovMode Disable
        ADC2->CR1 |= ( 1 << 5);    // EOC 인터럽트 발생 허용

        ADC2->CR2 |= (1<<1);		// CONT: ADC_ContinuousConvMode Enable
        ADC2->CR2 &= ~(3<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_None
        ADC2->CR2 &= ~(1<<11);		// ALIGN: ADC_DataAlign_Right
        ADC2->CR2 &= ~(1<<10);		// EOCS 설정 X (연속 모드이므로)
        
        ADC2->SQR1 &= ~0x00F00000;	// L[3:0]: ADC Regular channel sequece length = 1 conversion
        
        //Channel selection, The Conversion Sequence of PIN1(ADC2_CH11) is first, Config sequence Range is possible from 0 to 17
        ADC2->SQR3 |= 0x0b;
        
        ADC2->CR2 |= 1<<0;		// ADON: ADC2 ON
        
        NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
       

}

void TIMER3_Init(void) //TIM3 설정
{
        //TIMER3 SET (PA6)
       	RCC->AHB1ENR |= 0x01; //ENABLE GPIOA
       	GPIOA->MODER 	|= ( 3 << 12 );	// GPIOA PIN6 Output ★Alternate function mode★					
	GPIOA->OSPEEDR 	|= ( 3 << 12 );	// GPIOA PIN6 Output speed (100MHz High speed)
	GPIOA->OTYPER	= 0x00000000;	// GPIOA PIN6 Output type push-pull (reset state)
	GPIOA->PUPDR	        |= ( 1 << 12 );	// GPIOA PIN6 Pull-up
  					// PA6 ==> TIM3_CH1
	GPIOA->AFR[0]	        |= ( 1 << 24 );	// (매뉴얼에서 봐야함, AFR[0] 의 AF2 ( TIM3 )
 
        
       	RCC->APB1ENR |= 0x02;// RCC_APB1ENR TIMER3 Enable
        NVIC->ISER[0] |= ( 1 << 29 ); // Enable Timer3 global Interrupt
        
        TIM3->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM3->ARR	= 2500-1;	// Auto reload  (0.1ms * 2500 = 250ms : Period) //250ms마다 트리거, ADC가 동작
        //falling edge에서만 동작하도록 설정했으면 주기를 반으로 줄여야 하는데, 토글시에 동작하도록 설정했으므로 주기를 250ms로 하여도 된다.

        TIM3->CCER	|= (1<<0);	// CC1E: OC1 Active(Capture/Compare 1 output enable)
        TIM3->CCER	&= ~(1<<1);	// CC1P: Capture/Compare 1 output Polarity High
 
        TIM3->CCR1	= 10;		// CCR1 value (임의의 초기값)
        
        TIM3->CCMR1 &= ~(3<<0); // CC1S(CC1 channel): Output 
        TIM3->CCMR1 &= ~(1<<3); // OC1PE: Output Compare 1preload disable
        TIM3->CCMR1 |= (3<<4);	// OC1M: Output Compare 1 Mode : toggle

	TIM3->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM3->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM3->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	
       	TIM3->DIER |= (1<<0);	//Enable the Tim3 Update interrupt
       	TIM3->EGR |=(1<<0);	//Update generation 
       	TIM3->CR1	|= (1<<0);	// CEN: Counter TIM3 enable
}

void TIM3_IRQHandler(void) //TIM3 핸들러
{
  if((TIM3->SR & 0x01) != RESET)
  {
    TIM3->SR &= ~0x01; //TIM3 인터럽트가 발생하면, 플래그를 클리어 해준다.
  }
  
  if((TIM3->SR & 0x02) != RESET)
  {
    TIM3->SR &= ~0x02; //TIM3 CC 인터럽트가 발생하면, 플래그를 클리어 해준다.
  }
  
}

void TIMER6_Init(void)
{
//TIMER6 SET
       	RCC->APB1ENR |= 0x10; //ENABLE  TIM6 ( 기본 타이머라 GPIO 설정을 하지 않아도 된다.)
       
        NVIC->ISER[1] |= ( 1 << (54-32) ); // Enable Timer6 global Interrupt
        
        TIM6->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM6->ARR	= 100-1;	// Auto reload  (0.1ms * 100 = 10ms : Period) //10ms마다 인터럽트 발생!

	TIM6->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM6->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM6->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	
       	TIM6->DIER       |= (1<<0);	//Enable the Tim6 Update interrupt
       	TIM6->EGR       |=(1<<0);	//Update generation 
       	TIM6->CR1	|= (1<<0);	// CEN: Counter TIM6 enable
}

void TIM6_DAC_IRQHandler(void) //TIM6 핸들러
{
  if((TIM6->SR & 0x01) != RESET)
  {
    TIM6->SR &= ~0x01; //TIM6 인터럽트가 발생하면, 플래그를 클리어 해준다.
    
          if( MODE == 1 ) //모드 1 : 지자기 센서 값만 업로드
          {
            sprintf(str_T,"H : 0x%3X\r\n",ADC1_Value); 
            //ADC1값을 저장한 변수를 통해 PC에 보낼 문자열을 만든다.
            //\r\n ( return carriage : 캐럿이 줄 맨 앞으로, line feed : 캐럿이 다음줄로)를 통해 줄바꿈을 한다.
            Serial_PutString(str_T); //PC에 만든 문자열을 보낸다.
          }
          else if( MODE == 2) //모드 2 : 가속도 센서 값만 업로드
          {
            sprintf(str_T,"A : %4d\r\n",ADC2_Value);
            //ADC2값을 저장한 변수를 통해 PC에 보낼 문자열을 만든다.
            //\r\n ( return carriage : 캐럿이 줄 맨 앞으로, line feed : 캐럿이 다음줄로)를 통해 줄바꿈을 한다.
            Serial_PutString(str_T); //PC에 만든 문자열을 보낸다.
          }
          else if( MODE == 3) //모드 3: 두 센서값 동시 업로드
          {
            sprintf(str_T,"H : 0x%3X A : %4d\r\n",ADC1_Value, ADC2_Value);
            //ADC1, ADC2값을 저장한 변수를 통해 PC에 보낼 문자열을 만든다.
            //\r\n ( return carriage : 캐럿이 줄 맨 앞으로, line feed : 캐럿이 다음줄로)를 통해 줄바꿈을 한다.
            Serial_PutString(str_T); //PC에 만든 문자열을 보낸다.
          }
          else if( MODE == 4) //업로드 정지
          {
          }
  }
}

void USART1_Init(void) //USART 설정
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR	|= 0x01;	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER	|= 0x00080000;	// GPIOA PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR	|= 0x000C0000;	// GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->OTYPER	|= 0x00000000;	// GPIOA PIN9 Output type push-pull (reset state)
	GPIOA->PUPDR	|= 0x00040000;	// GPIOA PIN9 Pull-up
	GPIOA->AFR[1]	|= 0x70;	// Connect GPIOA pin9 to AF7(USART1)
    
	// USART1 : RX(PA10)
	GPIOA->MODER 	|= 0x200000;	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR	|= 0x00300000;	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->AFR[1]	|= 0x700;	//Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR	|= 0x0010;	// RCC_APB2ENR USART1 Enable
    
	USART_BRR_Configuration(19200); // USART Baud rate Configuration
    
        //***패리티 사용 안할때***
	USART1->CR1	&= ~USART_CR1_M;	// USART_WordLength 8 Data bit = (1 << 12)
        USART1->CR1	&= ~USART_CR1_PCE ;	// USART_Parity_No
        
        //***패리티 사용 할 때( 데이터 비트 9, 홀수 패리티***
//        USART1->CR1	|= 1<<12;	// USART_WordLength 8 Data bit = (1 << 12)
//        USART1->CR1	|= 1<<10 ; //패리티 사용
//         USART1->CR1	|= 1<<9 ; // 홀수 패리티
         
	USART1->CR1	|= USART_CR1_RE;	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= USART_CR1_TE ;	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2	&= ~USART_CR2_STOP;	// USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= USART_CR1_RXNEIE;	//  0x0020, RXNE interrupt Enable

	NVIC->ISER[1]	|= (1 << 5); 	// Enable Interrupt USART1 (NVIC 37번) (37 - 32)
	USART1->CR1 	|= USART_CR1_UE;	//  0x2000, USART1 Enable
}

void _GPIO_Init(void)
{
	// LED GPIO(PORT G) 설정
    	RCC->AHB1ENR    |= 0x00000040; 	// RCC_AHB1ENR(bit8~0) GPIOG(bit#6) Enable							
	GPIOG->MODER    = 0x00005555;	// GPIOG PIN0~PIN7 Output mode (0b01)						
	GPIOG->OTYPER   = 0x0000;	// GPIOG PIN0~PIN7 : Push-pull  (PIN8~PIN15) (reset state)	
 	GPIOG->OSPEEDR  = 0x00005555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 
    
	// SW GPIO(PORT H) 설정 
	 RCC->AHB1ENR    |= 0x00000080;	// RCC_AHB1ENR(bit8~0) GPIOH(bit#7) Enable							
	 GPIOH->MODER    = 0x00000000;	// GPIOH PIN8~PIN15 Input mode (reset state)				
	 GPIOH->PUPDR    = 0x00000000;	// GPIOH PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
 //다른 것들은 쓸 일이 없어서 삭제
}

void _EXTI_Init(void)    //EXTI SW0~4 설정
{
    	RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	//Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	

	 SYSCFG->EXTICR[2] |= 0x7777;          //EXTI 11,10,9,8에 대한 소스 입력은 GPIOH로 설정 (EXTICR3)
        SYSCFG->EXTICR[3] |= 0x0007;          //EXTI 12에 대한 소스 입력은 GPIOH로 설정 (EXTICR4)
        
	 EXTI->FTSR     |= 0x1f00;		// Falling Trigger Enable 

    	EXTI->IMR       |= 0x1f00;  	// 인터럽트 mask (Interrupt Enable)

	 NVIC->ISER[0] |= ( 1 << 23 ); // Enable Interrupt EXTI9~5 Vector table Position 참조
     NVIC->ISER[1] |= ( 1 << 8 ); // Enable Interrupt EXTI15~10 Vector table Position 참조
}

void EXTI15_10_IRQHandler(void) //EXTI10~15 인터럽트 핸들러
{
        
         if(EXTI->PR & 0x0800) //SW3 EXTI 
        {
              EXTI->PR |= 0x0800; //clear flag
              
              ADC1->CR2 |= 1<<0;	        // ADC1 enable
              ADC1->SR &= ~(1<<1);      //ADC1 EOC CLEAR
              
             GPIOG->ODR &= ~( 0x1f );   //LED ALL OFF
             GPIOG->ODR |= ( 1 << 3);   //LED3 ON
        }
        
         if(EXTI->PR & 0x1000) //SW4 EXTI 
        {
              EXTI->PR |= 0x1000; //clear flag
           
              ADC2->CR2  |= (1<<0);             //ADC2 enable
              ADC2->CR2 |= ADC_CR2_SWSTART;     //ADC2 연속읽기 시작( 처음 시작시에만 사용 )
              
             GPIOG->ODR &= ~( 0x1f );   //LED ALL OFF
             GPIOG->ODR |= ( 1 << 4);   //LED 4 ON
        }
}

void EXTI9_5_IRQHandler(void) //EXTI5~9 인터럽트 핸들러
{
         if(EXTI->PR & 0x0100) //SW0 EXTI 
        {
              EXTI->PR |= 0x0100; //clear flag
              
             ADC1->CR2  &= ~(1<<0);     // ADC1 disable
             ADC1->SR |= (1 << 1);      //ADC1 EOC SET
             
             GPIOG->ODR &= ~( 0x1f );   //LED ALL OFF
             GPIOG->ODR |= ( 1 << 1);   //LED1 ON
        }
        
         if(EXTI->PR & 0x0200) //SW1 EXTI 
        {
              EXTI->PR |= 0x0200; //clear flag
              
             ADC2->CR2  &= ~(1<<0);     //ADC2 disable
             
             
             GPIOG->ODR &= ~( 0x1f );   //LED ALL OFF
             GPIOG->ODR |= ( 1 << 2);   //LED2 ON
        }
        
}



void SerialPutChar(uint8_t Ch) // 1문자 보내기 함수
{
        while((USART1->SR & USART_SR_TXE) == RESET); //  USART_SR_TXE:0x0080, 송신 가능한 상태까지 대기 SR레지스터의 TXE 플래그가 1이면 비어있고 0이면 차있다!!!!!!!!!!!!!!!!!!!!!!!!!!!! TRUE면 계속 while문만 돈다. FALSE가 되보리면 비어있으니까 전송스한다

	USART1->DR = (Ch & 0x01FF);	// 전송 (전송은 인터럽트 없음)
}

void Serial_PutString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialPutChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}

// Baud rate  
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency USART 2345일때는 42000000로 바꿔주어야한다.
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Determine the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8 : 0x8000
        {                                                                  // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Integer part computing in case Oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Integer part computing in case Oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Determine the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)
	{
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else // if ((USARTx->CR1 & USART_CR1_OVER8) == 0) 
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}


void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
		for(; Dly; Dly--);
}

void DisplayTitle(void)
{
	LCD_Clear(RGB_YELLOW);
	LCD_SetFont(&Gulim10);		//폰트 
        
	LCD_SetBackColor(RGB_BLUE);	//글자배경색
	LCD_SetTextColor(RGB_BLACK);	//글자색
       	LCD_DisplayText(0,0,"Sensor");
        
        LCD_SetBackColor(RGB_YELLOW);	//글자배경색
        LCD_DisplayText(1,0,"Height:0x000");
      	LCD_DisplayText(2,0,"ACCx:0000");

	LCD_SetBackColor(RGB_YELLOW);	//글자배경색
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		key_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
        		return 0xFF00;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}

/******************************************************************************/
/*     RCC Set up                                                             */
/******************************************************************************/
void _RCC_Init(void)
{
    // PLL (clocked by HSE) used as System clock source                    

    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    // Enable HSE : 외부 입력 클락주파수: 5MHz(회로도 참조)
    RCC->CR |= 0x00010000;	// Set bit#16 of CR
 
    // Wait till HSE is ready and if Time out is reached exit 
    do
    {
	HSEStatus = RCC->CR & 0x00020000;	// CHECK bit#17 of CR (HSE RDY?) 
	StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CR & 0x00020000) != RESET)	// CHECK bit#17 of CR (HSE RDY?) // RESET is 0
    {
	HSEStatus = 0x01;	// HSE is Ready!
    }
    else
    {
	HSEStatus = 0x00;	// HSE is NOT Ready!
    }

    if (HSEStatus == 0x01)	// HSE clock Enable
    {
	// HCLK = SYSCLK / 1 (HCLK = 168MHz)
	RCC->CFGR |= 0x00000000;
 
	// PCLK2 = HCLK / 2  (PCLK2 = 84MHz)
	RCC->CFGR |= 0x00008000;	// PPRE2: APB(Advanced Peripheral Bus)(APB2) High-speed prescaler
					// 100: AHB clock divided by 2 

	// PCLK1 = HCLK / 4  (PCLK1 = 42MHz)
	RCC->CFGR |= 0x00001400;	// PPRE1: APB(Advanced Peripheral Bus)(APB1) Low-speed prescaler
					// 101: AHB clock divided by 4 

    	// Configure the main PLL 
	// Reset vlaue: 0x2400 3010 (PPLQ:4, PLLSR:0, PLL_M:16, PLL_N:192, PLL_P: 2(00))
        RCC->PLLCFGR = 0;
	RCC->PLLCFGR |= 8;		// PLL_M(6bits): 8(0b001000): /8
	RCC->PLLCFGR |= (336 << 6);	// PLL_N(9bits): 336 : *336
	RCC->PLLCFGR |= (0<<16);	// PLL_P(2bits): (2 >> 1) -1=0b00 : 2 : /2 
	RCC->PLLCFGR |= 0x00400000; 	// PLL_SR(1bit): 1 : HSE oscillator clock selected as PLL and PLLI2S clock
//      RCC->PLLCFGR = 0x24405408;
	// SYSCLK 계산 (HSE 입력클럭주파수: 8MHz)
	// SYSCLK = 8M * 336(N) /8(M) /2(P) = 168MHz	
    
	// Enable the main PLL 
	RCC->CR |= 0x01000000;	// Set bit#24 of CR : PLL ON

	// Wait till the main PLL is ready 
	while((RCC->CR & 0x02000000) == 0)	// CHECK bit#25 : PLL RDY?  
	{}
   
	// Configure Flash prefetch, Instruction cache, Data cache and wait state 
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	// Select the main PLL as system clock source 
	// Reset value of RCC->CFGR: 0x0000 0000
	RCC->CFGR &= ~0x00000003;	// clear, (Reset value: HSI) 
	RCC->CFGR |= 0x00000002;	// PLL

	// Wait till the main PLL is used as system clock source 
	while ((RCC->CFGR & 0x0000000C ) != 0x00000008);	// CHECK bit#2~3 : PLL as system clock is RDY?  
	{}
    }
    else
    { // If HSE fails to start-up, the application will have wrong clock
        // configuration. User can add here some code to deal with this error 
    }
}

