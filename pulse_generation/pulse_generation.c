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
void BEEP(void);

void TIMER2_Init(void);
void USART1_Init(void);
uint16_t KEY_Scan(void);
uint16_t KEY_Scan_Navi(void);
void _EXTI_Init(void); 
void USART_BRR_Configuration(uint32_t USART_BaudRate);
void Set_Clock(int h, int m);

void SerialPutChar(uint8_t c);
void Serial_PutString(char *s);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

int p_count = 0, p2_count = 0, g_spec = 0, g_degree = 0, g_time = 0, p = 0, t = 0;
char spec[3], degree[4], time[3], pulse[4], speed[4], c_pulse[4];
char ch_buffer[3]; //문자열을 받아 오기 위한 변수
uint8_t ch_count = 0; //문자열을 받아 오기 위한 카운트 변수



int main(void)
{
	_RCC_Init();
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);		// LCD구동 딜레이
    
	DisplayTitle();		//LCD 초기화면구동 함수
	_GPIO_Init();
        _EXTI_Init();
        USART1_Init();

        TIMER2_Init();            //GPIO, EXTI, TIMER, USART 설정 함수
    
	while(1)
	{

            
            switch(KEY_Scan_Navi()) //조이스틱 값을 스캔
		{
			case (0x1E0) : 	//LEFT
                          g_time--;     //Goal Time -1
                          if(g_time < 0)
                            g_time = 99;        //00~99 반복
 			break;
                        
                        case (0x2E0) : 	//RIGHT
                          g_time++;     //Goal Time +1
                          if(g_time > 99)       //00~99 반복
                            g_time = 0;
 			break;
                        
                  }
            sprintf(time, "%2d", g_time); // Goal Time값 문자열로 변환하여 LCD에 display
            LCD_SetTextColor(RGB_BLUE);                                           
            LCD_DisplayText(3,13, time);
            
            if(g_spec != 0) // Goal pulse = Goal degree/spec degree에서, 분모가 0이 아닐때만 연산되게 한다 (오류 방지)
            {
              p = g_degree/g_spec;
              sprintf(pulse, "%3d", p); //Goal pulse값 문자열로 변환하여 LCD에 display
              LCD_SetTextColor(RGB_RED);
              LCD_DisplayText(4,18, pulse);
            }
            
            if(g_time != 0) // Goal speed = Goal pulse/Goal time에서, 분모가 0이 아닐때만 연산되게 한다 (오류 방지)
            {
              t = p/g_time;
              sprintf(speed, "%3d", t);
              LCD_SetTextColor(RGB_RED);
              LCD_DisplayText(5,18, speed); //Goal speed값 문자열로 변환하여 LCD에 display
              TIM2->ARR	= (1000/t)*10/2 -1; // TIM2가 목표 속도로 펄스를 내보내도록 ARR값을 조절한다.
              //PSC 설정을 한 뒤 인터럽트 주기가 0.1ms이다. t값은 sec단위의 주기를 역수로 한 주파수 값이다.이를 다시 주기로 설정하기 위해 역수를 취한뒤 sec를 ms로 변환하기 위해 1000을 곱한다. 
              //이때 기존의 인터럽트 주기가 0.1ms이므로 10을 더 곱한다. 그리고 원하는 펄스 주기값 = 인터럽트 주기값*2 이므로 원하는 펄스 주기값/2 = 인터럽트 주기값 으로 해준다. 마지막으로 up count이므로 -1을 해준다.
              TIM2->CCR4 = (TIM2->ARR+1)/2; //duty비를 50%로 만들기 위해 CCR을 ARR값+1 의 절반만큼 설정해준다.
            }
          }

 	
}

void USART1_IRQHandler(void) //수신 인터럽트를 사용하기 위해 USART 핸들러 사용
{       
	if ( (USART1->SR & USART_SR_RXNE) ) //수신 레지스터가 채워지면 인터럽트가 발생한다.
	{
          char ch;
          ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
          ch_buffer[ch_count++] = ch; //문자를 수신할때마다 버퍼에 쌓는다.
          if(ch_count == 2) //버퍼가 세칸 쌓이면(commaster에서 두자리 숫자(한자리이면 01,02 이런식으로 보내야 한다) + null문자가 들어오면)
          {
            ch_count = 0; //버퍼를 다시 첫번째 칸으로 옮겨서 문자를 받아온다
            Serial_PutString(ch_buffer); //commaster에 에코

            LCD_SetTextColor(RGB_BLUE); //받아온 값을 LCD에 출력한다.
            LCD_DisplayText(1,14, ch_buffer);
            
            g_spec = atoi(ch_buffer); //버퍼에 저장된 ASCII코드 문자열 값을 정수형으로 변환하여 저장한다.(spec값)
          }
	}
        // DR 을 읽으면 SR.RXNE bit(flag bit)는 자동으로 clear 된다.
  
}


void TIMER2_Init(void) //TIM2 설정
{
        //TIMER2 SET (PB11)
       	RCC->AHB1ENR |= 0x02; //ENABLE GPIOB
       	GPIOB->MODER 	|= ( 2 << 22 );	// GPIOB PIN11 Output ★Alternate function mode★					
	GPIOB->OSPEEDR 	|= ( 3 << 22 );	// GPIOB PIN11 Output speed (100MHz High speed)
	GPIOB->OTYPER	= 0x00000000;	// GPIOB PIN11 Output type push-pull (reset state)
	GPIOB->PUPDR	        |= ( 1 << 22 );	// GPIOB PIN11 Pull-up
	GPIOB->AFR[1]	        |= ( 1 << 12 );	// (매뉴얼에서 봐야함, AFR[1] 의 AF1 ( TIM2 )
 
        
       	RCC->APB1ENR |= 0x01;// RCC_APB1ENR TIMER2 Enable
        NVIC->ISER[0] |= ( 1 << 28 ); // Enable Timer2 global Interrupt
        
        TIM2->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM2->ARR	= 5000-1;	// Auto reload  (0.1ms * 5000 = 500ms : Period) //500ms마다 트리거, ADC가 동작
       

        TIM2->CCER	|= (1<<12);	// CC4E: OC4 Active(Capture/Compare 4 output enable)
        TIM2->CCER	&= ~(1<<13);	// CC4P: Capture/Compare 4 output Polarity High
 
        TIM2->CCR4	= 2500;		// CCR4 value (임의의 초기값)
        
        TIM2->CCMR2 &= ~(3<<8); // CC4S(CC4 channel): Output 
        TIM2->CCMR2 &= ~(1<<11); // OC4PE: Output Compare 4preload disable
        TIM2->CCMR2 |= (3<<12);	// OC4M: Output Compare 4 Mode : toggle

	TIM2->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM2->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM2->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	
       	TIM2->DIER &= ~(1<<0);	//enable the Tim2 Update interrupt
       	TIM2->EGR  &= ~(1<<0);	//Update generation 
       	TIM2->CR1  &= ~(1<<0);	// Counter TIM2 enable
}

void TIM2_IRQHandler(void) //TIM2 핸들러
{
  if((TIM2->SR & 0x01) != RESET)
  {
    TIM2->SR &= ~0x01; //TIM2 인터럽트가 발생하면, 플래그를 클리어 해준다.
  }
   if((TIM2->SR & 0x02) != RESET)
  {
    TIM2->SR &= ~0x02; //TIM2 CC 인터럽트가 발생하면, 플래그를 클리어 해준다.
    GPIOG->ODR |= (1<<4); //LED4 ON

    p2_count++; //인터럽트가 발생할때 마다 증가 (인터럽트 카운트값)
    
    if(p2_count % 2 == 0) //카운트 값이 짝수일때 ( 인터럽트가 두번 발생할때 마다 = 펄스가 한번 발생할때 마다 )
    { 
      p_count++; //펄스 카운트 값이 증가
      sprintf(c_pulse,"%3d", p_count); //현재 펄스 수를 LCD에 출력
      LCD_SetTextColor(RGB_RED);
      LCD_DisplayText(6,14,c_pulse);
      
      if( p == p_count ) //목표 펄스수와 현재 펄스수가 같아지면
      {
        GPIOG->ODR &= ~(1<<4); //LED4 OFF 
        p_count = 0; //현재 펄스수를 0으로 초기화
        
        TIM2->DIER &= ~(1<<0);	//disable the Tim2 Update interrupt
        TIM2->EGR  &= ~(1<<0);	//Update generation X 
        TIM2->CR1  &= ~(1<<0);	// Counter TIM2 disable
      }
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

    USART_BRR_Configuration(9600); // USART Baud rate Configuration

    //***패리티 사용 안할때***
    USART1->CR1	&= ~USART_CR1_M;	// USART_WordLength 8 Data bit = (1 << 12)
    USART1->CR1	&= ~USART_CR1_PCE ;	// USART_Parity_No

    USART1->CR1	|= USART_CR1_RE;	// 0x0004, USART_Mode_RX Enable
    USART1->CR1	|= USART_CR1_TE ;	// 0x0008, USART_Mode_Tx Enable
    USART1->CR2	&= ~USART_CR2_STOP;	// USART_StopBits_1
    USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA

    USART1->CR1 	|= USART_CR1_RXNEIE;	//  0x0020, RXNE interrupt Enable

    NVIC->ISER[1]	|= (1 << 5); 	// Enable Interrupt USART1 (NVIC 37번) (37 - 32)
    
    USART1->CR1 	|= USART_CR1_UE;	//  0x2000, UART5 Enable
	
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

         //NAVI.SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
	
         
}

void _EXTI_Init(void)    //EXTI SW
{
    	RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	//Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	

	SYSCFG->EXTICR[2] |= 0x7777;          //EXTI 11,10,9,8에 대한 소스 입력은 GPIOH로 설정 (EXTICR3)
        SYSCFG->EXTICR[3] |= 0x7777;          //EXTI 12,13,14,15에 대한 소스 입력은 GPIOH로 설정 (EXTICR4)
        
	 EXTI->FTSR     |= 0xFF00;		// Falling Trigger Enable 

    	EXTI->IMR       |= 0xFF00;  	// 인터럽트 mask (Interrupt Enable)

        NVIC->ISER[0] |= ( 1 << 23 ); // Enable Interrupt EXTI9~5 Vector table Position 참조
        NVIC->ISER[1] |= ( 1 << 8 ); // Enable Interrupt EXTI15~10 Vector table Position 참조
}

void EXTI15_10_IRQHandler(void) //EXTI10~15 인터럽트 핸들러
{
        
         if(EXTI->PR & 0x0800) //SW3 EXTI 
        {
              EXTI->PR |= 0x0800; //clear flag
        }
        
         if(EXTI->PR & 0x1000) //SW4 EXTI 
        {
              EXTI->PR |= 0x1000; //clear flag
              if(p > 0) //목표 펄스수가 1개 이상이면 펄스 발생 시작
              {
                GPIOG->ODR |= (1 << 4); //LED4 ON
                sprintf(c_pulse,"%3d", p_count); //현재 펄스수 (초기화 된 0개 값) 를 LCD에 출력
                LCD_SetTextColor(RGB_RED);
                LCD_DisplayText(6,14,c_pulse);
                TIM2->DIER |= (1<<0);	//enable the Tim2 Update interrupt
                TIM2->EGR  |= (1<<0);	//Update generation X
                TIM2->CR1  |= (1<<0);	// Counter TIM2 enable
              }
        }
        
        if(EXTI->PR & 0x2000) //SW5 EXTI : Goal degree의 백의자리값 증감
        {
              EXTI->PR |= 0x2000; //clear flag
              
              if(g_degree/100 == 9) //Goal degree 백의자리가 9일때 스위치가 눌렸다면
                g_degree -= 900; //900을 감소시켜 백의자리를 0으로 만든다
              else
                g_degree += 100; //그 외에는 백의자리를 1씩 증가 (+100)
              
              sprintf(degree, "%3d", g_degree); //Goal degree값을 lcd에 display
              LCD_SetTextColor(RGB_BLUE);
              LCD_DisplayText(2,15, degree);
        }
        
        if(EXTI->PR & 0x4000) //SW6 EXTI : Goal degree의 십의자리값 증가
        {
              EXTI->PR |= 0x4000; //clear flag
              
              if((g_degree%100)/10 == 9) //Goal degree의 십의자리가 9일때 스위치가 눌렸다면
                g_degree -= 90; //90을 감소시켜 십의자리를 0으로 만든다
              else
                g_degree += 10; //그 외에는 십의자리를 1씩 증가(+10)
              
              sprintf(degree, "%3d", g_degree); //Goal degree값을 lcd에 display
              LCD_SetTextColor(RGB_BLUE);
              LCD_DisplayText(2,15, degree);

        }
        if(EXTI->PR & 0x8000) //SW7 EXTI  : Goal degree의 일의자리값 증가
        {
              EXTI->PR |= 0x8000; //clear flag
              
              if(((g_degree%100)%10) == 9) //Goal degree의 일의자리가 9일때 스위치가 눌렸다면
                g_degree -= 9; //9를 감소시켜 일의자리를 0으로 만든다
              else
                g_degree += 1; //그 외에는 일의자리를 1씩 증가 (+1)
              
              sprintf(degree, "%3d", g_degree); //Goal degree값을 lcd에 display
              LCD_SetTextColor(RGB_BLUE);
              LCD_DisplayText(2,15, degree);
        }
}

void EXTI9_5_IRQHandler(void) //EXTI5~9 인터럽트 핸들러
{
         if(EXTI->PR & 0x0100) //SW0 EXTI 
        {
              EXTI->PR |= 0x0100; //clear flag
              
    
        }
        
         if(EXTI->PR & 0x0200) //SW1 EXTI 
        {
              EXTI->PR |= 0x0200; //clear flag
              
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

void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DisplayTitle(void) //초기 화면 셋팅
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim7);		//폰트
        
	LCD_SetBackColor(RGB_GREEN);	//글자배경색
        LCD_SetTextColor(RGB_BLACK);
        LCD_DisplayText(0,0,"Step Motor Pulse Gen. ");
        
        LCD_SetBackColor(RGB_WHITE);
        LCD_DisplayText(1,0, "Motor spec(D):  (deg/p)");
        LCD_DisplayText(2,0, "Goal Degree(X):   (deg)"); 
        LCD_DisplayText(3,0, "Goal Time(T):  (sec)"); 
        LCD_DisplayText(4,0, "Goal Pulse(P=X/D):   (p)");
        LCD_DisplayText(5,0, "Goal Speed(V=P/T):   (p/s)");
        
        LCD_SetTextColor(RGB_BLUE);
        LCD_DisplayText(1,14, "00");
        LCD_DisplayText(2,15, "000");
        LCD_DisplayText(3,13, "00");
        
        LCD_SetTextColor(RGB_RED);
        LCD_DisplayText(6,0, "Current Pulse:000(p)"); 
        LCD_DisplayText(4,18, "000");
        LCD_DisplayText(5,18, "000");
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
		{	//DelayMS(10);
        		key_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
        		return 0xFF00;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			//DelayMS(10);
 			return key;
        	}
	}
}

uint8_t key_flag_ = 0;
uint16_t KEY_Scan_Navi(void)	// input navi
{ 
	uint16_t key;
	key = GPIOI->IDR & 0x3E0;	// any key pressed ?
	if(key == 0x3E0)		// if no key, check key off
	{  	if(key_flag_ == 0)
        		return key;
      		else
		{	//DelayMS(10);
        		key_flag_ = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	
          if(key_flag_ != 0)	// if continuous key, treat as no key input
        		return 0x3E0;
      		else			// if new key,delay for debounce
		{	key_flag_ = 1;
			//DelayMS(10);
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

