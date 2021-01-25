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
char ch_buffer[3]; //���ڿ��� �޾� ���� ���� ����
uint8_t ch_count = 0; //���ڿ��� �޾� ���� ���� ī��Ʈ ����



int main(void)
{
	_RCC_Init();
	LCD_Init();		// LCD ���� �Լ�
	DelayMS(10);		// LCD���� ������
    
	DisplayTitle();		//LCD �ʱ�ȭ�鱸�� �Լ�
	_GPIO_Init();
        _EXTI_Init();
        USART1_Init();

        TIMER2_Init();            //GPIO, EXTI, TIMER, USART ���� �Լ�
    
	while(1)
	{

            
            switch(KEY_Scan_Navi()) //���̽�ƽ ���� ��ĵ
		{
			case (0x1E0) : 	//LEFT
                          g_time--;     //Goal Time -1
                          if(g_time < 0)
                            g_time = 99;        //00~99 �ݺ�
 			break;
                        
                        case (0x2E0) : 	//RIGHT
                          g_time++;     //Goal Time +1
                          if(g_time > 99)       //00~99 �ݺ�
                            g_time = 0;
 			break;
                        
                  }
            sprintf(time, "%2d", g_time); // Goal Time�� ���ڿ��� ��ȯ�Ͽ� LCD�� display
            LCD_SetTextColor(RGB_BLUE);                                           
            LCD_DisplayText(3,13, time);
            
            if(g_spec != 0) // Goal pulse = Goal degree/spec degree����, �и� 0�� �ƴҶ��� ����ǰ� �Ѵ� (���� ����)
            {
              p = g_degree/g_spec;
              sprintf(pulse, "%3d", p); //Goal pulse�� ���ڿ��� ��ȯ�Ͽ� LCD�� display
              LCD_SetTextColor(RGB_RED);
              LCD_DisplayText(4,18, pulse);
            }
            
            if(g_time != 0) // Goal speed = Goal pulse/Goal time����, �и� 0�� �ƴҶ��� ����ǰ� �Ѵ� (���� ����)
            {
              t = p/g_time;
              sprintf(speed, "%3d", t);
              LCD_SetTextColor(RGB_RED);
              LCD_DisplayText(5,18, speed); //Goal speed�� ���ڿ��� ��ȯ�Ͽ� LCD�� display
              TIM2->ARR	= (1000/t)*10/2 -1; // TIM2�� ��ǥ �ӵ��� �޽��� ���������� ARR���� �����Ѵ�.
              //PSC ������ �� �� ���ͷ�Ʈ �ֱⰡ 0.1ms�̴�. t���� sec������ �ֱ⸦ ������ �� ���ļ� ���̴�.�̸� �ٽ� �ֱ�� �����ϱ� ���� ������ ���ѵ� sec�� ms�� ��ȯ�ϱ� ���� 1000�� ���Ѵ�. 
              //�̶� ������ ���ͷ�Ʈ �ֱⰡ 0.1ms�̹Ƿ� 10�� �� ���Ѵ�. �׸��� ���ϴ� �޽� �ֱⰪ = ���ͷ�Ʈ �ֱⰪ*2 �̹Ƿ� ���ϴ� �޽� �ֱⰪ/2 = ���ͷ�Ʈ �ֱⰪ ���� ���ش�. ���������� up count�̹Ƿ� -1�� ���ش�.
              TIM2->CCR4 = (TIM2->ARR+1)/2; //duty�� 50%�� ����� ���� CCR�� ARR��+1 �� ���ݸ�ŭ �������ش�.
            }
          }

 	
}

void USART1_IRQHandler(void) //���� ���ͷ�Ʈ�� ����ϱ� ���� USART �ڵ鷯 ���
{       
	if ( (USART1->SR & USART_SR_RXNE) ) //���� �������Ͱ� ä������ ���ͷ�Ʈ�� �߻��Ѵ�.
	{
          char ch;
          ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
          ch_buffer[ch_count++] = ch; //���ڸ� �����Ҷ����� ���ۿ� �״´�.
          if(ch_count == 2) //���۰� ��ĭ ���̸�(commaster���� ���ڸ� ����(���ڸ��̸� 01,02 �̷������� ������ �Ѵ�) + null���ڰ� ������)
          {
            ch_count = 0; //���۸� �ٽ� ù��° ĭ���� �Űܼ� ���ڸ� �޾ƿ´�
            Serial_PutString(ch_buffer); //commaster�� ����

            LCD_SetTextColor(RGB_BLUE); //�޾ƿ� ���� LCD�� ����Ѵ�.
            LCD_DisplayText(1,14, ch_buffer);
            
            g_spec = atoi(ch_buffer); //���ۿ� ����� ASCII�ڵ� ���ڿ� ���� ���������� ��ȯ�Ͽ� �����Ѵ�.(spec��)
          }
	}
        // DR �� ������ SR.RXNE bit(flag bit)�� �ڵ����� clear �ȴ�.
  
}


void TIMER2_Init(void) //TIM2 ����
{
        //TIMER2 SET (PB11)
       	RCC->AHB1ENR |= 0x02; //ENABLE GPIOB
       	GPIOB->MODER 	|= ( 2 << 22 );	// GPIOB PIN11 Output ��Alternate function mode��					
	GPIOB->OSPEEDR 	|= ( 3 << 22 );	// GPIOB PIN11 Output speed (100MHz High speed)
	GPIOB->OTYPER	= 0x00000000;	// GPIOB PIN11 Output type push-pull (reset state)
	GPIOB->PUPDR	        |= ( 1 << 22 );	// GPIOB PIN11 Pull-up
	GPIOB->AFR[1]	        |= ( 1 << 12 );	// (�Ŵ��󿡼� ������, AFR[1] �� AF1 ( TIM2 )
 
        
       	RCC->APB1ENR |= 0x01;// RCC_APB1ENR TIMER2 Enable
        NVIC->ISER[0] |= ( 1 << 28 ); // Enable Timer2 global Interrupt
        
        TIM2->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM2->ARR	= 5000-1;	// Auto reload  (0.1ms * 5000 = 500ms : Period) //500ms���� Ʈ����, ADC�� ����
       

        TIM2->CCER	|= (1<<12);	// CC4E: OC4 Active(Capture/Compare 4 output enable)
        TIM2->CCER	&= ~(1<<13);	// CC4P: Capture/Compare 4 output Polarity High
 
        TIM2->CCR4	= 2500;		// CCR4 value (������ �ʱⰪ)
        
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

void TIM2_IRQHandler(void) //TIM2 �ڵ鷯
{
  if((TIM2->SR & 0x01) != RESET)
  {
    TIM2->SR &= ~0x01; //TIM2 ���ͷ�Ʈ�� �߻��ϸ�, �÷��׸� Ŭ���� ���ش�.
  }
   if((TIM2->SR & 0x02) != RESET)
  {
    TIM2->SR &= ~0x02; //TIM2 CC ���ͷ�Ʈ�� �߻��ϸ�, �÷��׸� Ŭ���� ���ش�.
    GPIOG->ODR |= (1<<4); //LED4 ON

    p2_count++; //���ͷ�Ʈ�� �߻��Ҷ� ���� ���� (���ͷ�Ʈ ī��Ʈ��)
    
    if(p2_count % 2 == 0) //ī��Ʈ ���� ¦���϶� ( ���ͷ�Ʈ�� �ι� �߻��Ҷ� ���� = �޽��� �ѹ� �߻��Ҷ� ���� )
    { 
      p_count++; //�޽� ī��Ʈ ���� ����
      sprintf(c_pulse,"%3d", p_count); //���� �޽� ���� LCD�� ���
      LCD_SetTextColor(RGB_RED);
      LCD_DisplayText(6,14,c_pulse);
      
      if( p == p_count ) //��ǥ �޽����� ���� �޽����� ��������
      {
        GPIOG->ODR &= ~(1<<4); //LED4 OFF 
        p_count = 0; //���� �޽����� 0���� �ʱ�ȭ
        
        TIM2->DIER &= ~(1<<0);	//disable the Tim2 Update interrupt
        TIM2->EGR  &= ~(1<<0);	//Update generation X 
        TIM2->CR1  &= ~(1<<0);	// Counter TIM2 disable
      }
    }
  }
}

void USART1_Init(void) //USART ����
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

    //***�и�Ƽ ��� ���Ҷ�***
    USART1->CR1	&= ~USART_CR1_M;	// USART_WordLength 8 Data bit = (1 << 12)
    USART1->CR1	&= ~USART_CR1_PCE ;	// USART_Parity_No

    USART1->CR1	|= USART_CR1_RE;	// 0x0004, USART_Mode_RX Enable
    USART1->CR1	|= USART_CR1_TE ;	// 0x0008, USART_Mode_Tx Enable
    USART1->CR2	&= ~USART_CR2_STOP;	// USART_StopBits_1
    USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA

    USART1->CR1 	|= USART_CR1_RXNEIE;	//  0x0020, RXNE interrupt Enable

    NVIC->ISER[1]	|= (1 << 5); 	// Enable Interrupt USART1 (NVIC 37��) (37 - 32)
    
    USART1->CR1 	|= USART_CR1_UE;	//  0x2000, UART5 Enable
	
}

void _GPIO_Init(void)
{
	// LED GPIO(PORT G) ����
    	RCC->AHB1ENR    |= 0x00000040; 	// RCC_AHB1ENR(bit8~0) GPIOG(bit#6) Enable							
	GPIOG->MODER    = 0x00005555;	// GPIOG PIN0~PIN7 Output mode (0b01)						
	GPIOG->OTYPER   = 0x0000;	// GPIOG PIN0~PIN7 : Push-pull  (PIN8~PIN15) (reset state)	
 	GPIOG->OSPEEDR  = 0x00005555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 
    
	// SW GPIO(PORT H) ���� 
	 RCC->AHB1ENR    |= 0x00000080;	// RCC_AHB1ENR(bit8~0) GPIOH(bit#7) Enable							
	 GPIOH->MODER    = 0x00000000;	// GPIOH PIN8~PIN15 Input mode (reset state)				
	 GPIOH->PUPDR    = 0x00000000;	// GPIOH PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)

         //NAVI.SW(PORT I) ����
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
	
         
}

void _EXTI_Init(void)    //EXTI SW
{
    	RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	//Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	

	SYSCFG->EXTICR[2] |= 0x7777;          //EXTI 11,10,9,8�� ���� �ҽ� �Է��� GPIOH�� ���� (EXTICR3)
        SYSCFG->EXTICR[3] |= 0x7777;          //EXTI 12,13,14,15�� ���� �ҽ� �Է��� GPIOH�� ���� (EXTICR4)
        
	 EXTI->FTSR     |= 0xFF00;		// Falling Trigger Enable 

    	EXTI->IMR       |= 0xFF00;  	// ���ͷ�Ʈ mask (Interrupt Enable)

        NVIC->ISER[0] |= ( 1 << 23 ); // Enable Interrupt EXTI9~5 Vector table Position ����
        NVIC->ISER[1] |= ( 1 << 8 ); // Enable Interrupt EXTI15~10 Vector table Position ����
}

void EXTI15_10_IRQHandler(void) //EXTI10~15 ���ͷ�Ʈ �ڵ鷯
{
        
         if(EXTI->PR & 0x0800) //SW3 EXTI 
        {
              EXTI->PR |= 0x0800; //clear flag
        }
        
         if(EXTI->PR & 0x1000) //SW4 EXTI 
        {
              EXTI->PR |= 0x1000; //clear flag
              if(p > 0) //��ǥ �޽����� 1�� �̻��̸� �޽� �߻� ����
              {
                GPIOG->ODR |= (1 << 4); //LED4 ON
                sprintf(c_pulse,"%3d", p_count); //���� �޽��� (�ʱ�ȭ �� 0�� ��) �� LCD�� ���
                LCD_SetTextColor(RGB_RED);
                LCD_DisplayText(6,14,c_pulse);
                TIM2->DIER |= (1<<0);	//enable the Tim2 Update interrupt
                TIM2->EGR  |= (1<<0);	//Update generation X
                TIM2->CR1  |= (1<<0);	// Counter TIM2 enable
              }
        }
        
        if(EXTI->PR & 0x2000) //SW5 EXTI : Goal degree�� �����ڸ��� ����
        {
              EXTI->PR |= 0x2000; //clear flag
              
              if(g_degree/100 == 9) //Goal degree �����ڸ��� 9�϶� ����ġ�� ���ȴٸ�
                g_degree -= 900; //900�� ���ҽ��� �����ڸ��� 0���� �����
              else
                g_degree += 100; //�� �ܿ��� �����ڸ��� 1�� ���� (+100)
              
              sprintf(degree, "%3d", g_degree); //Goal degree���� lcd�� display
              LCD_SetTextColor(RGB_BLUE);
              LCD_DisplayText(2,15, degree);
        }
        
        if(EXTI->PR & 0x4000) //SW6 EXTI : Goal degree�� �����ڸ��� ����
        {
              EXTI->PR |= 0x4000; //clear flag
              
              if((g_degree%100)/10 == 9) //Goal degree�� �����ڸ��� 9�϶� ����ġ�� ���ȴٸ�
                g_degree -= 90; //90�� ���ҽ��� �����ڸ��� 0���� �����
              else
                g_degree += 10; //�� �ܿ��� �����ڸ��� 1�� ����(+10)
              
              sprintf(degree, "%3d", g_degree); //Goal degree���� lcd�� display
              LCD_SetTextColor(RGB_BLUE);
              LCD_DisplayText(2,15, degree);

        }
        if(EXTI->PR & 0x8000) //SW7 EXTI  : Goal degree�� �����ڸ��� ����
        {
              EXTI->PR |= 0x8000; //clear flag
              
              if(((g_degree%100)%10) == 9) //Goal degree�� �����ڸ��� 9�϶� ����ġ�� ���ȴٸ�
                g_degree -= 9; //9�� ���ҽ��� �����ڸ��� 0���� �����
              else
                g_degree += 1; //�� �ܿ��� �����ڸ��� 1�� ���� (+1)
              
              sprintf(degree, "%3d", g_degree); //Goal degree���� lcd�� display
              LCD_SetTextColor(RGB_BLUE);
              LCD_DisplayText(2,15, degree);
        }
}

void EXTI9_5_IRQHandler(void) //EXTI5~9 ���ͷ�Ʈ �ڵ鷯
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



void SerialPutChar(uint8_t Ch) // 1���� ������ �Լ�
{
        while((USART1->SR & USART_SR_TXE) == RESET); //  USART_SR_TXE:0x0080, �۽� ������ ���±��� ��� SR���������� TXE �÷��װ� 1�̸� ����ְ� 0�̸� ���ִ�!!!!!!!!!!!!!!!!!!!!!!!!!!!! TRUE�� ��� while���� ����. FALSE�� �Ǻ����� ��������ϱ� ���۽��Ѵ�

	USART1->DR = (Ch & 0x01FF);	// ���� (������ ���ͷ�Ʈ ����)
}

void Serial_PutString(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialPutChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

// Baud rate  
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency USART 2345�϶��� 42000000�� �ٲ��־���Ѵ�.
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

void DisplayTitle(void) //�ʱ� ȭ�� ����
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim7);		//��Ʈ
        
	LCD_SetBackColor(RGB_GREEN);	//���ڹ���
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

    // Enable HSE : �ܺ� �Է� Ŭ�����ļ�: 5MHz(ȸ�ε� ����)
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
	// SYSCLK ��� (HSE �Է�Ŭ�����ļ�: 8MHz)
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

