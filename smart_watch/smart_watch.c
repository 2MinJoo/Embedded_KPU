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

void _ADC_Init(void);
void TIMER3_Init(void);
void TIMER4_Init(void);
void TIMER7_Init(void);
void TIMER9_Init(void);
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

int clk_h = 15, clk_m = 10, ala_h = 0, ala_m = 0, ala_h_ = 0, ala_m_ = 0; // MODE 1 ����
char clkh[2], clkm[2], ala[4];

int sw_s = 0, sw_ms = 0, rec_s = 0, rec_ms = 0; //MODE 2 ����
char sw[6], rec[6];

int cal_1, cal_2, cal_r; //MODE 3 ����
char cal[11];

int temp, heat, cool; //MODE 4 ����
char temp_c[6], temp_s[8];
unsigned short adc; 

unsigned short MODE = 1, ala_flag = 0, cal_flag = 0; //�÷��� ������


int main(void)
{
	_RCC_Init();
	LCD_Init();		// LCD ���� �Լ�
	DelayMS(10);		// LCD���� ������
    
	DisplayTitle();		//LCD �ʱ�ȭ�鱸�� �Լ�
	_GPIO_Init();
        _EXTI_Init();
	_ADC_Init();
        USART1_Init();
        TIMER9_Init();
        TIMER7_Init(); 
        TIMER4_Init();
        TIMER3_Init();            //GPIO, EXTI, ADC, TIMER, USART ���� �Լ�
    
	while(1)
	{
          if(MODE == 1) //MODE 1�϶��� ����
          { 
            
            switch(KEY_Scan_Navi()) //���̽�ƽ ���� ��ĵ
		{
			case (0x1E0) : 	//LEFT
				 --ala_m; //�� ����
                                 if(ala_m < 0) //0~15 (16����)
                                  ala_m = 15;
                                 sprintf(ala,"%X",ala_m); //���ڿ� ����� LCD�� ���
                                 LCD_SetTextColor(RGB_RED);
                                 LCD_DisplayText(2, 10, ala);
 			break;
                        
                        case (0x2E0) : 	//RIGHT
				 ++ala_m; //�� ����
                                if(ala_m > 15) //0~15 (16����)
                                  ala_m = 0;
                                sprintf(ala,"%X",ala_m); //���ڿ� ����� LCD�� ���
                                LCD_SetTextColor(RGB_RED);
                                LCD_DisplayText(2, 10, ala);
 			break;
                        
                        case (0x360) : 	//DOWN
				 --ala_h; //�� ����
                                if(ala_h < 0) //0~15 (16����)
                                  ala_h = 15;
                                sprintf(ala,"%X",ala_h); //���ڿ� ����� LCD�� ���
                                LCD_SetTextColor(RGB_RED);
                                LCD_DisplayText(2, 8, ala);
 			break;
                        
                        case (0x3A0) : 	//UP
				 ++ala_h; //�� ����
                                if(ala_h > 15) //0~15 (16����)
                                  ala_h = 0;
                                sprintf(ala,"%X",ala_h); //���ڿ� ����� LCD�� ���
                                LCD_SetTextColor(RGB_RED);
                                LCD_DisplayText(2, 8, ala);
 			break;
                        
                        case (0x3C0) : 	//PUSH
                                BEEP();
                                LCD_SetPenColor(RGB_YELLOW); //���� �˶� �ð��� �����(��������� ����)
                                Set_Clock(ala_h_, ala_m_);
				ala_flag = 1; //�˶� �÷��׸� 1�� �������
                                ala_h_ = ala_h; //�ٲ� �˶� �ð����� ������Ʈ
                                ala_m_ = ala_m;
                                LCD_SetPenColor(RGB_RED); //������Ʈ�� �˶� �ð����� �Ƴ��α� �ð迡 display
                                Set_Clock(ala_h_, ala_m_);
 			break;
                  }
            
          }

 	}
}

void ADC_IRQHandler(void) //ADC �ڵ鷯
{
  if(ADC1->SR & 0x02)
  {
    ADC1->SR &= ~(1<<1); //ADC1�� 500ms���� AD ��ȯ�� �ϰ�, EOC �÷��װ� �߻��ϹǷ�, �÷��׸� �����ش�.
    adc = ADC1->DR;   // Read ADC1 result value from ADC1 Data Reg(ADC1->DR) 
    temp = (adc * 10) /819 - 10; //0~4095�� 10�� ���Ͽ� 0~40950���� ����� 819�� ������ 0~50�� �ȴ�. ���⿡ 10�� ���ָ� -10~40�� �ȴ�.
    sprintf(temp_c,"%3d ",temp); //����� ������ ���ڿ��� �����.
    LCD_SetTextColor(RGB_BLACK);	//���ڻ�
    LCD_DisplayText(2,3,temp_c); //LCD�� �µ��� ���
    
    LCD_SetBrushColor(RGB_WHITE); //�µ� �׷����� �׸��� ���� ������ �׷��� �µ� �׷��� Ŭ����
    LCD_DrawFillRect(60, 37, 70, 10);
    
    if(temp >= -10 && temp <= 0) //-10��~0�� : ���� 1�ܰ�(BLUE)
    {
      heat = 2; //���� ����2
      cool = 0; //�� ����0
      TIM4->CCR1 = 9000; // LV2 : DR 90%
      GPIOG->ODR &= ~(1 << 2); 
      GPIOG->ODR |= (1 << 1);   //���� => LED1�� ON
      LCD_SetBrushColor(RGB_BLUE);
    }
    else if( temp > 0 && temp <= 10) //1��~10�� : ���� 2�ܰ�(BLUE)
    {
      heat = 1; //���� ����1
      cool = 0; //�� ����0
      TIM4->CCR1 = 1000; // LV1 : DR 10%
      GPIOG->ODR &= ~(1 << 2);
      GPIOG->ODR |= (1 << 1);   //���� => LED1�� ON
      LCD_SetBrushColor(RGB_BLUE);
    }
    else if( temp > 10 && temp <= 20) //11��~20�� : ���� 3�ܰ�(GREEN)
    {
      heat = 0; //�Ѵ� ���� 0
      cool = 0;
      TIM4->CCR1 = 0;	// Heater / Cooler off
      GPIOG->ODR &= ~(3 << 1); //LED OFF
      LCD_SetBrushColor(RGB_GREEN);
    }
    else if( temp > 20 && temp <= 30) //21��~30�� : ���� 4�ܰ�(RED)
    {
      heat = 0;
      cool = 1; //�� ����1
      TIM4->CCR1 = 1000; // LV1 : DR 10%
      GPIOG->ODR &= ~(1 << 1);
      GPIOG->ODR |= (1 << 2); //�� => LED 2�� ON
      LCD_SetBrushColor(RGB_RED);
    }
    else if( temp > 30 && temp <= 40) //31��~40�� : ���� 5�ܰ�(RED)
    {
      heat = 0;
      cool = 2; //�� ���� 2
      TIM4->CCR1 = 9000; // LV2 : DR 90%
      GPIOG->ODR &= ~(1 << 1);
      GPIOG->ODR |= (1 << 2); //�� => LED 2�� ON
      LCD_SetBrushColor(RGB_RED);
    }
    
    LCD_SetTextColor(RGB_BLACK);//���ڻ�
    sprintf(temp_s,"H:%d C:%d",heat, cool);
    LCD_DisplayText(3,1,temp_s); //LCD�� display
    
    LCD_DrawFillRect(60, 37, temp+30, 10); //�µ��� ����ؼ� �׷��� ���
  }
}
  
void USART1_IRQHandler(void) //���� ���ͷ�Ʈ�� ����ϱ� ���� USART �ڵ鷯 ���
{       
	if ( (USART1->SR & USART_SR_RXNE) ) //���� �������Ͱ� ä������ ���ͷ�Ʈ�� �߻��Ѵ�.
	{
          char ch;
          ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
          
          switch(ch) //�ƽ�Ű�ڵ� ������ �����ϱ� ������, �ƽ�Ű�ڵ庰�� ���ڸ� ��������(16�����̹Ƿ� ���� ������Ѵ�)
          {
            case '0' :
              ch = 0x0;
              break;
            case '1' :
              ch = 0x1;
              break;
            case '2' :
              ch = 0x2;
              break;
            case '3' :
              ch = 0x3;
              break;
            case '4' :
              ch = 0x4;
              break;
            case '5' :
              ch = 0x5;
              break;
            case '6' :
              ch = 0x6;
              break;
            case '7' :
              ch = 0x7;
              break;
            case '8' :
              ch = 0x8;
              break;
            case '9' :
              ch = 0x9;
              break;
            case 'A' :
              ch = 0xA;
              break;
            case 'B' :
              ch = 0xB;
              break;
            case 'C' :
              ch = 0xC;
              break;
            case 'D' :
              ch = 0xD;
              break;
            case 'E' :
              ch = 0xE;
              break;
            case 'F' :
              ch = 0xF;
              break;
          }
            

          if(ch == '=') //=�� ���ŵǾ�����,
          {
            cal_flag = 2; //�÷��׸� 2�� ����� ( ����ϴ� ��� )
            cal_r = cal_1 + cal_2; //������� ���
            sprintf(cal,"%X + %X = %2X",cal_1, cal_2, cal_r); //���ڿ��� ����� LCD�� ���÷������ش�.
            LCD_SetTextColor(RGB_BLACK);
            LCD_DisplayText(2, 2, cal);
	BEEP();
            cal_flag = 0; //���� ���Ű��� ù��° operand
          }
          else //=�� �ƴҶ�
          {
            if(cal_flag == 0) //�÷��װ� 0�϶� = operand 1 
            {
              cal_1 = ch; //������ ���� ����1�� �����ѵ�, ù��° operand�� display
              sprintf(cal,"%X",cal_1);
              LCD_SetTextColor(RGB_BLACK);
              LCD_DisplayText(2, 2, cal);
	  BEEP();
              cal_flag = 1; //���� ���Ű��� �ι�° operand
            }
            else if(cal_flag == 1) //�÷��װ� 1�϶� = operand 2
            {
              cal_2 = ch; //������ ���� ����2�� ������ ��, �ι�° operand�� display
              sprintf(cal,"%X",cal_2);
              LCD_SetTextColor(RGB_BLACK);
              LCD_DisplayText(2, 6, cal);
	  BEEP();
              cal_flag = 0; //���� ���Ű��� ù��° operand
            }
          }
	} 
        // DR �� ������ SR.RXNE bit(flag bit)�� �ڵ����� clear �ȴ�.
  
}

void _ADC_Init(void) //ADC1 : PA1
{   
        //ADC1 (PA1)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// 0x00000001  // ENABLE GPIOA CLK
	GPIOA->MODER |= GPIO_MODER_MODER1;       // 0x0000000C	// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// 0x00000100  // ENABLE ADC1 CLK

        ADC->CCR &= ~0X0000001F;	// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= 0x00010000;		// ADCPRE: ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
	ADC->CCR &= ~0x0000C000;	// DMA: Disable adc ����� ����

        ADC1->CR1 &= ~(3<<24);		// RES[1:0]: 12bit Resolution ( res - resolution )
        ADC1->CR1 &= ~(1<<8);		// SCAN: ADC_ScanCovMode Disable
        ADC1->CR1 |= ( 1 << 5);            // EOC ���ͷ�Ʈ �߻� ���
        
        ADC1->CR2 |= (3<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(RISING FALLING �Ѵ� ����, ��� �ɶ����� ����)
	ADC1->CR2 |= (7 << 24);	        // EXTSEL[3:0]: ADC_ExternalTrig (TIM3_CC1)
        
        ADC1->CR2 &= ~(1<<1);		// CONT: ADC_ContinuousConvMode Disable
        ADC1->CR2 &= ~(1<<11);		// ALIGN: ADC_DataAlign_Right
        ADC1->CR2 |= (1<<10);		// EOCS: The EOC bit is set at the end of each regular conversion 250ms���� EOC ��Ʈ set.
        
        ADC1->SQR1 &= ~0x00F00000;	// L[3:0]: ADC Regular channel sequece length = 1 conversion
        
        ADC1->SMPR2	|= 0x07 << (3*1);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1) ���ø�Ÿ��. ũ�� �߿��Ѱ� �ƴ�
        
                //Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
        ADC1->SQR3 |= 0x01<<0;
        
        NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
        
        //ADC1 ON�� MODE4���� ����
       

}

void TIMER3_Init(void) //TIM3 ����
{
        //TIMER3 SET (PA6)
       	RCC->AHB1ENR |= 0x01; //ENABLE GPIOA
       	GPIOA->MODER 	|= ( 2 << 12 );	// GPIOA PIN6 Output ��Alternate function mode��					
	GPIOA->OSPEEDR 	|= ( 3 << 12 );	// GPIOA PIN6 Output speed (100MHz High speed)
	GPIOA->OTYPER	= 0x00000000;	// GPIOA PIN6 Output type push-pull (reset state)
	GPIOA->PUPDR	        |= ( 1 << 12 );	// GPIOA PIN6 Pull-up
  					// PA6 ==> TIM3_CH1
	GPIOA->AFR[0]	        |= ( 1 << 24 );	// (�Ŵ��󿡼� ������, AFR[0] �� AF2 ( TIM3 )
 
        
       	RCC->APB1ENR |= 0x02;// RCC_APB1ENR TIMER3 Enable
        NVIC->ISER[0] |= ( 1 << 29 ); // Enable Timer3 global Interrupt
        
        TIM3->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM3->ARR	= 5000-1;	// Auto reload  (0.1ms * 5000 = 500ms : Period) //500ms���� Ʈ����, ADC�� ����
        //falling edge������ �����ϵ��� ���������� �ֱ⸦ ������ �ٿ��� �ϴµ�, ��۽ÿ� �����ϵ��� ���������Ƿ� �ֱ⸦ 250ms�� �Ͽ��� �ȴ�.

        TIM3->CCER	|= (1<<0);	// CC1E: OC1 Active(Capture/Compare 1 output enable)
        TIM3->CCER	&= ~(1<<1);	// CC1P: Capture/Compare 1 output Polarity High
 
        TIM3->CCR1	= 10;		// CCR1 value (������ �ʱⰪ)
        
        TIM3->CCMR1 &= ~(3<<0); // CC1S(CC1 channel): Output 
        TIM3->CCMR1 &= ~(1<<3); // OC1PE: Output Compare 1preload disable
        TIM3->CCMR1 |= (3<<4);	// OC1M: Output Compare 1 Mode : toggle

	TIM3->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM3->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM3->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	
       	TIM3->DIER &= ~(1<<0);	//disable the Tim3 Update interrupt
       	TIM3->EGR &= ~(1<<0);	//Update generation X
       	TIM3->CR1  &= ~(1<<0);	// Counter TIM3 disable
}

void TIM3_IRQHandler(void) //TIM3 �ڵ鷯, 0.5�� �ֱ� �߻�
{
   if((TIM3->SR & 0x02) != RESET)
  {
    TIM3->SR &= ~0x02; //TIM3 CC ���ͷ�Ʈ�� �߻��ϸ�, �÷��׸� Ŭ���� ���ش�.
  }
}

void TIMER4_Init(void)
{
// PB6�� ��¼����ϰ� Alternate function(TIM4_CH1)���� ��� ����
	RCC->AHB1ENR	|= (1 << 1);	// RCC_AHB1ENR GPIOB Enable
	GPIOB->MODER 	|= (2 << 12);	// PB6 AF MODE					
	GPIOB->OSPEEDR 	|= (3 << 12);	// PB6 Output speed (100MHz High speed)
	GPIOB->OTYPER	= 0x00000000;	// PB6 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1 << 12);	// PB6 Pull-up
	GPIOB->AFR[0]	|= (2 << 24);	// (�Ŵ��󿡼� ������, AFR[0] = AFRL, AFR[1] = AFRH) ----AFRH : Connect TIM4 pins(PB6) to AF2(TIM3..5) <-�������� ����߿� �����ϴ� ��������
 
	// TIM4 Channel 1 : PWM 1 mode
	// Assign 'PWM Pulse Period'
        RCC->APB1ENR 	|= 0x00000004;
        
	TIM4->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM4->ARR	= 10000-1;	// Auto reload  (0.1ms * 10000 = 1s : PWM Period)

	// CR1 : Up counting
	TIM4->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM4->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
       	TIM4->CR1 	|=  (2<<5); 	// CMS(Center-aligned mode Sel) : Mode 2
    	
	// Define the corresponding pin by 'Output'  
	TIM4->CCER	|= (1<<0);	// CC1E: OC1 Active(Capture/Compare 1 output enable)
	TIM4->CCER	&= ~(1<<1);	// CC1P: Capture/Compare 1 output Polarity High  (���� ����)

	// Duty Ratio 
	TIM4->CCR1	= 0;		// CCR1 value (reset : DR�� 0)

	// 'Mode' Selection : Output mode, PWM 1
	TIM4->CCMR1 	&= ~(3<<0); 	// CC1S(CC1 channel): Output 
	TIM4->CCMR1 	&= ~(1<<3); 	// OC1PE: Output Compare 1 preload disable
	TIM4->CCMR1	|= (6<<4);	// OC1M: Output compare 1 mode: PWM 1 mode
	TIM4->CCMR1	|= (1<<7);	// OC1CE: Output compare 1 Clear enable
	
	//Counter TIM4 enable
	TIM4->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
        
        TIM4->DIER &= ~(1<<0);	//disable the Tim4 Update interrupt
        TIM4->EGR &= ~(1<<0);	//Update generationX
        TIM4->CR1  &= ~(1<<0); //counter TIM4 disable
}

void TIM4_IRQHandler(void)
{
  if((TIM4->SR & 0x01) != RESET)
  {
    TIM4->SR &= ~0x01; //TIM4 ���ͷ�Ʈ�� �߻��ϸ�, �÷��׸� Ŭ���� ���ش�.
  }
  
  if((TIM4->SR & 0x02) != RESET)
  {
    TIM4->SR &= ~0x02; //TIM4 CC ���ͷ�Ʈ�� �߻��ϸ�, �÷��׸� Ŭ���� ���ش�.
  }
  
}

void TIMER9_Init(void) //TIM9 ����
{
        //TIMER9 SET (PA2)
       	RCC->AHB1ENR |= 0x01; //ENABLE GPIOA
       	GPIOA->MODER 	|= 0x20;	// GPIOA PIN2 Output ��Alternate function mode��					
	GPIOA->OSPEEDR 	|= 0x30;	// GPIOA PIN2 Output speed (100MHz High speed)
	GPIOA->OTYPER	= 0x00000000;	// GPIOA PIN2 Output type push-pull (reset state)
	GPIOA->PUPDR	        |= 0x10;	// GPIOA PIN2 Pull-up
  					// PA2 ==> TIM9_CH1
	GPIOA->AFR[0]	        |= ( 3 << 8 );	// (�Ŵ��󿡼� ������, AFR[0] �� AF3 ( TIM9 )
 
        
       	RCC->APB2ENR |= (1 << 16);// RCC_APB2ENR TIMER9 Enable
        NVIC->ISER[0] |= ( 1 << 24 ); // Enable Timer9 global Interrupt
        
        TIM9->PSC	= 16800-1;	// Prescaler 168,000,000Hz/16800 = 10,000 Hz(0.1ms)  (1~65536)
	TIM9->ARR	= 1000-1;	// Auto reload  (0.1ms * 1000 = 100ms : Period) //100ms���� ���ͷ�Ʈ �߻�

        TIM9->CCER	|= (1<<0);	// CC1E: OC1 Active(Capture/Compare 1 output enable)
        TIM9->CCER	&= ~(1<<1);	// CC1P: Capture/Compare 1 output Polarity High
 
        TIM9->CCR1	= 10;		// CCR1 value (������ �ʱⰪ)
        
        TIM9->CCMR1 &= ~(3<<0); // CC1S(CC1 channel): Output 
        TIM9->CCMR1 &= ~(1<<3); // OC1PE: Output Compare 1preload disable
        TIM9->CCMR1 |= (3<<4);	// OC1M: Output Compare 1 Mode : toggle

	TIM9->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM9->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM9->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	
       	TIM9->DIER &= ~(1<<0);	//Enable the Tim9 Update interrupt
       	TIM9->EGR &= ~(1<<0);	//Update generation 
        TIM9->CR1 &=  ~(1<<0);	// CEN: Counter TIM9 enable
}

void TIM1_BRK_TIM9_IRQHandler(void) //TIM9 �ڵ鷯
{
  
  if((TIM9->SR & 0x02) != RESET)
  {
    TIM9->SR &= ~0x02; //TIM9 CC ���ͷ�Ʈ�� �߻��ϸ�, �÷��׸� Ŭ���� ���ش�.

    sw_ms++; //��ž��ġ�� 0.1s���� ���������ش�. 
    if(sw_ms > 15) //16������ ǥ��, 15���� Ŀ���� ���� ������ ����������
    {
      sw_s++;
      sw_ms = 0;
    }
    if(sw_s > 255) //���� ������ 16���� ���ڸ��� �̹Ƿ� 255(FF)���� ������ �� �ִ�.
      sw_s = 0;
    
    sprintf(sw,"%2X:%X ",sw_s, sw_ms); //���ڿ��� ����� lcd�� ����Ѵ�.
      
     LCD_SetTextColor(RGB_RED);	//���ڻ�
     LCD_DisplayText(2,5,sw);
    
    
  }
  
}

void TIMER7_Init(void)
{
//TIMER7 SET
       	RCC->APB1ENR |= 0x20; //ENABLE  TIM7 ( �⺻ Ÿ�̸Ӷ� GPIO ������ ���� �ʾƵ� �ȴ�.)
       
        NVIC->ISER[1] |= ( 1 << (55-32) ); // Enable Timer7 global Interrupt
        
        TIM7->PSC	= 4200-1;	// Prescaler 84,000,000Hz/4200 = 2000 Hz(0.5ms)  (1~65536)
	TIM7->ARR	= 20000-1;	// Auto reload  (0.5ms * 20000= 1s : Period) //1s���� ���ͷ�Ʈ �߻�!

	TIM7->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM7->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM7->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	
       	TIM7->DIER       |= (1<<0);	//Enable the Tim7 Update interrupt
       	TIM7->EGR       |=(1<<0);	//Update generation 
       	TIM7->CR1	|= (1<<0);	// CEN: Counter TIM7 enable
}

void TIM7_IRQHandler(void) //TIM7 �ڵ鷯, 1�� �ֱ� �߻�
{
  if((TIM7->SR & 0x01) != RESET)
  {
    TIM7->SR &= ~0x01; //TIM7 ���ͷ�Ʈ�� �߻��ϸ�, �÷��׸� Ŭ���� ���ش�.
    
    ++clk_m; //���� �ð��� minute ���� 1�ʸ��� ������Ų��.
    
    if(clk_m > 15) //16���� ǥ���̹Ƿ� 15���� Ŀ���� 0���� ����
    {
      ++clk_h; //hour�� ������Ų��.
      clk_m = 0;
    }
    if(clk_h > 15) //16���� ǥ���̹Ƿ� 15���� Ŀ���� 0���� ����
      clk_h = 0;
    
    sprintf(clkh,"%X",clk_h); //���ڿ��� ����� lcd�� ǥ��
    sprintf(clkm,":%X",clk_m);
      
    LCD_SetTextColor(RGB_BLACK);	//���ڻ�
    LCD_DisplayText(0,16,clkh); 
    LCD_SetTextColor(RGB_BLUE);	//���ڻ�
    LCD_DisplayText(0,17,clkm);

    if(MODE == 1) //mode 1�϶��� lcd�� �Ƴ��α� �ð踦 ǥ���Ѵ�.
    {
    
      LCD_SetBrushColor(RGB_YELLOW);
      LCD_DrawFillRect(80, 50, 60, 60);  
      
      LCD_SetPenColor(RGB_RED); //�˶��� ���������� ǥ��
      Set_Clock(ala_h_, ala_m_); //���� ������ �˶� �ð����� ǥ�õǵ��� �����Ͽ���.
      //(�ð��� �ٲܶ����� �ٷ� �׷����°� �ƴ϶� Navi push �ϸ� �׶��� ���� �ð����� �׸��� �׷�����.)
      
      LCD_SetPenColor(RGB_BLUE); //���� �ð��� �Ķ������� ǥ��
      Set_Clock(clk_h, clk_m);
    }
    
    if(ala_flag == 1) //�˶��ð��� ����(Navi push)�������� �÷��װ� 1�̵Ǹ�,
    {
        if((ala_h_ == clk_h) &&  (ala_m_ == clk_m)) //�� �ð��� ��ġ�ϸ� �˶��� �︰��.
      {
        BEEP();
        DelayMS(30);
        BEEP();
        DelayMS(30);
        BEEP(); //�Ҹ� ����
        ala_flag = 0; //�׸���� �÷��׸� 0���� ����� �ѹ��� ���� �ٽ� ���� �ð��� �Ǿ �Ҹ��� ���� �ʴ´�.
      } 
    }


  }
}

void Set_Clock(int h, int m) //�Ƴ��α� �ð踦 ���۽�Ű�� ���� �ۼ��� �Լ�.
{
  UINT16 x, y; //LCD��ǥ���� UINT16�̹Ƿ� ���� Ÿ���� ���� ����
  
  //360���� 16�ð����� �������� ��, �ѽð���  22.5���̴�.
  //0��, 4��, 8��, 12�� (���� 0��, 90��, 180��, 270���� �ش��ϴ� �ð�) ���� ������ ������ŭ display�ǰ� ���ش�
  // -> sin, cos���� UINT�� �ٲٸ� ��Ȯ�� �������� �ʾ� ������ �����, sin, cos�� ���ڰ� degree�� �ƴ� rad�̶� ��Ȯ�� ���� �Է��ϱ� ����� ������ �����. 
  // �ٸ� �ð����� �ִ��� ������ ������ �ʵ��� ������ �����Ͽ���, ������ �� �������� 90,180,270����ŭ�� �ð����� lcd�� �� �������� display�Ͽ���.
  if( h == 4 && m == 0 ) //4�� 0���϶�
  {
   x = 110+30*sin(1.571); // 4�� - 90�� : 90*����/180 , ���� �߽���ǥ�� 110�� 30sin(x) (x�� �귯�� �ð��� �ش��ϴ� ����)�� �����ش�.
   y = 80-30*cos(1.571); //LCD�� x,y��ǥ��� �Ϲ� ��ǥ��� �ٸ��� ���� ���� (0,0), x�� �������� ���� y�� �Ʒ����� �����̹Ƿ� y�� ��ȣ�� �ݴ����(-)���� ���ش�.
  }
  else if( h == 8 && m == 0) //8�� 0���϶�
  {
   x = 110+30*sin(3.14); //8�� - 180�� : 180*����/180.
   y = 80-30*cos(3.14);
  }
  else if( h == 12 && m == 0 ) //12�� 0���϶�
  {
   x = 110+30*sin(4.7); //12�� - 270�� :  270*����/180
   y = 80-30*cos(4.7);
  }
  else if( h == 0 && m == 0 ) //0�� 0���϶�
  {
   x = 110+30*sin(0); // 0�� - 0��
   y = 80-30*cos(0);
  }
  else
  {
     x = 110+30*sin(0.4*h + 0.021*m); //�׿��� �ð�����, 1�ð��� 22.5*����/180��, 1�п� (22.5/16)*����/180�� ��ŭ �����δ�.
     y = 80-30*cos(0.4*h + 0.021*m); 
  }
  
  LCD_DrawLine(110, 80, x, y); //����� ���� lcd�� ǥ��.
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
	
        // Buzzer GPIO(PORT F) ���� 
    	RCC->AHB1ENR    |= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
	GPIOF->MODER    |= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
	GPIOF->OTYPER   &= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
 	GPIOF->OSPEEDR  |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 
         
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
              if(MODE == 2)
              {
                //��ž��ġ start
                TIM9->DIER |= (1<<0);	//Enable the Tim9 Update interrupt
                TIM9->EGR |=(1<<0);	//Update generation 
                TIM9->CR1	|= (1<<0);	// CEN: Counter TIM9 enable
                BEEP();
               GPIOG->ODR &= ~(1 << 6); //��ž��ġ stop led(led 6)�� �����ִٸ� ����.
                GPIOG->ODR |= (1 << 4); //led4 on
              }
        }
        
        if(EXTI->PR & 0x2000) //SW5 EXTI 
        {
              EXTI->PR |= 0x2000; //clear flag
              if(MODE == 2)
              {
                //���� ��ž��ġ�� record
                rec_s = sw_s; //���� ��ž��ġ�� ����
                rec_ms = sw_ms;
                sprintf(rec,"%2X:%X ",rec_s, rec_ms); //���ڿ� ����� lcd�� ���
                LCD_SetTextColor(RGB_RED);	//���ڻ�
                LCD_DisplayText(3,8,rec);
                BEEP();
                GPIOG->ODR |= (1 << 5); //led5 on
              }
        }
        
        if(EXTI->PR & 0x4000) //SW6 EXTI 
        {
              EXTI->PR |= 0x4000; //clear flag
              if(MODE == 2)
              {
                //��ž��ġ stop
                TIM9->DIER &= ~(1<<0);	//disable the Tim9 Update interrupt
                TIM9->EGR &=~(1<<0);	//Update generation X 
                TIM9->CR1 &= ~(1<<0);	// Counter TIM9 disable
                BEEP();
                GPIOG->ODR &= ~( 3 << 4 ); //led off
                GPIOG->ODR |= (1 << 6); //led6 on
              }
        }
        if(EXTI->PR & 0x8000) //SW7 EXTI 
        {
              EXTI->PR |= 0x8000; //clear flag
              if(MODE == 2)
              {
                //��ž��ġ, ���ڴ� �ʱ�ȭ
                TIM9->DIER &= ~(1<<0);	//disable the Tim9 Update interrupt
                TIM9->EGR &=~(1<<0);	//Update generation X
                TIM9->CR1 &= ~(1<<0);	// Counter TIM9 disable
                sw_s = 0;
                sw_ms = 0;
                rec_s = 0;
                rec_ms = 0; //������ ��� 0���� �ʱ�ȭ
                BEEP();
                GPIOG->ODR &= ~( 7 << 4 ); //led off
                sprintf(sw,"%2X:%X ",sw_s, sw_ms); //���ڿ� ����
                sprintf(rec,"%2X:%X ",rec_s, rec_ms);
                LCD_SetTextColor(RGB_RED);	//���ڻ�
                LCD_DisplayText(2,5,sw);        //lcd�� �ʱ�ȭ�� ��ž��ġ, ���ڴ� �� ���
                LCD_DisplayText(3,8,rec);
              }
        }
}

void EXTI9_5_IRQHandler(void) //EXTI5~9 ���ͷ�Ʈ �ڵ鷯
{
         if(EXTI->PR & 0x0100) //SW0 EXTI 
        {
              EXTI->PR |= 0x0100; //clear flag
              
            MODE++; //sw ������ MODE ��ȭ
            BEEP();
            
            if(MODE > 4) //MODE�� 4���� Ŀ���� ����. ���� 1~4
              MODE = 1;
            
            if(MODE == 1)
            {
              ADC1->CR2 &= ~(1<<0);        // ADC1 OFF
              TIM3->DIER &= ~(1<<0);	//disable the Tim3 Update interrupt
              TIM3->EGR &= ~(1<<0);	//Update generation X 
              TIM3->CR1	&= ~(1<<0);	// Counter TIM3 disable
              TIM4->DIER &= ~(1<<0);	//disable the Tim3 Update interrupt
              TIM4->EGR &= ~(1<<0);	//Update generation X
              TIM4->CR1	&= ~(1<<0);	//Counter TIM4 disable
              
              LCD_Clear(RGB_WHITE); //MODE 1 ȭ�� ����
              LCD_SetTextColor(RGB_GREEN);
              LCD_DisplayText(1,1,"1. ALARM");
              LCD_SetTextColor(RGB_BLACK);
              LCD_DisplayText(2,1, "Alarm "); 
              LCD_SetTextColor(RGB_RED);
              sprintf(ala,"%X:%X",ala_h_, ala_m_);
              LCD_DisplayText(2,8,ala);
              
            }
            else if(MODE == 2)
            {
              //(MODE1 �˶��� �ٸ� ȭ���̿��� �۵��ؾ��ϱ� ������ �ʱ�ȭ, disable ����)
              LCD_Clear(RGB_WHITE); //MODE 2 ȭ�� ���� 
              LCD_SetTextColor(RGB_GREEN);
              LCD_DisplayText(1,1,"2. Stop Watch");
              LCD_SetTextColor(RGB_BLACK);
              LCD_DisplayText(2,1, "S-W ");  
              LCD_DisplayText(3,1, "Record "); 
              LCD_SetTextColor(RGB_RED);
              LCD_DisplayText(2,5, "00:0");  
              LCD_DisplayText(3,8, "00:0"); 
            
            }
            else if(MODE == 3)
            {
              TIM9->DIER &= ~(1<<0);	//disable the Tim9 Update interrupt
              TIM9->EGR &=~(1<<0);	//Update generation X
              TIM9->CR1 &= ~(1<<0);	// Counter TIM9 disable
              GPIOG->ODR &= ~( 7 << 4 ); //led off
              sw_s = sw_ms = rec_s = rec_ms = 0; //MODE 2 ���� �ʱ�ȭ
              
              LCD_Clear(RGB_WHITE); //MODE 3 ȭ�� ����
              LCD_SetTextColor(RGB_GREEN);
              LCD_DisplayText(1,1,"3. Calculator");
              LCD_SetTextColor(RGB_BLACK);
              LCD_DisplayText(2,2, "0 + 0 = 00");  
              USART1->CR1 	|= USART_CR1_UE;	//  0x2000, USART1 Enable
              
            }
            else if(MODE == 4)
            {
              USART1->CR1  &= ~USART_CR1_UE;	//  ~0x2000, USART1 disable
              cal_1 = cal_2 = cal_r = 0; //MODE 3 ���� �ʱ�ȭ
              
              LCD_Clear(RGB_WHITE); //MODE 4 ȭ�� ����
              LCD_SetTextColor(RGB_GREEN);
              LCD_DisplayText(1,1,"4. Thermostat");
              LCD_SetTextColor(RGB_BLACK);
              LCD_DisplayText(2,1,"T:");
              LCD_DisplayText(3,1,"H:  C:");
              
              ADC1->CR2 |= (1<<0);		// ADON: ADC1 ON
              TIM3->DIER |= (1<<0);	//Enable the Tim3 Update interrupt
              TIM3->EGR |=(1<<0);	//Update generation 
              TIM3->CR1	|= (1<<0);	// CEN: Counter TIM3 enable
              TIM4->DIER |= (1<<0);	//Enable the Tim4 Update interrupt
              TIM4->EGR |=(1<<0);	//Update generation 
              TIM4->CR1	|= (1<<0);	// CEN: Counter TIM4 enable
            }
    
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

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim10);		//��Ʈ   
	LCD_SetBackColor(RGB_WHITE);	//���ڹ���
        
        LCD_SetTextColor(RGB_GREEN);
        LCD_DisplayText(1,1,"1. ALARM");
        LCD_SetTextColor(RGB_BLACK);
        LCD_DisplayText(2,1, "Alarm "); 
        LCD_SetTextColor(RGB_RED);
        LCD_DisplayText(2,8,"0:0");
        LCD_SetBrushColor(RGB_YELLOW);
        LCD_DrawFillRect(80, 50, 60, 60);
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
uint16_t KEY_Scan_Navi(void)	// input key SW0 - SW7 
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

