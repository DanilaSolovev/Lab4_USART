
#include "main.h"

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */
#define Start                      ((uint8_t)0x01)
#define Load_tumbler               ((uint8_t)0x02)
#define Stop                       ((uint8_t)0x03)
#define Reset                      ((uint8_t)0x04)
//Пины, используемые в проекте
#define Led1                       (GPIO_PIN_8)
#define Led2                       (GPIO_PIN_9)
#define Led3                       (GPIO_PIN_10)
#define Led4                       (GPIO_PIN_11)
#define Led5                       (GPIO_PIN_12)
#define Led6                       (GPIO_PIN_8)
#define Led7                       (GPIO_PIN_9)
#define Led8                       (GPIO_PIN_10)

#define Tumb1                       (GPIO_PIN_1)
#define Tumb2                       (GPIO_PIN_2)
#define Tumb3                       (GPIO_PIN_3)
#define Tumb4                       (GPIO_PIN_4)
#define Tumb5                       (GPIO_PIN_5)
#define Tumb6                       (GPIO_PIN_6)
#define Tumb7                       (GPIO_PIN_7)
#define Tumb8                       (GPIO_PIN_11)
/*Определение прототипов функциий*/
void beg_og(uint32_t tick_delay);
void init_IT(void);
void init_Int_UART(void);
void gpio_settings_set(GPIO_TypeDef* g, uint16_t pin, uint8_t io);
void delay (uint32_t ticks);
uint32_t tumblrch(void);
uint8_t uvernaj(GPIO_TypeDef* g, uint16_t pin, uint32_t n);
void led_set(uint8_t state);
void led_change(uint8_t* led_change);
void UART2cnf(void);
void UARTSend(int8_t c);
uint8_t UARTResive(void);
void UARTSendSTR(char *string);
// Определение переменных
uint32_t pin_state = 0;
uint32_t spd = 0;
uint32_t spdnew = 0;
uint32_t tumblr = 0;
uint8_t led_state=0b10000000;
uint8_t uver = 0;
uint8_t data = 0;










int main(void)
{
// Инициализация пинов на выход		
    gpio_settings_set(GPIOA, Led1, 1);
    gpio_settings_set(GPIOA, Led2, 1);
    gpio_settings_set(GPIOA, Led3, 1);
    gpio_settings_set(GPIOA, Led4, 1);
    gpio_settings_set(GPIOA, Led5, 1);
    gpio_settings_set(GPIOC, Led6, 1);
    gpio_settings_set(GPIOC, Led7, 1);
    gpio_settings_set(GPIOC, Led8, 1);
// Инициализация пинов на вход
    gpio_settings_set(GPIOC, GPIO_PIN_13, 0);
    gpio_settings_set(GPIOC, Tumb1, 0);
    gpio_settings_set(GPIOC, Tumb2, 0);
    gpio_settings_set(GPIOC, Tumb3, 0);
    gpio_settings_set(GPIOC, Tumb4, 0);
    gpio_settings_set(GPIOC, Tumb5, 0);
    gpio_settings_set(GPIOC, Tumb6, 0);
    gpio_settings_set(GPIOC, Tumb7, 0);
	gpio_settings_set(GPIOC, Tumb8, 0);
   
    
    
    
    
	/*Инициализация прерываний*/
	init_IT();
	UART2cnf();
    init_Int_UART();
    led_set(0b10000000);
    pin_state = Start;
  while (1)
  {
	    beg_og(1000000);
  }

    
//    while(1)
//    {
//        data = UARTResive();
//        UARTSend(data);
//        
//        delay(100000);
//    }
    





}
void init_Int_UART(void)
{
    //Разрешить прерывание 2 и 3 пина порта A
	AFIO->EXTICR[0] &= (~0xFF)>>8  ;
	//Разрешить прерывание 2 и 3 линии
	EXTI->IMR|=(EXTI_IMR_MR2);
	EXTI->EMR|=(EXTI_EMR_MR2);
    EXTI->IMR|=(EXTI_IMR_MR3);
	EXTI->EMR|=(EXTI_EMR_MR3);
	
	//Прерывание 2 и 3 линии по спадающему фронту
	EXTI->RTSR|=(EXTI_RTSR_TR2);
    EXTI->RTSR|=(EXTI_RTSR_TR3);
	
	/* Разрешение прерываний */
    NVIC->ISER[(((uint32_t)USART2_IRQn ) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)USART2_IRQn ) & 0x1FUL));
    USART2->CR1 |= (1<<5); /*Прерывание при приеме данных*/

}
void UART2cnf(void)
{
    RCC->APB1ENR |= (1<<17);
    RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_IOPAEN_Msk;
    GPIOA->CRL = 0;
    GPIOA->CRL &= ~(3<<8);
    GPIOA->CRL |= (2<<10);
	
    GPIOA->CRL &= ~(3<<12);
    GPIOA->CRL |= (2<<14);
	
    GPIOA->ODR |= 1<<3;
    
    USART2->CR1 |= 0x00;
    USART2->CR1 |= (1<<13);
    USART2->CR1 &= ~(1<<12);
    USART2->BRR =0x22C;
    
    USART2->CR1 |= (1<<2);
    USART2->CR1 |= (1<<3);
}
void UARTSend(int8_t c)
{
    USART2->DR = c;
	while (!(USART2->SR & (1<<6)));
}
void UARTSendSTR(char *string)
{
    while (*string) UARTSend (*string++);
}
uint8_t UARTResive(void)
{
    uint8_t Resive;
	Resive = USART2->DR;
	return Resive;
}
void beg_og(uint32_t tick_delay)
{
	/****************************************************************/
	/*Программа моргания светодиодом каждые tick_delay тиков*/
	/****************************************************************/
    switch(pin_state)
        {
        case Load_tumbler:
            spdnew = UARTResive();
        case Start:
            led_set(led_state);
            delay(tick_delay/(1+spd));
            led_change(&led_state);
            break;
        case Stop:
            spd=spdnew;
            break;
        case Reset:
            led_set(0x00);
            led_state=0b10000000;
            break;
        default:
            break;
        }
    
}

void gpio_settings_set(GPIO_TypeDef* g, uint16_t pin, uint8_t io)
{
    if (g == GPIOA)
    {
        /*Включение тактирования для порта A*/
        RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_IOPAEN_Msk;
     
    }
    else
    {
        /*Включение тактирования для порта C*/
        RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_IOPCEN_Msk;
	}
    
    switch (pin)
        {
            case GPIO_PIN_0:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE0_Pos) | (0x02 << GPIO_CRL_CNF0_Pos);
  
                g->ODR |= (1<<0); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1);
                /* Установка регистра CRH битов CNF0[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF0_0 | GPIO_CRL_CNF0_1));
            }
            break;
            case GPIO_PIN_1:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE1_Pos) | (0x02 << GPIO_CRL_CNF1_Pos);
  
                g->ODR |= (1<<1); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);
                /* Установка регистра CRH битов CNF1[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1));
            }
            break;
            case GPIO_PIN_2:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE2_Pos) | (0x02 << GPIO_CRL_CNF2_Pos);
  
                g->ODR |= (1<<2); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1);
                /* Установка регистра CRH битов CNF2[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1));
            }    
            break;
            case GPIO_PIN_3:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE3_Pos) | (0x02 << GPIO_CRL_CNF3_Pos);
  
                g->ODR |= (1<<3); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1);
                /* Установка регистра CRH битов CNF3[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF3_0 | GPIO_CRL_CNF3_1));
            }    
            break;
            case GPIO_PIN_4:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE4_Pos) | (0x02 << GPIO_CRL_CNF4_Pos);
  
                g->ODR |= (1<<4); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1);
                /* Установка регистра CRH битов CNF4[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF4_0 | GPIO_CRL_CNF4_1));
            }    
            break;
            case GPIO_PIN_5:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE5_Pos) | (0x02 << GPIO_CRL_CNF5_Pos);
  
                g->ODR |= (1<<5); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1);
                /* Установка регистра CRH битов CNF5[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF5_0 | GPIO_CRL_CNF5_1));
            }    
            break;
            case GPIO_PIN_6:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE6_Pos) | (0x02 << GPIO_CRL_CNF6_Pos);
  
                g->ODR |= (1<<6); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1);
                /* Установка регистра CRH битов CNF6[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1));
            }    
            break;
            case GPIO_PIN_7:
            if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRL |= (0x00 << GPIO_CRL_MODE7_Pos) | (0x02 << GPIO_CRL_CNF7_Pos);
  
                g->ODR |= (1<<7); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRL = (g->CRL) | ( GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1);
                /* Установка регистра CRH битов CNF7[1:0] на тип выхода push-pull */
                g->CRL = (g->CRL) & ( ~( GPIO_CRL_CNF7_0 | GPIO_CRL_CNF7_1));
            }    
            break;
            case GPIO_PIN_8:
                if (io == 0)
            {
                /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE8_Pos) | (0x02 << GPIO_CRH_CNF8_Pos);
  
                g->ODR |= (1<<8); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1);
                /* Установка регистра CRH битов CNF8[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1));
            }    
            break;
            case GPIO_PIN_9:
            if (io == 0)
            {
                 /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE9_Pos) | (0x02 << GPIO_CRH_CNF9_Pos);
  
                g->ODR |= (1<<9); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1);
                /* Установка регистра CRH битов CNF9[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1));
            }    
            break;
            case GPIO_PIN_10:
                 if (io == 0)
            {
                 /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE10_Pos) | (0x02 << GPIO_CRH_CNF10_Pos);
  
                g->ODR |= (1<<10); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1);
                /* Установка регистра CRH битов CNF10[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF10_0 | GPIO_CRH_CNF10_1));
            }    
            break;
            case GPIO_PIN_11:
                if (io == 0)
            {
                 /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE11_Pos) | (0x02 << GPIO_CRH_CNF11_Pos);
  
                g->ODR |= (1<<11); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1);
                /* Установка регистра CRH битов CNF11[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF11_0 | GPIO_CRH_CNF11_1));
            }    
            break;
            case GPIO_PIN_12:
             if (io == 0)
            {
                 /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE12_Pos) | (0x02 << GPIO_CRH_CNF12_Pos);
  
                g->ODR |= (1<<12); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1);
                /* Установка регистра CRH битов CNF12[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF12_0 | GPIO_CRH_CNF12_1));
            }
            break;
            case GPIO_PIN_13:
             if (io == 0)
            {
                 /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE13_Pos) | (0x02 << GPIO_CRH_CNF13_Pos);
  
                g->ODR |= (1<<13); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);
                /* Установка регистра CRH битов CNF13[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1));
            }
            break;
            case GPIO_PIN_14:
                if (io == 0)
            {
                 /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE14_Pos) | (0x02 << GPIO_CRH_CNF14_Pos);
  
                g->ODR |= (1<<14); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1);
                /* Установка регистра CRH битов CNF14[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF14_0 | GPIO_CRH_CNF14_1));
            }
            break;
            case GPIO_PIN_15:
            if (io == 0)
            {
                 /// Настраиваем на вход с подтяжкой к питанию ///
                g->CRH &= ~(GPIO_CRH_MODE15 | GPIO_CRH_CNF15);
                //MODE: вход, оставляем в нуле
                //CNF: вход с pull-up / pull-down
                g->CRH |= (0x00 << GPIO_CRH_MODE15_Pos) | (0x02 << GPIO_CRH_CNF15_Pos);
  
                g->ODR |= (1<<15); //Включаем подтяжку вверх
            }
            else 
            {   
                g->CRH = (g->CRH) | ( GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1);
                /* Установка регистра CRH битов CNF15[1:0] на тип выхода push-pull */
                g->CRH = (g->CRH) & ( ~( GPIO_CRH_CNF15_0 | GPIO_CRH_CNF15_1));
            }
            default:
            break;   
        }
   
}
void led_set(uint8_t state)
{
    if ((state & 0b10000000) == 0)
    {
        GPIOA->BRR = GPIOA->BRR | Led1;
    }
    else GPIOA->BSRR = GPIOA->BSRR | Led1;
    
    if ((state & 0b01000000) == 0)
    {
        GPIOA->BRR = GPIOA->BRR | Led2;
    }
    else GPIOA->BSRR = GPIOA->BSRR | Led2;
    
    if ((state & 0b00100000) == 0)
    {
        GPIOA->BRR = GPIOA->BRR | Led3;
    }
    else GPIOA->BSRR = GPIOA->BSRR | Led3;
    
    if ((state & 0b00010000) == 0)
    {
        GPIOA->BRR = GPIOA->BRR | Led4;
    }
    else GPIOA->BSRR = GPIOA->BSRR | Led4;
    
    if ((state & 0b00001000) == 0)
    {
        GPIOA->BRR = GPIOA->BRR | Led5;
    }
    else GPIOA->BSRR = GPIOA->BSRR | Led5;
    
    if ((state & 0b00000100) == 0)
    {
        GPIOC->BRR = GPIOC->BRR | Led6;
    }
    else GPIOC->BSRR = GPIOC->BSRR | Led6;
    
    if ((state & 0b00000010) == 0)
    {
        GPIOC->BRR = GPIOC->BRR | Led7;
    }
    else GPIOC->BSRR = GPIOC->BSRR | Led7;
    
    if ((state & 0b00000001) == 0)
    {
        GPIOC->BRR = GPIOC->BRR | Led8;
    }
    else GPIOC->BSRR = GPIOC->BSRR | Led8;
    
}
uint32_t tumblrch(void)
{
    volatile uint8_t spdv = 0;
   
    //Считаем содержимое пина 1
    uver = uvernaj(GPIOC, Tumb1, 3);
    if (uver==1)
    {
            spdv = spdv | 0b00000001;
    }
    //Считаем содержимое пина 2
    uver = uvernaj(GPIOC, Tumb2, 3);
    if (uver==1)
    {
            spdv = spdv | 0b00000010;
    }
        //Считаем содержимое пина 3
    uver = uvernaj(GPIOC, Tumb3, 3);
    if (uver==1)
    {
            spdv = spdv | 0b00000100;
    }
        //Считаем содержимое пина 4
    uver = uvernaj(GPIOC, Tumb4, 3);
    if (uver==1)
    {
            spdv = spdv | 0b00001000;
    }
        //Считаем содержимое пина 5
    uver = uvernaj(GPIOC, Tumb5, 3);
    if (uver==1)
    {
            spdv = spdv | 0b00010000;
    }
        //Считаем содержимое пина 6
    uver = uvernaj(GPIOC, Tumb6, 3);
    if (uver==1)
    {
            spdv = spdv | 0b00100000;
    }
        //Считаем содержимое пина 7
    uver = uvernaj(GPIOC, Tumb7, 3);
    if (uver==1)
    {
            spdv = spdv | 0b01000000;
    }
        //Считаем содержимое пина 11
    uver = uvernaj(GPIOC, Tumb8, 3);
    if (uver==1)
    {
            spdv = spdv | 0b10000000;
    }
    return spdv;
}
void led_change(uint8_t* led_change)
{
    if ((*led_change & 0b00000001) == 0) *led_change = *led_change >> 1;
    else *led_change = 0b10000000;
}
void init_IT(void)
{
	/****************************************************************/
	/*Включение прерываний от пина 13*/
	/****************************************************************/
	//Включение тактирования на блок альтернативных функций
	RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_AFIOEN;
	//Разрешить прерывание 13 пина порта С
	AFIO->EXTICR[3] = 0x0020;
	//Разрешить прерывание 13 линии
	EXTI->IMR|=(EXTI_IMR_MR13);
	EXTI->EMR|=(EXTI_EMR_MR13);
	
	//Прерывание 13 линии по спадающему фронту фронту
	EXTI->RTSR|=(EXTI_RTSR_TR13);
	
	/* Разрешение прерываний */
	NVIC->ISER[(((uint32_t)EXTI15_10_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)EXTI15_10_IRQn) & 0x1FUL));
}

void delay (uint32_t ticks)
{
	for (uint32_t i = 0; i < ticks; i++)
	{
	}
}

uint8_t uvernaj(GPIO_TypeDef* g, uint16_t pin, uint32_t n)
{
        uver=0;
        for (uint8_t i = 0; i < n; i++)
        {
        if ((g->IDR & pin)==0) uver++;
        }
        if (uver==n) uver=1;
        else if (uver==0) uver=0;
        else uver=2;
        return uver;
}
/*Обработчик прерывания по линии EXTI15_10*/
void USART2_IRQnHandler (void)
{
        EXTI->PR |= GPIO_PIN_3;
        spdnew = UARTResive();
}
void EXTI15_10_IRQHandler (void)
{
		EXTI->PR |= GPIO_PIN_13;
        uver = uvernaj(GPIOC, GPIO_PIN_13, 2);
        if (uver==1)
        {
            if (pin_state == Reset)
            {
                pin_state = Start;
            }
            else
            {
                pin_state ++;
            }
        }
}



void HardFault_Handler (void)
{
	while(1)
	{}
}
