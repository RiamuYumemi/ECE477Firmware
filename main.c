
#include "stm32f0xx.h"
#include <math.h>

void enable_port(void);
void TIM2_Init(void);
void TIM3_Init(void);
void Motor_Control(uint8_t dir, uint16_t speed);
void TIM6_ms_Delay(uint16_t delay);

void enable_port() {
    //enable clock for GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    //set PA5 and PA6 to alternate function mode
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1);
    //configure PA5 and PA6 as pulldown
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR5_1 | GPIO_PUPDR_PUPDR6_1);
    //select AF1 for PA5 and AF2 for PA6
    GPIOA->AFR[0] |= (~GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6);
}

void TIM2_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    //set clock freq to 1MHz
    TIM2->PSC = 16-1;
    //timer period = 20ms
    TIM2->ARR = 20000;
    TIM2->CNT = 0;
    //PWM mode for timer
    TIM2->CCMR1 = 0x0060;
    //enable channel 1 as output
    TIM2->CCER |= 1;
    //pulse width for PWM
    TIM2->CCR1 = 5000;
}

void TIM3_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    //set clock freq to 1MHz
    TIM2->PSC = 16-1;
    //timer period = 20ms
    TIM2->ARR = 20000;
    TIM2->CNT = 0;
    //PWM mode for timer
    TIM2->CCMR1 = 0x0060;
    //enable channel 1 as output
    TIM2->CCER |= 1;
    //pulse width for PWM
    TIM2->CCR1 = 5000;
}

void Motor_Control(uint8_t dir, uint16_t speed) {
    if(dir == 0) {
        while (TIM3->CNT <= TIM3->CCR1){};
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM6_ms_Delay(1);
        TIM2->CCR1 = (uint16_t)((speed / 225) * 20000);
    }
    else {
        while (TIM2->CNT <= TIM2->CCR1){};
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM6_ms_Delay(1);
        TIM3->CR1 |= TIM_CR1_CEN;
        TIM3->CCR1 = (uint16_t)((speed / 225) * 20000);
    }
}

void TIM6_ms_Delay(uint16_t delay) {
    RCC->APB1ENR |= 1<<2;
    TIM6->PSC = 16000 - 1;
    TIM6->CNT = 0;
    TIM6->CR1 |= 1;
    while(!(TIM6->SR & TIM_SR_UIF)) {}
    TIM6->SR &= ~(0x0001);
}


int main(void) {
    enable_port();
    TIM2_Init();
    TIM3_Init();
    TIM2->CR1 |= TIM_CR1_CEN;
    while(1) {
        Motor_Control (1, 100);
        TIM6_ms_Delay(10000);
        Motor_Control(0, 170);
        TIM6_ms_Delay(10000);
    }
    return 0;
}
