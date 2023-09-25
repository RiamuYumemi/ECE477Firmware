
#include "stm32f0xx.h"
#include <math.h>
#define N 1000
#define RATE 20000
int volume = 2048;

void enable_ports(void);
void TIM1_Init(void);


void enable_ports() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~0x30000;
    GPIOA->MODER |= 0x20000;
    GPIOA->AFR[1] |= 0x0002;
}
void TIM1_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->PSC = 4-1;
    TIM1->ARR = 2-1;
    TIM1->CCMR1 |= 0x6<<4 | 0x6<<12;
    TIM1->CCMR2 |= 0x6<<4 | 0x6<<12 | TIM_CCMR2_OC4PE;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->CR1 |= TIM_CR1_CEN;
}

short int wavetable[N];
int step = 440;
int offset = 0;

void init_wavetable(void) {
    for(int i = 0; i < N; i++) {
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
    }
}
void set_freq(float f) {
    step = f * N / RATE * (1<<16);
}

void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = (48 / (RATE % 9)) - 1; //24 - 1
    TIM7->ARR = (48000000 / ((48 / (RATE % 9)) * RATE)) - 1; //100 - 1
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] |= 1 << TIM7_IRQn;
}

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    offset += step;
    if((offset>>16) >= N) {
        offset -= N<<16;
    }
    int sample = wavetable[offset>>16];
    sample = ((sample * volume)>>17) + 1200;
    TIM1->CCR4 = sample;

}

int motor_driver(void) {
    enable_ports();
    TIM1_Init();
    init_wavetable();
    init_tim1();
    set_freq(1000.0);
    init_tim7();
    return 0;
}
