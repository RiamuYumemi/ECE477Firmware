/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include "lcd.h"
#include <stdint.h>
#include <stdlib.h>

#define PB_MOSI (15)
#define PB_SCK (13)
#define PB_DC (10)
#define PB_CS (11)
#define PB_RST (12)


#ifdef VVC_F0
  GPIOB->AFR[0]  &= ~(GPIO_AFRL_AFSEL3 |
                      GPIO_AFRL_AFSEL5);
#elif  VVC_L0
  GPIOB->AFR[0]  &= ~(GPIO_AFRL_AFRL3 |
                      GPIO_AFRL_AFRL5);
#endif

void __attribute__((optimize("O0"))) delay_cycles(uint32_t cyc) {
  uint32_t d_i;
  for (d_i = 0; d_i < cyc; ++d_i) {
    asm("NOP");
  }
}

void init_lcd_spi_pins(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;


    GPIOB -> MODER &= ~((0x3 << (PB_MOSI * 2)) |
            		   (0x3 << (PB_SCK  * 2)) |
                       (0x3 << (PB_DC   * 2)) |
					   (0x3 << (PB_CS  * 2)) |
					   (0x3 << (PB_RST  * 2)));
    //Use pull down
    GPIOB->MODER   |=  ((0x2 << (PB_MOSI * 2)) |
                        (0x2 << (PB_SCK  * 2)) |
                        (0x1 << (PB_DC   * 2)) |
						(0x1 << (PB_CS  * 2)) |
						(0x1 << (PB_RST  * 2)));

    // Use pull-down resistors for the SPI peripheral?
    // Or no pulling resistors?
    GPIOB->PUPDR   &= ~((0x3 << (PB_MOSI * 2)) |
                        (0x3 << (PB_SCK  * 2)) |
                        (0x3 << (PB_DC   * 2)) |
						(0x3 << (PB_CS   * 2)) |
						(0x3 << (PB_RST   * 2)));


    GPIOB->PUPDR  |=   ((0x1 << (PB_MOSI * 2)) |
                        (0x1 << (PB_SCK  * 2)));

    // Output type: Push-pull
    GPIOB->OTYPER  &= ~((0x1 << PB_MOSI) |
                        (0x1 << PB_SCK)  |
                        (0x1 << PB_DC)   |
						(0x1 << PB_CS)   |
						(0x1 << PB_RST));
    // High-speed - 50MHz maximum
    // (Setting all '1's, so no need to clear bits first.)
    GPIOB->OSPEEDR |=  ((0x3 << (PB_MOSI * 2)) |
                        (0x3 << (PB_SCK  * 2)) |
                        (0x3 << (PB_DC   * 2)));

    // Set initial pin values.
    //   (The 'Chip Select' pin tells the display if it
    //    should be listening. '0' means 'hey, listen!', and
    //    '1' means 'ignore the SCK/MOSI/DC pins'.)
    GPIOB->ODR |=  (1 << PB_CS);
    //   (See the 'sspi_cmd' method for 'DC' pin info.)
    GPIOB->ODR |=  (1 << PB_DC);
    // Set SCK high to start
    GPIOB->ODR |=  (1 << PB_SCK);
    // Reset the display by pulling the reset pin low,
    // delaying a bit, then pulling it high.
    GPIOB->ODR &= ~(1 << PB_RST);
    // Delay at least 100ms; meh, call it 2 million no-ops.
    delay_cycles(2000000);
    GPIOB->ODR |=  (1 << PB_RST);
    delay_cycles(2000000);

    // Make sure that the peripheral is off, and reset it.
    SPI1->CR1 &= ~(SPI_CR1_SPE);
    RCC->APB2RSTR |=  (RCC_APB2RSTR_SPI1RST);
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);
    // Set clock polarity and phase.
    SPI1->CR1 |=  (SPI_CR1_CPOL |
                   SPI_CR1_CPHA);

    // Set the STM32 to act as a host device.
    SPI1->CR1 |=  (SPI_CR1_MSTR);
    // Set software 'Chip Select' pin.
    SPI1->CR1 |=  (SPI_CR1_SSM);
    // (Set the internal 'Chip Select' signal.)
    SPI1->CR1 |=  (SPI_CR1_SSI);

    // Enable the peripheral.
    SPI1->CR1 |=  (SPI_CR1_SPE);


}


void setup_serial(void)
{
    RCC->AHBENR |= 0x00180000;
    GPIOC->MODER  |= 0x02000000;
    GPIOC->AFR[1] |= 0x00020000;
    GPIOD->MODER  |= 0x00000020;
    GPIOD->AFR[0] |= 0x00000200;
    RCC->APB1ENR |= 0x00100000;
    USART5->CR1 &= ~0x00000001;
    USART5->CR1 |= 0x00008000;
    USART5->BRR = 0x340;
    USART5->CR1 |= 0x0000000c;
    USART5->CR1 |= 0x00000001;
}



int main(void)
{
	setup_serial();
	init_lcd_spi_pins();
	LCD_Setup();


	while(1){
		LCD_DrawFillRectangle(0, 0, 20, 20, BLUE);
	};
}
