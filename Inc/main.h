#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f3xx.h"
#include "gdi.h"
#include "game_engine.h"

volatile uint32_t ticks = 0;

volatile uint16_t line = 0;
volatile uint8_t image [5750] = {0};
gdi_handle handle = {image, 23 * 8, 250};
uint8_t game_field[81] = {0};

void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void delay_us(uint16_t us);
void RCC_Init(void);
void GPIO_Init(void);
void TIM3_Init(void);
void TIM1_Init(void);
void SPI1_Init(void);
void DMA1_Init(void);

void TIM1_UP_TIM16_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);

int main (void);

#endif
