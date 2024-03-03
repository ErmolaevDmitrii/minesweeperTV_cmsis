#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f3xx.h"
#include "gdi.h"
#include "game_engine.h"

uint32_t interrupt = 0;

volatile uint32_t ticks = 0;

uint16_t line = 0;
uint8_t image [5750] = {0};
gdi_handle handle = {image, 23 * 8, 250};
uint8_t game_field[81] = {0};

typedef struct {
    uint32_t time_irq;
    uint8_t flag_irq;
    uint8_t flag_exec;
    uint32_t EXTI_PR;
    IRQn_Type IRQn;
    void (*action) (game_handle *);
} button_handle;

button_handle buttons[] = {
    {0, 0, 0, EXTI_PR_PR15 , EXTI15_10_IRQn, game_player_move_down },
    {0, 0, 0, EXTI_PR_PR10 , EXTI15_10_IRQn, game_player_move_up   },
    {0, 0, 0, EXTI_PR_PR11 , EXTI15_10_IRQn, game_player_move_left },
    {0, 0, 0, EXTI_PR_PR14 , EXTI15_10_IRQn, game_player_move_right},
    {0, 0, 0, EXTI_PR_PR8  , EXTI9_5_IRQn  , game_player_put_flag  },
    {0, 0, 0, EXTI_PR_PR7  , EXTI9_5_IRQn  , game_player_open_cell },
};

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
void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);

uint32_t GetTick(void);

int main (void);

#endif
