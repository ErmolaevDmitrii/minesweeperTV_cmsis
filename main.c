#include "stm32f3xx.h"

#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)

volatile uint32_t ticks = 0;

void SysTick_Handler(void) {
    ++ticks;
}

void delay_ms(uint32_t ms) {
    uint32_t start_time = ticks;
    while((ticks - start_time) < ms) {}
    return;
}

void delay_us(uint16_t us) {

    //TIM_EnableCounter(TIM3);
    TIM3->CNT = 1;
    while(TIM3->CNT < us) {}
    //TIM_DisableCounter(TIM3);
    return;
}

void RCC_Init(void) {
    // 0. Reset all clock registers to default clock config
    SystemInit();
    // 1. flash cycles count
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_2);
    // 2. bus dividers config
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk , RCC_CFGR_HPRE_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_HPRE_DIV2);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_HPRE_DIV1);
    // 3. hse generator start
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY)) {}
    // 4. pll config
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL_Msk, RCC_CFGR_PLLMUL9);
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);
    // 5. pll start
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY)) {}
    // 6. switch system clock input to pll
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);
    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    // 7. update clock value
    SystemCoreClockUpdate();
    return;
}

void TIM3_Init(void) {
    // 1. enable TIM3 on APB1 bus
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    // 2. enable global interrupts in NVIC
    //TIM3->CR1 = TIM_CR1_CEN;
    // 3. TIM3 timing config
    //WRITE_REG(TIM3->PSC, 71);
    //WRITE_REG(TIM3->ARR, 0xFFFF);
    SET_BIT(RCC->CFGR3, RCC_CFGR3_TIM34SW);
    TIM3->PSC = (uint16_t) 143;
    TIM3->ARR = 0xFFFF;
    TIM3->EGR |= 1;
    //CLEAR_BIT(TIM3->CR1, TIM_CR1_DIR);
    //SET_BIT(TIM3->CR1, TIM_CR1_UDIS);
    //NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;
    return;
}

int main (void)
{
    RCC_Init();
    SysTick_Config(SystemCoreClock / 1000);
    TIM3_Init();
    //TIM_EnableIT_UPDATE(TIM3);
    //TIM_EnableCounter(TIM3);

    RCC->AHBENR     |= RCC_AHBENR_GPIOBEN; //RCC ON

    GPIOB->MODER    |= GPIO_MODER_MODER7_0; //mode out
    GPIOB->OTYPER   = 0;
    GPIOB->OSPEEDR  = 0x03 << 14;

    RCC->AHBENR     |= RCC_AHBENR_GPIOAEN;		// Подаем тактирование на порт


    GPIOA->MODER    |= 0x02 << 16; //mode out
    GPIOA->OTYPER   = 0;
    GPIOA->OSPEEDR  |= 0x03 << 16;

    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLNODIV);
    RCC->CFGR	&=~(RCC_CFGR_MCO);
    RCC->CFGR |= 0x04 << 24;

    //SystemCoreClockUpdate();


    while (1)
    {
        delay_us(7);
        GPIOB->ODR ^=   GPIO_ODR_7;
        /*SET_BIT(GPIOB->ODR, GPIO_ODR_7);
        delay(2400000);
        CLEAR_BIT(GPIOB->ODR, GPIO_ODR_7);*/
    }
}
