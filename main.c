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
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));
    // 4. pll config
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL_Msk, RCC_CFGR_PLLMUL9);
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);
    // 5. pll start
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));
    // 6. switch system clock input to pll
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);
    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
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

void SPI1_Init(void) {
    // 1. GPIO pins used by SPI1 init
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    GPIOA->MODER |= 0x02 << 10 | 0x02 << 14;

    GPIOA->AFR[0] |= (0x05<<5*4);
	//GPIOA->AFR[0] |= (0x05<<6*4);
	GPIOA->AFR[0] |= (0x05<<7*4);
    // 2. enable SPI1 clock on APB2 bus
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
    //GPIOA->OTYPER = 0;
    // 3. SPI1 init
    SPI1->CR1 = 1 << SPI_CR1_BIDIMODE_Pos | 1 << SPI_CR1_BIDIOE_Pos |
                1 << SPI_CR1_LSBFIRST_Pos | 1 << SPI_CR1_MSTR_Pos   |
                1 << SPI_CR1_SSM_Pos      | 0x02 << SPI_CR1_BR_Pos  |
                1 << SPI_CR1_SSI_Pos      | 1 << SPI_CR1_LSBFIRST_Pos;
    SPI1->CR2 = 0x07 << SPI_CR2_DS_Pos;
    // 4. enable SPI1
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    //SPI1->CR1 |= 1 << SPI_CR1_SPE;
    return;
}

int main (void)
{
    //Peripherial init section
    RCC_Init();
    SysTick_Config(SystemCoreClock / 1000);
    SPI1_Init();
    TIM3_Init();
    //
    //TIM_EnableIT_UPDATE(TIM3);
    //TIM_EnableCounter(TIM3);

    RCC->AHBENR     |= RCC_AHBENR_GPIOBEN; //RCC ON

    GPIOB->MODER    |= GPIO_MODER_MODER7_0; //mode out
    GPIOB->OTYPER   = 0;
    GPIOB->OSPEEDR  = 0x03 << 14;

    RCC->AHBENR     |= RCC_AHBENR_GPIOAEN;		// Подаем тактирование на порт


    GPIOA->MODER    |= 0x02 << 16; //mode out
    GPIOA->OTYPER    = 0;
    GPIOA->OSPEEDR  |= 0x03 << 16;
    //GPIOA->AFR[1]     |= 0x00 << ;

    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLNODIV);
    RCC->CFGR	&=~(RCC_CFGR_MCO);
    RCC->CFGR |= 0x04 << 24;

    //SystemCoreClockUpdate();


    while (1)
    {
        delay_us(100);

        while(!(SPI1->SR & SPI_SR_TXE));

        if(SPI1->SR & SPI_SR_BSY) {
            GPIOB->ODR |= GPIO_ODR_7;
            while(1);
        }

        //GPIOB->ODR ^=   GPIO_ODR_7;
        *(uint8_t*)&SPI1->DR = 0b0000001;
    }
}
