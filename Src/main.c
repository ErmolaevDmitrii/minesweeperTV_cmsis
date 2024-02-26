#include "main.h"

void SysTick_Handler(void) {
    ++ticks;
}

void delay_ms(uint32_t ms) {
    uint32_t start_time = ticks;
    while((ticks - start_time) < ms) {}
    return;
}

void delay_us(uint16_t us) {
    TIM3->CNT = 0;
    TIM3->EGR |= 1;
    //uint16_t a = TIM3->CNT;
    while(TIM3->CNT < us);
    return;
}

void RCC_Init(void) {
    // 0. Reset all clock registers to default clock config
    //SystemInit();
    // 1. flash cycles count
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_2);
    // 2. bus dividers config
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk , RCC_CFGR_HPRE_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE1_DIV2);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_DIV1);
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

void GPIO_Init(void) {
    //1. gpio buses init
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

    //2. pins init
    //SPI MOSI and SCK pins
    GPIOA->MODER    |= 0x02 << 10 | 0x02 << 14;
    GPIOA->AFR[0]   |= 0x05 << 20;
	GPIOA->AFR[0]   |= 0x05 << 28;

    //PAL SYNC pin
    GPIOB->MODER    |= GPIO_MODER_MODER5_0;
    GPIOB->OTYPER    = 0;
    GPIOB->OSPEEDR   = 0x03 << 10;

    //MCO pin
    GPIOA->MODER    |= 0x02 << 16;
    GPIOA->OTYPER    = 0;
    GPIOA->OSPEEDR  |= 0x03 << 16;
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLNODIV);
    RCC->CFGR	    &= ~(RCC_CFGR_MCO);
    RCC->CFGR       |= 0x04 << 24;

    //Red LED pin
    GPIOB->MODER    |= GPIO_MODER_MODER14_0;

    return;
}

void TIM3_Init(void) {
    // 1. enable TIM3 clock on APB1 bus
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // 2. set TIM3 settings
    TIM3->PSC = 71;
    TIM3->ARR = 0xFFFF;
    TIM3->CR1 |= 1 << TIM_CR1_ARPE_Pos;
    // 3. enable TIM3 counting
    TIM3->EGR |= 1;
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3->CR1 |= TIM_CR1_CEN;
    return;
}

void TIM1_Init(void) {
    // 1. enable TIM1 clock on APB2 bus
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    // 2. set TIM1 settings
    TIM1->SR = 0;
    TIM1->PSC = 2303;
    TIM1->ARR = 1;
    TIM1->CR1 |= 1 << TIM_CR1_ARPE_Pos;
    // 3. enable update interrupt for TIM1
    TIM1->DIER |= TIM_DIER_UIE;
    // 4. enable TIM1 counting
    TIM1->EGR |= 1;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM1->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    TIM1->CR1 |= TIM_CR1_CEN;
    return;
}

void SPI1_Init(void) {
    // 1. enable SPI1 clock on APB2 bus
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
    // 2. SPI1 settings
    SPI1->CR1 = 1 << SPI_CR1_BIDIMODE_Pos | 1 << SPI_CR1_BIDIOE_Pos |
                1 << SPI_CR1_LSBFIRST_Pos | 1 << SPI_CR1_MSTR_Pos   |
                1 << SPI_CR1_SSM_Pos      | 0x03 << SPI_CR1_BR_Pos  |
                1 << SPI_CR1_SSI_Pos;
    SPI1->CR2 = 0x07 << SPI_CR2_DS_Pos;
    SET_BIT(SPI1->CR2, SPI_CR2_TXDMAEN);
    // 3. enable SPI1
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    return;
}

void DMA1_Init(void) {
    //1. enable DMA1 clock on AHB bus
    SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
    //2. clear interrupt and flags registers
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1->IFCR, DMA_IFCR_CTEIF3 | DMA_IFCR_CHTIF3 |
                          DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3);
    //3. DMA1 settings
    DMA1_Channel3->CCR = 0 << DMA_CCR_MEM2MEM_Pos
                       | 0x03 << DMA_CCR_PL_Pos    | 0x00 << DMA_CCR_MSIZE_Pos
                       | 0x00 << DMA_CCR_PSIZE_Pos | 1 << DMA_CCR_MINC_Pos
                       | 0 << DMA_CCR_PINC_Pos     | 0 << DMA_CCR_CIRC_Pos
                       | 1 << DMA_CCR_DIR_Pos;
    return;
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    if(READ_BIT(TIM1->SR, TIM_SR_UIF))
    {
        CLEAR_BIT(TIM1->SR, TIM_SR_UIF);
        TIM1->SR;
        //GPIOB->ODR ^= GPIO_ODR_5;
        CLEAR_BIT(GPIOB->ODR, GPIO_ODR_5);

        if(line == 313) {
            delay_us(28);
            SET_BIT(GPIOB->ODR, GPIO_ODR_5);
            delay_us(4);
            CLEAR_BIT(GPIOB->ODR, GPIO_ODR_5);
            delay_us(28);
            SET_BIT(GPIOB->ODR, GPIO_ODR_5);
            delay_us(3);
            TIM1->CNT = 0;
            TIM1->EGR |= 1;
        }

        delay_us(4);
        SET_BIT(GPIOB->ODR, GPIO_ODR_5);
        delay_us(7);

        void *line_pointer = image + (line - 40) * 23;

        if(line >= 40 && line < 290) {
            CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
            CLEAR_BIT(DMA1->IFCR, DMA_IFCR_CTEIF3 | DMA_IFCR_CHTIF3 |
                                  DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3);

            DMA1_Channel3->CPAR = (uint32_t)(&SPI1->DR);
            DMA1_Channel3->CMAR = (uint32_t)line_pointer;
            DMA1_Channel3->CNDTR = 23;

            DMA1_Channel3->CCR |= 1 << DMA_CCR_EN_Pos;
        }
        if(line >= 313) {
            line = 0;
            return;
        }
        ++line;
    }

    return;
}

uint32_t Cock() {
    return 1000;
}

void draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    for(int i = y; i < y + height; ++i) {
        for(int j = x; j < x + width; ++j) {
            if(j / 8 == 19) {
              continue;
            }
            image[i * 23 + j / 8] |= (1 << (j % 8));
        }
    }
}


int main (void)
{
    //Peripherial init section
    RCC_Init();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);
    GPIO_Init();
    DMA1_Init();
    SPI1_Init();
    TIM3_Init();
    TIM1_Init();

    game_handle game = {9, 9, game_field, &handle, Cock};
    game_start(&game);

    while (1)
    {
        __NOP();
    }
}
