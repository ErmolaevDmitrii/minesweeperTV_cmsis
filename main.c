
#include "stm32f303xe.h"

void __attribute__((optimize("O0"))) delay (uint32_t time)
{
    static uint32_t i;
    for (i=0; i<time; i++) {}
}

int main (void)
{

    RCC->AHBENR     |= RCC_AHBENR_GPIOBEN; //RCC ON

    GPIOB->MODER    |= GPIO_MODER_MODER7_0; //mode out
    GPIOB->OTYPER   = 0;
    GPIOB->OSPEEDR  = 0;

    while (1)
    {
        delay(100000);
        GPIOB->ODR ^=   GPIO_ODR_7;
    }
}
