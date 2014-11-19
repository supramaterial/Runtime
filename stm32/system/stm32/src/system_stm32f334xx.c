/*
 * Copyright (c) 2014 ELL-i co-operative
 *
 * This file is part of ELL-i software.
 *
 * ELL-i software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ELL-i software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ELL-i software.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @author  Ivan Raul Lopez Guadarrama <ivan.raul@ell-i.org> 2014
 *
 * @brief   STM32F334X8 devices initialisation code.
 *          Based on system_stm32f3xx.c, for STM32F334x8,
 *          distributed by STMicroelectronics.
 */

#include "stm32f3xx.h"


#ifndef HSE_VALUE
    #define HSE_VALUE   ((uint32_t)8000000)
#endif

#ifndef  HSI_VALUE
    #define HSI_VALUE   ((uint32_t)8000000)
#endif

#define VECT_TAB_OFFSET     0x0


uint32_t SystemCoreClock = 64000000;


/* Set clock configuration to Reset state, disable all
 * interrupts, configure FPU access if required and relocate
 * the Vector table if necessary. This function configures
 * the system to use HSI (8 MHz) as SYSCLK, and resets all
 * the prescalers and PLL multipliers. It uses bitmasks
 * as indicated by the RM0364 Reference Manual (DM00093941.pdf)
 */

void SystemInit(void) {

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= 0x00F00000;  /* set CP10 and CP11 */
#endif

    /* Set HSION bit */
    RCC->CR |= 0x00000001;

    /* Reset MCO[2:0], PPRE2[2:0], PPRE1[2:0],
    * HPRE[3:0] and SW[1:0] bits
    */
    RCC->CFGR &= 0xF8FFC00C;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= 0xFEF6FFFF;

    /* Reset HSEBYP bit */
    RCC->CR &= 0xFFFBFFFF;

    /* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
    RCC->CFGR &= 0xFFC0FFFF;

    /* Reset PREDIV1[3:0] bits */
    RCC->CFGR2 &= 0xFFFFFFF0;

    /* Reset USART3SW[1:0], USART3SW[1:0], HRTIM1SW,
    * TIM1SW, I2CSW and USART1SW[1:0] bits
    */
    RCC->CFGR3 &= 0xFFF0EEEC;

    /* Disable all interrupts */
    RCC->CIR = 0x00000000;

#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Move Vector Table to Internal SRAM */
#endif

}

/*
 * Configure the PLL (clocked by HSI) used as SYSCLK,
 * at the maximum frequency of 64 Mhz (multiplier x8).
 *
 * The STM32F334 Nucleo board does not have an
 * oscilator present for providing HSE, so
 * HSI is the only possible source for the PLL
 * without soldering additional components.
 *
 * Frequency Limits:
 *   HCLK  = SYSCLK / (AHB prescaler) ->  72 MHz
 *   PCLK1 = HCLK / (APB1 prescaler)  ->  72 Mhz
 *   PCLK2 = HCLK / (APB2 prescaler)  ->  36 Mhz
 *   HRTIM = PLLCLK x 2               -> 144 Mhz
 *
 * When using the PLL at max frequency configuration,
 * a ratio of 2:1 should be mantained between
 * the frequency of SYSCLK to PCLK2, either by
 * the AHB prescaler or the APB2 prescaler.
 */

void SystemSetSysClock(void) {

    /* Wait until HSI is ready. */
    while ((RCC->CR & RCC_CR_HSIRDY) == 0)
        ;

    /* Enable the HSI */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;

    /* HCLK = SYSCLK / 1 */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    /* PCLK1 = HCLK / 1 */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
    /* PCLK2 = HCLK / 2 */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* PLLMUL = HSI * 8 */
    RCC->CFGR &= ~RCC_CFGR_PLLMUL;
    RCC->CFGR |= RCC_CFGR_PLLMUL8;

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
        ;

    /* Configure Flash
    *    Prefetch buffer enabled,
    *    Half cycle access disabled,
    *    Latency 2 wait states (48 MHz < SYSCLK < 72 MHz)
    */
    FLASH->ACR = FLASH_ACR_PRFTBE | !FLASH_ACR_HLFCYA | FLASH_ACR_LATENCY_2;

    /* Select the main PLL as system clock source */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
        ;
}


void SystemCoreClockUpdate (void) {
    /* XXX WIP */
}
