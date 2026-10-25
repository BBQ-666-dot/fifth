#include <stdint.h>

extern int main(void);
extern void SystemInit(void);
extern void __libc_init_array(void);
extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK_HANDLER(name) void name(void) __attribute__((weak, alias("Default_Handler")))

WEAK_HANDLER(NMI_Handler);
WEAK_HANDLER(HardFault_Handler);
WEAK_HANDLER(MemManage_Handler);
WEAK_HANDLER(BusFault_Handler);
WEAK_HANDLER(UsageFault_Handler);
WEAK_HANDLER(SVC_Handler);
WEAK_HANDLER(DebugMon_Handler);
WEAK_HANDLER(PendSV_Handler);
WEAK_HANDLER(SysTick_Handler);
WEAK_HANDLER(WWDG_IRQHandler);
WEAK_HANDLER(PVD_IRQHandler);
WEAK_HANDLER(TAMP_STAMP_IRQHandler);
WEAK_HANDLER(RTC_WKUP_IRQHandler);
WEAK_HANDLER(FLASH_IRQHandler);
WEAK_HANDLER(RCC_IRQHandler);
WEAK_HANDLER(EXTI0_IRQHandler);
WEAK_HANDLER(EXTI1_IRQHandler);
WEAK_HANDLER(EXTI2_IRQHandler);
WEAK_HANDLER(EXTI3_IRQHandler);
WEAK_HANDLER(EXTI4_IRQHandler);
WEAK_HANDLER(DMA1_Stream0_IRQHandler);
WEAK_HANDLER(DMA1_Stream1_IRQHandler);
WEAK_HANDLER(DMA1_Stream2_IRQHandler);
WEAK_HANDLER(DMA1_Stream3_IRQHandler);
WEAK_HANDLER(DMA1_Stream4_IRQHandler);
WEAK_HANDLER(DMA1_Stream5_IRQHandler);
WEAK_HANDLER(DMA1_Stream6_IRQHandler);
WEAK_HANDLER(ADC_IRQHandler);
WEAK_HANDLER(CAN1_TX_IRQHandler);
WEAK_HANDLER(CAN1_RX0_IRQHandler);
WEAK_HANDLER(CAN1_RX1_IRQHandler);
WEAK_HANDLER(CAN1_SCE_IRQHandler);
WEAK_HANDLER(EXTI9_5_IRQHandler);
WEAK_HANDLER(TIM1_BRK_TIM9_IRQHandler);
WEAK_HANDLER(TIM1_UP_TIM10_IRQHandler);
WEAK_HANDLER(TIM1_TRG_COM_TIM11_IRQHandler);
WEAK_HANDLER(TIM1_CC_IRQHandler);
WEAK_HANDLER(TIM2_IRQHandler);
WEAK_HANDLER(TIM3_IRQHandler);
WEAK_HANDLER(TIM4_IRQHandler);
WEAK_HANDLER(I2C1_EV_IRQHandler);
WEAK_HANDLER(I2C1_ER_IRQHandler);
WEAK_HANDLER(I2C2_EV_IRQHandler);
WEAK_HANDLER(I2C2_ER_IRQHandler);
WEAK_HANDLER(SPI1_IRQHandler);
WEAK_HANDLER(SPI2_IRQHandler);
WEAK_HANDLER(USART1_IRQHandler);
WEAK_HANDLER(USART2_IRQHandler);
WEAK_HANDLER(USART3_IRQHandler);
WEAK_HANDLER(EXTI15_10_IRQHandler);
WEAK_HANDLER(RTC_Alarm_IRQHandler);
WEAK_HANDLER(OTG_FS_WKUP_IRQHandler);
WEAK_HANDLER(TIM8_BRK_TIM12_IRQHandler);
WEAK_HANDLER(TIM8_UP_TIM13_IRQHandler);
WEAK_HANDLER(TIM8_TRG_COM_TIM14_IRQHandler);
WEAK_HANDLER(TIM8_CC_IRQHandler);
WEAK_HANDLER(DMA1_Stream7_IRQHandler);
WEAK_HANDLER(FSMC_IRQHandler);
WEAK_HANDLER(SDIO_IRQHandler);
WEAK_HANDLER(TIM5_IRQHandler);
WEAK_HANDLER(SPI3_IRQHandler);
WEAK_HANDLER(UART4_IRQHandler);
WEAK_HANDLER(UART5_IRQHandler);
WEAK_HANDLER(TIM6_DAC_IRQHandler);
WEAK_HANDLER(TIM7_IRQHandler);
WEAK_HANDLER(DMA2_Stream0_IRQHandler);
WEAK_HANDLER(DMA2_Stream1_IRQHandler);
WEAK_HANDLER(DMA2_Stream2_IRQHandler);
WEAK_HANDLER(DMA2_Stream3_IRQHandler);
WEAK_HANDLER(DMA2_Stream4_IRQHandler);
WEAK_HANDLER(ETH_IRQHandler);
WEAK_HANDLER(ETH_WKUP_IRQHandler);
WEAK_HANDLER(CAN2_TX_IRQHandler);
WEAK_HANDLER(CAN2_RX0_IRQHandler);
WEAK_HANDLER(CAN2_RX1_IRQHandler);
WEAK_HANDLER(CAN2_SCE_IRQHandler);
WEAK_HANDLER(OTG_FS_IRQHandler);
WEAK_HANDLER(DMA2_Stream5_IRQHandler);
WEAK_HANDLER(DMA2_Stream6_IRQHandler);
WEAK_HANDLER(DMA2_Stream7_IRQHandler);
WEAK_HANDLER(USART6_IRQHandler);
WEAK_HANDLER(I2C3_EV_IRQHandler);
WEAK_HANDLER(I2C3_ER_IRQHandler);
WEAK_HANDLER(OTG_HS_EP1_OUT_IRQHandler);
WEAK_HANDLER(OTG_HS_EP1_IN_IRQHandler);
WEAK_HANDLER(OTG_HS_WKUP_IRQHandler);
WEAK_HANDLER(OTG_HS_IRQHandler);
WEAK_HANDLER(DCMI_IRQHandler);
WEAK_HANDLER(HASH_RNG_IRQHandler);
WEAK_HANDLER(FPU_IRQHandler);

__attribute__((section(".isr_vector"), used))
const uintptr_t __Vectors[] = {
    (uintptr_t)&_estack, (uintptr_t)Reset_Handler,
    (uintptr_t)NMI_Handler, (uintptr_t)HardFault_Handler,
    (uintptr_t)MemManage_Handler, (uintptr_t)BusFault_Handler,
    (uintptr_t)UsageFault_Handler, 0, 0, 0, 0,
    (uintptr_t)SVC_Handler, (uintptr_t)DebugMon_Handler, 0,
    (uintptr_t)PendSV_Handler, (uintptr_t)SysTick_Handler,
    (uintptr_t)WWDG_IRQHandler, (uintptr_t)PVD_IRQHandler,
    (uintptr_t)TAMP_STAMP_IRQHandler, (uintptr_t)RTC_WKUP_IRQHandler,
    (uintptr_t)FLASH_IRQHandler, (uintptr_t)RCC_IRQHandler,
    (uintptr_t)EXTI0_IRQHandler, (uintptr_t)EXTI1_IRQHandler,
    (uintptr_t)EXTI2_IRQHandler, (uintptr_t)EXTI3_IRQHandler,
    (uintptr_t)EXTI4_IRQHandler, (uintptr_t)DMA1_Stream0_IRQHandler,
    (uintptr_t)DMA1_Stream1_IRQHandler, (uintptr_t)DMA1_Stream2_IRQHandler,
    (uintptr_t)DMA1_Stream3_IRQHandler, (uintptr_t)DMA1_Stream4_IRQHandler,
    (uintptr_t)DMA1_Stream5_IRQHandler, (uintptr_t)DMA1_Stream6_IRQHandler,
    (uintptr_t)ADC_IRQHandler, (uintptr_t)CAN1_TX_IRQHandler,
    (uintptr_t)CAN1_RX0_IRQHandler, (uintptr_t)CAN1_RX1_IRQHandler,
    (uintptr_t)CAN1_SCE_IRQHandler, (uintptr_t)EXTI9_5_IRQHandler,
    (uintptr_t)TIM1_BRK_TIM9_IRQHandler, (uintptr_t)TIM1_UP_TIM10_IRQHandler,
    (uintptr_t)TIM1_TRG_COM_TIM11_IRQHandler, (uintptr_t)TIM1_CC_IRQHandler,
    (uintptr_t)TIM2_IRQHandler, (uintptr_t)TIM3_IRQHandler,
    (uintptr_t)TIM4_IRQHandler, (uintptr_t)I2C1_EV_IRQHandler,
    (uintptr_t)I2C1_ER_IRQHandler, (uintptr_t)I2C2_EV_IRQHandler,
    (uintptr_t)I2C2_ER_IRQHandler, (uintptr_t)SPI1_IRQHandler,
    (uintptr_t)SPI2_IRQHandler, (uintptr_t)USART1_IRQHandler,
    (uintptr_t)USART2_IRQHandler, (uintptr_t)USART3_IRQHandler,
    (uintptr_t)EXTI15_10_IRQHandler, (uintptr_t)RTC_Alarm_IRQHandler,
    (uintptr_t)OTG_FS_WKUP_IRQHandler, (uintptr_t)TIM8_BRK_TIM12_IRQHandler,
    (uintptr_t)TIM8_UP_TIM13_IRQHandler, (uintptr_t)TIM8_TRG_COM_TIM14_IRQHandler,
    (uintptr_t)TIM8_CC_IRQHandler, (uintptr_t)DMA1_Stream7_IRQHandler,
    (uintptr_t)FSMC_IRQHandler, (uintptr_t)SDIO_IRQHandler,
    (uintptr_t)TIM5_IRQHandler, (uintptr_t)SPI3_IRQHandler,
    (uintptr_t)UART4_IRQHandler, (uintptr_t)UART5_IRQHandler,
    (uintptr_t)TIM6_DAC_IRQHandler, (uintptr_t)TIM7_IRQHandler,
    (uintptr_t)DMA2_Stream0_IRQHandler, (uintptr_t)DMA2_Stream1_IRQHandler,
    (uintptr_t)DMA2_Stream2_IRQHandler, (uintptr_t)DMA2_Stream3_IRQHandler,
    (uintptr_t)DMA2_Stream4_IRQHandler, (uintptr_t)ETH_IRQHandler,
    (uintptr_t)ETH_WKUP_IRQHandler, (uintptr_t)CAN2_TX_IRQHandler,
    (uintptr_t)CAN2_RX0_IRQHandler, (uintptr_t)CAN2_RX1_IRQHandler,
    (uintptr_t)CAN2_SCE_IRQHandler, (uintptr_t)OTG_FS_IRQHandler,
    (uintptr_t)DMA2_Stream5_IRQHandler, (uintptr_t)DMA2_Stream6_IRQHandler,
    (uintptr_t)DMA2_Stream7_IRQHandler, (uintptr_t)USART6_IRQHandler,
    (uintptr_t)I2C3_EV_IRQHandler, (uintptr_t)I2C3_ER_IRQHandler,
    (uintptr_t)OTG_HS_EP1_OUT_IRQHandler, (uintptr_t)OTG_HS_EP1_IN_IRQHandler,
    (uintptr_t)OTG_HS_WKUP_IRQHandler, (uintptr_t)OTG_HS_IRQHandler,
    (uintptr_t)DCMI_IRQHandler, 0, (uintptr_t)HASH_RNG_IRQHandler,
    (uintptr_t)FPU_IRQHandler
};

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst;

    for (dst = &_sdata; dst < &_edata;)
        *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss;)
        *dst++ = 0;

    SystemInit();
    __libc_init_array();
    (void)main();
    while (1) {}
}

void Default_Handler(void)
{
    while (1) {}
}
