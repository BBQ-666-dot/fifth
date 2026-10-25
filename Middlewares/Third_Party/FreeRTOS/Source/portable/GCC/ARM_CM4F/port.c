#include "FreeRTOS.h"
#include "task.h"

#ifndef __VFP_FP__
#error Hardware floating point must be enabled for the Cortex-M4F port
#endif

#define portNVIC_SYSTICK_CTRL_REG (*((volatile uint32_t *)0xe000e010))
#define portNVIC_SYSTICK_LOAD_REG (*((volatile uint32_t *)0xe000e014))
#define portNVIC_SYSTICK_CURRENT_VALUE_REG (*((volatile uint32_t *)0xe000e018))
#define portNVIC_SYSPRI2_REG (*((volatile uint32_t *)0xe000ed20))
#define portNVIC_SYSTICK_CLK_BIT (1UL << 2UL)
#define portNVIC_SYSTICK_INT_BIT (1UL << 1UL)
#define portNVIC_SYSTICK_ENABLE_BIT (1UL << 0UL)
#define portNVIC_PENDSV_PRI (((uint32_t)configKERNEL_INTERRUPT_PRIORITY) << 16UL)
#define portNVIC_SYSTICK_PRI (((uint32_t)configKERNEL_INTERRUPT_PRIORITY) << 24UL)
#define portINITIAL_XPSR 0x01000000UL
#define portINITIAL_EXC_RETURN 0xfffffffdUL
#define portSTART_ADDRESS_MASK ((StackType_t)0xfffffffeUL)
#define portFPCCR ((volatile uint32_t *)0xe000ef34)
#define portASPEN_AND_LSPEN_BITS (0x3UL << 30UL)

static UBaseType_t uxCriticalNesting = 0xaaaaaaaaUL;

void vPortSetupTimerInterrupt(void);

static void prvTaskExitError(void)
{
    configASSERT(uxCriticalNesting == ~0UL);
    portDISABLE_INTERRUPTS();
    for (;;) {}
}

StackType_t *pxPortInitialiseStack(StackType_t *top, TaskFunction_t code, void *parameters)
{
    top--;
    *top = portINITIAL_XPSR;
    top--;
    *top = ((StackType_t)code) & portSTART_ADDRESS_MASK;
    top--;
    *top = (StackType_t)prvTaskExitError;
    top -= 5;
    *top = (StackType_t)parameters;
    top--;
    *top = portINITIAL_EXC_RETURN;
    top -= 8;
    return top;
}

void vPortSVCHandler(void) __attribute__((naked));
void vPortSVCHandler(void)
{
    __asm volatile(
        "ldr r3, =pxCurrentTCB\n"
        "ldr r1, [r3]\n"
        "ldr r0, [r1]\n"
        "ldmia r0!, {r4-r11, r14}\n"
        "msr psp, r0\n"
        "isb\n"
        "mov r0, #0\n"
        "msr basepri, r0\n"
        "bx r14\n");
}

static void prvPortStartFirstTask(void) __attribute__((naked));
static void prvPortStartFirstTask(void)
{
    __asm volatile(
        "ldr r0, =0xE000ED08\n"
        "ldr r0, [r0]\n"
        "ldr r0, [r0]\n"
        "msr msp, r0\n"
        "mov r0, #0\n"
        "msr control, r0\n"
        "cpsie i\n"
        "cpsie f\n"
        "dsb\n"
        "isb\n"
        "svc 0\n"
        "nop\n");
}

static void vPortEnableVFP(void) __attribute__((naked));
static void vPortEnableVFP(void)
{
    __asm volatile(
        "ldr.w r0, =0xE000ED88\n"
        "ldr r1, [r0]\n"
        "orr r1, r1, #(0xf << 20)\n"
        "str r1, [r0]\n"
        "bx r14\n");
}

BaseType_t xPortStartScheduler(void)
{
    configASSERT(configMAX_SYSCALL_INTERRUPT_PRIORITY != 0);
    portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI | portNVIC_SYSTICK_PRI;
    vPortSetupTimerInterrupt();
    uxCriticalNesting = 0;
    vPortEnableVFP();
    *portFPCCR |= portASPEN_AND_LSPEN_BITS;
    prvPortStartFirstTask();
    return 0;
}

void vPortEndScheduler(void)
{
    configASSERT(uxCriticalNesting == 1000UL);
}

void vPortEnterCritical(void)
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
    if (uxCriticalNesting == 1)
        configASSERT((portNVIC_INT_CTRL_REG & 0xffUL) == 0);
}

void vPortExitCritical(void)
{
    configASSERT(uxCriticalNesting != 0);
    if (--uxCriticalNesting == 0)
        portENABLE_INTERRUPTS();
}

void xPortPendSVHandler(void) __attribute__((naked));
void xPortPendSVHandler(void)
{
    __asm volatile(
        "mrs r0, psp\n"
        "isb\n"
        "ldr r3, =pxCurrentTCB\n"
        "ldr r2, [r3]\n"
        "tst r14, #0x10\n"
        "it eq\n"
        "vstmdbeq r0!, {s16-s31}\n"
        "stmdb r0!, {r4-r11, r14}\n"
        "str r0, [r2]\n"
        "stmdb sp!, {r0, r3}\n"
        "mov r0, %0\n"
        "msr basepri, r0\n"
        "dsb\n"
        "isb\n"
        "bl vTaskSwitchContext\n"
        "mov r0, #0\n"
        "msr basepri, r0\n"
        "ldmia sp!, {r0, r3}\n"
        "ldr r1, [r3]\n"
        "ldr r0, [r1]\n"
        "ldmia r0!, {r4-r11, r14}\n"
        "tst r14, #0x10\n"
        "it eq\n"
        "vldmiaeq r0!, {s16-s31}\n"
        "msr psp, r0\n"
        "isb\n"
        "bx r14\n"
        :: "i"(configMAX_SYSCALL_INTERRUPT_PRIORITY) : "memory");
}

void xPortSysTickHandler(void)
{
    portDISABLE_INTERRUPTS();
    if (xTaskIncrementTick() != pdFALSE)
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
    portENABLE_INTERRUPTS();
}

__attribute__((weak)) void vPortSetupTimerInterrupt(void)
{
    portNVIC_SYSTICK_CTRL_REG = 0;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;
    portNVIC_SYSTICK_LOAD_REG = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT |
                               portNVIC_SYSTICK_INT_BIT |
                               portNVIC_SYSTICK_ENABLE_BIT;
}

void vPortValidateInterruptPriority(void)
{
    uint32_t ipsr;
    __asm volatile("mrs %0, ipsr" : "=r"(ipsr));
    if (ipsr >= 16UL) {
        const volatile uint8_t *priorities = (const volatile uint8_t *)0xe000e3f0;
        configASSERT(priorities[ipsr] >= configMAX_SYSCALL_INTERRUPT_PRIORITY);
    }
}
