#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

#define portCHAR char
#define portFLOAT float
#define portDOUBLE double
#define portLONG long
#define portSHORT short
#define portSTACK_TYPE uint32_t
#define portBASE_TYPE long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if (configUSE_16_BIT_TICKS == 1)
typedef uint16_t TickType_t;
#define portMAX_DELAY ((TickType_t)0xffff)
#else
typedef uint32_t TickType_t;
#define portMAX_DELAY ((TickType_t)0xffffffffUL)
#define portTICK_TYPE_IS_ATOMIC 1
#endif

#define portSTACK_GROWTH (-1)
#define portTICK_PERIOD_MS ((TickType_t)1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT 8

#define portNVIC_INT_CTRL_REG (*((volatile uint32_t *)0xe000ed04))
#define portNVIC_PENDSVSET_BIT (1UL << 28UL)

#define portYIELD() do { \
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT; \
    __asm volatile("dsb" ::: "memory"); \
    __asm volatile("isb"); \
} while (0)

#define portEND_SWITCHING_ISR(x) do { if ((x) != pdFALSE) portYIELD(); } while (0)
#define portYIELD_FROM_ISR(x) portEND_SWITCHING_ISR(x)

static inline __attribute__((always_inline)) void vPortSetBASEPRI(uint32_t value)
{
    __asm volatile("msr basepri, %0" :: "r"(value) : "memory");
}

static inline __attribute__((always_inline)) uint32_t ulPortRaiseBASEPRI(void)
{
    uint32_t original;
    uint32_t value = configMAX_SYSCALL_INTERRUPT_PRIORITY;
    __asm volatile(
        "mrs %0, basepri\n"
        "msr basepri, %1\n"
        "dsb\n"
        "isb\n"
        : "=r"(original) : "r"(value) : "memory");
    return original;
}

static inline __attribute__((always_inline)) BaseType_t xPortIsInsideInterrupt(void)
{
    uint32_t ipsr;
    __asm volatile("mrs %0, ipsr" : "=r"(ipsr) :: "memory");
    return ipsr == 0U ? pdFALSE : pdTRUE;
}

extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);
#define portSET_INTERRUPT_MASK_FROM_ISR() ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) vPortSetBASEPRI(x)
#define portDISABLE_INTERRUPTS() ((void)ulPortRaiseBASEPRI())
#define portENABLE_INTERRUPTS() vPortSetBASEPRI(0)
#define portENTER_CRITICAL() vPortEnterCritical()
#define portEXIT_CRITICAL() vPortExitCritical()

#define portTASK_FUNCTION_PROTO(fn, args) void fn(void *args)
#define portTASK_FUNCTION(fn, args) void fn(void *args)

#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1
#if configMAX_PRIORITIES > 32
#error configMAX_PRIORITIES must not exceed 32
#endif
#define portRECORD_READY_PRIORITY(priority, ready) ((ready) |= (1UL << (priority)))
#define portRESET_READY_PRIORITY(priority, ready) ((ready) &= ~(1UL << (priority)))
#define portGET_HIGHEST_PRIORITY(top, ready) ((top) = 31UL - (uint32_t)__builtin_clz(ready))
#endif

#ifdef configASSERT
void vPortValidateInterruptPriority(void);
#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID() vPortValidateInterruptPriority()
#endif

#define portNOP() __asm volatile("nop")
#define portINLINE __inline
#define portFORCE_INLINE inline __attribute__((always_inline))

#ifdef __cplusplus
}
#endif

#endif
