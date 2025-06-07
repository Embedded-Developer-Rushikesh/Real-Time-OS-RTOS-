This project demonstrates a bare-metal Round Robin task scheduler implemented from scratch on an STM32F4 ARM Cortex-M microcontroller, 
providing a lightweight alternative to commercial RTOS solutions like FreeRTOS. The core architecture revolves around a Task Control Block (TCB) structure 
that manages each task's stack pointer (PSP), execution state (ready/blocked), and entry point.

The scheduler supports five concurrent tasks (including an idle task) with preemptive time-slicing driven by the SysTick timer at a configurable rate (TICK_HZ).
Key innovations include manual context switching via the PendSV handler, which efficiently saves and restores R4-R11 registers during task transitions, and 
a stack initialization routine that pre-configures each task's stack frame with proper exception return behavior (0xFFFFFFFD in LR for thread mode with PSP).
The implementation showcases critical RTOS concepts: task states (ready/blocked) are managed through task_delay() which suspends tasks for specified tick periods,
while the scheduler's update_next_task() implements round-robin prioritization. Processor fault handlers (HardFault, MemManage, BusFault) are instrumented for debugging stack overflows or memory violations. 
The switch_sp_to_psp() function transitions the kernel from MSP to PSP-based operation, enabling per-task stack isolation. A debug structure tracks task execution counts for performance monitoring.
Notable ARM-specific features include naked functions for precise assembly control, the use of SVC exceptions for system calls, and careful management of the CONTROL register for stack pointer selection.
The design minimizes latency by deferring context switches to PendSV (tail-chained to SysTick), while the unblock_tasks() mechanism handles timed wakeups. 
This educational implementation reveals the inner workings of commercial RTOS schedulers, particularly in stack management (initializing XPSR, PC, and LR in each task's stack) and exception handling.
The code serves as both a functional scheduler for resource-constrained devices and a foundation for extending with features like semaphores or priority inheritance,
demonstrating how ARM's dual-stack architecture (MSP/PSP) enables efficient preemptive multitasking.
