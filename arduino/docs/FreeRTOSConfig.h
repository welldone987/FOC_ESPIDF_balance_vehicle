/*
 * FreeRTOS configuration audit snapshot for balancing_vehicle_work.
 *
 * Target: WEMOS LOLIN32 Lite (ESP32, dual-core Xtensa)
 * Framework: Arduino-ESP32 2.0.17
 * ESP-IDF: v4.4.7 (Arduino package commit 38eeba213a)
 * PlatformIO package: framework-arduinoespressif32 3.20017.241212
 * Audit date: 2026-08-18
 *
 * IMPORTANT:
 * This file is documentation.  It records the effective FreeRTOS settings
 * found in the locally installed precompiled Arduino-ESP32 SDK.  It is not an
 * application override and must not be copied into balancing_vehicle_work's
 * include directory or used to replace the framework FreeRTOSConfig.h.
 *
 * Local sources of truth:
 *   1. tools/sdk/esp32/sdkconfig
 *   2. tools/sdk/esp32/include/freertos/include/esp_additions/freertos/
 *      FreeRTOSConfig.h
 *   3. tools/sdk/esp32/include/freertos/port/xtensa/include/freertos/
 *      FreeRTOSConfig_arch.h
 *   4. Preprocessor macro dump generated with the active PlatformIO compiler,
 *      definitions, and include paths from .pio/build/lolin32_lite/idedata.json
 *
 * Interpretation references:
 *   https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/02-Customization
 *   https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/09-Memory-management/01-Memory-management
 *   https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/system/freertos.html
 */

#ifndef BALANCING_VEHICLE_FREERTOS_CONFIG_AUDIT_H
#define BALANCING_VEHICLE_FREERTOS_CONFIG_AUDIT_H

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

/*-----------------------------------------------------------
 * Architecture and port configuration
 *----------------------------------------------------------*/

#define portNUM_PROCESSORS                              2
#define portUSING_MPU_WRAPPERS                          0

#define configNUM_CORES                                 2
#define configXT_BOARD                                  1
#define configXT_SIMULATOR                              0
#define configSTACK_ALIGNMENT                           16
#define configKERNEL_INTERRUPT_PRIORITY                 1
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            3
#define configISR_STACK_SIZE                            2096

/* Xtensa port library and timer selection. */
#define XT_USE_THREAD_SAFE_CLIB                         0
#define XT_TIMER_INDEX                                  0

/*-----------------------------------------------------------
 * Scheduler behavior
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                            1
#define configUSE_TIME_SLICING                          1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION         0
#define configUSE_16_BIT_TICKS                          0
#define configTICK_RATE_HZ                              1000
#define configMAX_PRIORITIES                            25
#define configIDLE_SHOULD_YIELD                         0
#define configUSE_TICKLESS_IDLE                         0

/* This value is inactive while configUSE_TICKLESS_IDLE is 0. */
#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP           2

/*-----------------------------------------------------------
 * Task names, stacks, and task-local storage
 *
 * ESP-IDF's FreeRTOS APIs use bytes for task stack sizes.  This differs from
 * upstream FreeRTOS ports that commonly express stack depth in StackType_t
 * elements.
 *----------------------------------------------------------*/

#define configMAX_TASK_NAME_LEN                         16
#define configSTACK_OVERHEAD_CHECKER                    0
#define configSTACK_OVERHEAD_OPTIMIZATION               0
#define configSTACK_OVERHEAD_APPTRACE                   0
#define configSTACK_OVERHEAD_WATCHPOINT                 60
#define configSTACK_OVERHEAD_TOTAL                      60
#define configMINIMAL_STACK_SIZE                        828
#define configIDLE_TASK_STACK_SIZE                      1024
#define configSTACK_DEPTH_TYPE                          uint16_t
#define configRECORD_STACK_HIGH_ADDRESS                 1

#define configNUM_THREAD_LOCAL_STORAGE_POINTERS         1
#define configTHREAD_LOCAL_STORAGE_DELETE_CALLBACKS     1

/*-----------------------------------------------------------
 * Memory allocation
 *
 * ESP-IDF uses its capability-aware multi-heap allocator.  The heap boundary
 * expression below is the framework definition, not a recommendation to add a
 * standalone heap_x.c implementation to the Arduino project.
 *----------------------------------------------------------*/

#define configSUPPORT_DYNAMIC_ALLOCATION                1
#define configSUPPORT_STATIC_ALLOCATION                 1
#define configAPPLICATION_ALLOCATED_HEAP                1
#define configTOTAL_HEAP_SIZE                           (&_heap_end - &_heap_start)
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP       0

/* Effective FreeRTOS default: no vApplicationMallocFailedHook callback. */
#define configUSE_MALLOC_FAILED_HOOK                    0

/*-----------------------------------------------------------
 * Hooks, assertions, and diagnostic facilities
 *----------------------------------------------------------*/

#define configUSE_IDLE_HOOK                             1
#define configUSE_TICK_HOOK                             1
#define configUSE_DAEMON_TASK_STARTUP_HOOK              0

/* CONFIG_FREERTOS_ASSERT_FAIL_ABORT=y in the installed SDK. */
#define configASSERT(a)                                 assert(a)

/* Canary-based stack checking is enabled. */
#define configCHECK_FOR_STACK_OVERFLOW                  2

/* Runtime task tables and CPU-time accumulation are disabled. */
#define configUSE_TRACE_FACILITY                        0
#define configUSE_STATS_FORMATTING_FUNCTIONS            0
#define configGENERATE_RUN_TIME_STATS                   0

#define configQUEUE_REGISTRY_SIZE                       0
#define configENABLE_TASK_SNAPSHOT                      1
#define configCHECK_MUTEX_GIVEN_BY_OWNER                1
#define configBENCHMARK                                 0

/*-----------------------------------------------------------
 * Synchronization and inter-task communication
 *----------------------------------------------------------*/

#define configUSE_MUTEX                                 1
#define configUSE_MUTEXES                               1
#define configUSE_RECURSIVE_MUTEXES                     1
#define configUSE_COUNTING_SEMAPHORES                   1
#define configUSE_QUEUE_SETS                            1
#define configUSE_TASK_NOTIFICATIONS                    1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES           1
#define configMESSAGE_BUFFER_LENGTH_TYPE                size_t

/*-----------------------------------------------------------
 * Software timers
 *----------------------------------------------------------*/

#define configUSE_TIMERS                                1
#define configTIMER_TASK_PRIORITY                       1
#define configTIMER_QUEUE_LENGTH                        10
#define configTIMER_TASK_STACK_DEPTH                    2048

/*-----------------------------------------------------------
 * C library, co-routines, and compatibility
 *----------------------------------------------------------*/

#define configUSE_NEWLIB_REENTRANT                      1
#define configUSE_CO_ROUTINES                           0
#define configMAX_CO_ROUTINE_PRIORITIES                 2
#define configUSE_APPLICATION_TASK_TAG                  0
#define configUSE_POSIX_ERRNO                           0
#define configUSE_ALTERNATIVE_API                       0
#define configENABLE_BACKWARD_COMPATIBILITY             1

/*-----------------------------------------------------------
 * Optional API inclusion
 *----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet                        1
#define INCLUDE_uxTaskPriorityGet                       1
#define INCLUDE_vTaskDelete                             1
#define INCLUDE_vTaskCleanUpResources                   0
#define INCLUDE_vTaskSuspend                            1
#define INCLUDE_vTaskDelayUntil                         1
#define INCLUDE_vTaskDelay                              1
#define INCLUDE_uxTaskGetStackHighWaterMark             1
#define INCLUDE_uxTaskGetStackHighWaterMark2            0
#define INCLUDE_pcTaskGetTaskName                       1
#define INCLUDE_xTaskGetIdleTaskHandle                  1
#define INCLUDE_pxTaskGetStackStart                     1
#define INCLUDE_eTaskGetState                           1
#define INCLUDE_xTaskAbortDelay                         1
#define INCLUDE_xTaskGetHandle                          1
#define INCLUDE_xTaskGetCurrentTaskHandle               0
#define INCLUDE_xTaskGetSchedulerState                  0
#define INCLUDE_xTaskResumeFromISR                      1
#define INCLUDE_xSemaphoreGetMutexHolder                1
#define INCLUDE_xQueueGetMutexHolder                    0
#define INCLUDE_xTimerPendFunctionCall                  1
#define INCLUDE_xTimerGetTimerDaemonTaskHandle          0

/* ESP-IDF task.c extension hook is enabled by the framework. */
#define configINCLUDE_FREERTOS_TASK_C_ADDITIONS_H       1

/*-----------------------------------------------------------
 * ESP-IDF Kconfig audit notes that are not represented solely by upstream
 * config* macros
 *
 * Enabled:
 *   CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY
 *   CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK
 *   CONFIG_FREERTOS_INTERRUPT_BACKTRACE
 *   CONFIG_FREERTOS_CHECK_MUTEX_GIVEN_BY_OWNER
 *   CONFIG_FREERTOS_DEBUG_OCDAWARE
 *   CONFIG_FREERTOS_FPU_IN_ISR
 *   CONFIG_FREERTOS_ENABLE_TASK_SNAPSHOT
 *   CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION
 *   CONFIG_HEAP_POISONING_LIGHT
 *
 * Disabled:
 *   CONFIG_FREERTOS_UNICORE
 *   CONFIG_FREERTOS_USE_TRACE_FACILITY
 *   CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
 *   CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP
 *   CONFIG_FREERTOS_CHECK_PORT_CRITICAL_COMPLIANCE
 *   CONFIG_HEAP_TASK_TRACKING
 *   CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS
 *
 * Consequences:
 *   - OpenOCD-aware task debugging support is present.
 *   - Stack canary and end-of-stack watchpoint protection are present.
 *   - vTaskList(), uxTaskGetSystemState(), and CPU percentage statistics are
 *     unavailable in this precompiled SDK because trace/runtime statistics are
 *     disabled.
 *   - vApplicationMallocFailedHook() is not enabled by the effective kernel
 *     configuration; application code must still check allocation results.
 *----------------------------------------------------------*/

#endif /* BALANCING_VEHICLE_FREERTOS_CONFIG_AUDIT_H */
