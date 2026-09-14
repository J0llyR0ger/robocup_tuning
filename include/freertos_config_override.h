/**
 * @file    freertos_config_override.h
 * @brief   Project-specific FreeRTOS configuration overrides.
 *
 * Included automatically at the end of freertos-teensy's FreeRTOSConfig.h
 * (made visible to the library build via `-Iinclude` in platformio.ini).
 */

#pragma once

/* Place the heap in RAM2/OCRAM (after bss.dma) instead of the default DTCM
 * region, which is too small for our combined task stack allocations. */
#undef configTEENSY_HEAP_ALLOCATION
#define configTEENSY_HEAP_ALLOCATION 2

/* Total FreeRTOS heap size, in bytes. */
#undef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE ((size_t)(128 * 1024))
