#include "FreeRTOS.h"
#include <semphr.h>

#pragma once
inline SemaphoreHandle_t i2cMutex = nullptr;
