#include <stdio.h>
#include <stdint.h>

// define the MCU used as medium density device
# define STM32F10X_MD

// driver code is used instead of direct register access
# define USE_STDPERIPH_DRIVER

// using full assert
# define USE_FULL_ASSERT
#include "stm32f10x_conf.h"

