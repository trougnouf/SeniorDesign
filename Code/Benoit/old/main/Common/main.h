#ifndef MAINH
#define MAINH

#ifdef __STM32F0xx_HAL_H
#include "F070.h"
#elif defined __STM32F7xx_HAL_H
#include "F746.h"
#endif
// hw init/pinoutxxxx.h
// common functions
// UART: strOut
// Laser: 3functions (include laser.h) (define BT, speed constants)
// GPS: readDirFromSD, outputDir
// BT: mp3toBT
// 
#endif
