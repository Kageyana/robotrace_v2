/******************************************************************************
 *  File        : sd_functions.h
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/

#ifndef __SD_FUNCTIONS_H__
#define __SD_FUNCTIONS_H__

#include "fatfs.h"
#include <stdint.h>

extern char sd_path[];

// Mount and unmount
int sd_mount(void);
int sd_unmount(void);
int sd_remount(void);

// Space information
int sd_get_space_kb(void);

#endif // __SD_FUNCTIONS_H__
