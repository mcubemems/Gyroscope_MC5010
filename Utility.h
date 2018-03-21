/*****************************************************************************
 *
 * Copyright (c) 2018 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

/**
 * @file    Utility.h
 * @author  mCube
 * @date    5 Febuary 2018
 * @brief   Utility header file for mCube sensors.
 * @see     http://www.mcubemems.com
 */

#ifndef _UTILITY_H_
    #define _UTILITY_H_

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define MC_UTL_AXIS_X      0
#define MC_UTL_AXIS_Y      1
#define MC_UTL_AXIS_Z      2
#define MC_UTL_AXES_NUM    3

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
/* mCube ratcodes definition. */
typedef enum
{
    MCUBE_RATCODE_SUCCESS              =  0,
    MCUBE_RATCODE_ERROR_BUS            = -1,
    MCUBE_RATCODE_ERROR_NULL_POINTER   = -2,
    MCUBE_RATCODE_ERROR_STATUS         = -3,
    MCUBE_RATCODE_ERROR_SETUP          = -4,
    MCUBE_RATCODE_ERROR_GET_DATA       = -5,
    MCUBE_RATCODE_ERROR_IDENTIFICATION = -6,
    MCUBE_RATCODE_ERROR_NO_DATA        = -7,
    MCUBE_RATCODE_ERROR_WRONG_ARGUMENT = -8,
} MCUBE_RATCODE;
 
typedef struct
{
    signed char      bSign[MC_UTL_AXES_NUM];
    unsigned char    bMap[MC_UTL_AXES_NUM];
} MC_UTIL_ORIENTATIONREMAP;

typedef enum
{
    UTIL_ORIENTATION_TOP_LEFT_DOWN      = 0,
    UTIL_ORIENTATION_TOP_RIGHT_DOWN,
    UTIL_ORIENTATION_TOP_RIGHT_UP,
    UTIL_ORIENTATION_TOP_LEFT_UP,
    UTIL_ORIENTATION_BOTTOM_LEFT_DOWN,
    UTIL_ORIENTATION_BOTTOM_RIGHT_DOWN,
    UTIL_ORIENTATION_BOTTOM_RIGHT_UP,
    UTIL_ORIENTATION_BOTTOM_LEFT_UP,
    UTIL_ORIENTATION_TOTAL_CONFIG
} UTIL_ORIENTATIONREMAPCFG;

extern const MC_UTIL_ORIENTATIONREMAP g_MDrvUtilOrientationReMap[UTIL_ORIENTATION_TOTAL_CONFIG];

/*******************************************************************************
 *** GLOBAL VARIABLE
 *******************************************************************************/
class Utility{

};

#endif    // END of _MC_UTILITY_H_

