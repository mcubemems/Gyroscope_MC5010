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
 * @file    Utility.c
 * @author  mCube
 * @date    5 Febuary 2018
 * @brief   Utility variable for mCube sensors.
 * @see     http://www.mcubemems.com
 */

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/
#include "Utility.h"

/*******************************************************************************
 *** CONFIGURATION
 *******************************************************************************/

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/

/*******************************************************************************
 *** GLOBAL VARIABLE
 *******************************************************************************/
const MC_UTIL_ORIENTATIONREMAP g_MDrvUtilOrientationReMap[UTIL_ORIENTATION_TOTAL_CONFIG] =
              {
                  {{  1,  1,  1 }, { MC_UTL_AXIS_X, MC_UTL_AXIS_Y, MC_UTL_AXIS_Z }},
                  {{ -1,  1,  1 }, { MC_UTL_AXIS_Y, MC_UTL_AXIS_X, MC_UTL_AXIS_Z }},
                  {{ -1, -1,  1 }, { MC_UTL_AXIS_X, MC_UTL_AXIS_Y, MC_UTL_AXIS_Z }},
                  {{  1, -1,  1 }, { MC_UTL_AXIS_Y, MC_UTL_AXIS_X, MC_UTL_AXIS_Z }},
                  
                  {{ -1,  1, -1 }, { MC_UTL_AXIS_X, MC_UTL_AXIS_Y, MC_UTL_AXIS_Z }},
                  {{  1,  1, -1 }, { MC_UTL_AXIS_Y, MC_UTL_AXIS_X, MC_UTL_AXIS_Z }},
                  {{  1, -1, -1 }, { MC_UTL_AXIS_X, MC_UTL_AXIS_Y, MC_UTL_AXIS_Z }},
                  {{ -1, -1, -1 }, { MC_UTL_AXIS_Y, MC_UTL_AXIS_X, MC_UTL_AXIS_Z }},
              };


