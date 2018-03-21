/*****************************************************************************
 *
 * Copyright (c) 2017 mCube, Inc.  All rights reserved.
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
 * @file    mc5010.h
 * @author  mCube
 * @date    5 March 2018
 * @brief   Driver interface header file for gyroscope mc5010.
 * @see     http://www.mcubemems.com
 */

#ifndef _MC5010_H_
#define _MC5010_H_

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#include "Arduino.h"

// !!! DO NOT use both I2C and SPI simutaneously
#define MC5010_CFG_BUS_I2C
//#define MC5010_CFG_BUS_SPI

/*
SS  : pin 10, Active-low CS¡Xchip select
MOSI: pin 11, MOSI¡Xmaster out slave in
MISO: pin 12, MISO¡Xmaster in slave out
SCK : pin 13, SCK - SPI clock
*/

#if (!defined (MC5010_CFG_BUS_SPI) && !defined (MC5010_CFG_BUS_I2C))
  #error "At least one bus should be set to access register!"
#endif

#if (defined (MC5010_CFG_BUS_SPI) && defined (MC5010_CFG_BUS_I2C))
  #error "DO NOT use both SPI and I2C simutaneously!"
#endif

#ifdef MC5010_CFG_BUS_I2C
  #include <Wire.h>
#else
  #include <SPI.h>
  const int chipSelectPin = 10;
#endif

#define MC5010_REG_MAP_SIZE    191

#define FIFO_ENABLE
//Set 0 to chips below will initial sensors without setting OTP.
//And set 0x0D, 0x0E, 0x0F and register for FIFO only.

static uint8_t CHIP_R2B_REG[] =
{0x0D	,0x76	,0x76	,0x75	,0x21	,0x1E	,0x1F	,0x50	,0x51	,0x21	,
 0x22	,0x24	,0x24	,0x28	,0x28	,0x2D	,0x2D	,0x5E	,0x1F	,0x65	,
 0x66	,0x20	,0x50	,0x66	,0x25	,0x29	,0x2E	,0x52	,0x53	,0x54	,
 0x0E	,0x0F	,0x23	,0x26	,0x5B	,0x62	,0x27	,0x2A	,0x5C	,0x63	,
 0x2C	,0x2F	,0x64	,0x50	,0x5D	,0x1B	,0x74	,0x31	,0x31	,0x31	,
 0x6B	,0x31	,0x32	,0x33	,0x34	,0x35	,0x36	,0x37	,0x69	,0x0D	,
 0x46	,0x47	,0x4E	,0x4F	,0x2B}; 
static uint8_t CHIP_R2B_VAL[] =
{0x07	,0x6D	,0x43	,0x02	,0x24	,0x07	,0x81	,0x00	,0x00	,0x04	,
 0x08	,0x00	,0x01	,0x00	,0x01	,0x00	,0x01	,0x07	,0x00	,0x3A	,
 0x00	,0x05	,0x10	,0x70	,0x05	,0x05	,0x05	,0x00	,0x00	,0x00	,
 0x00	,0x8A	,0x01	,0x00	,0x80	,0x00	,0x01	,0x00	,0x80	,0x00	,
 0x01	,0x00	,0x00	,0x10	,0x80	,0x00	,0x00	,0x00	,0x02	,0x00	,
 0x08	,0x01	,0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,0x05	,
 0x00	,0x00	,0xF6	,0x01	,0x00};

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
 /* MC5010 axis definition. */
typedef enum
{
    MC5010_AXIS_X   = 0,
    MC5010_AXIS_Y   = 1,
    MC5010_AXIS_Z   = 2,
    MC5010_AXIS_NUM = 3,
} MC5010_axis_type;

/* Available I2C address for MC5010 sensor */
typedef enum
{
    MC5010_ADDR_5A = 0x5A,
    MC5010_ADDR_5B = 0x5B,
} MC5010_i2c_addr_type;

/* Available mode in MC5010 sensor. REG:0x0D[1:0] */
typedef enum
{
    MC5010_MODE_SLEEP    = 0x00,
    MC5010_MODE_WAKE,
    MC5010_MODE_RESERVED,			//Reserve, no use
    MC5010_MODE_STANDBY,
    MC5010_MODE_END,
} MC5010_mode_type;

/* Sensor sampling rate (corner frequency). REG:0x0E[3:0] */
typedef enum
{
    MC5010_RATE_MAX   = 0x00,	//ODR 2kHz and all pass through for corner frequency
    MC5010_RATE_1kHz  = 0x01,	//ODR 2kHz and 1kHz corner frequency
    MC5010_RATE_512Hz = 0x02,	//ODR 1kHz and 512Hz corner frequency
    MC5010_RATE_256Hz = 0x03,	//ODR 512Hz and 256Hz corner frequency
    MC5010_RATE_128Hz = 0x04,	//ODR 256Hz and 128Hz corner frequency
    MC5010_RATE_64Hz  = 0x05,	//ODR 128Hz and 64Hz corner frequency
    MC5010_RATE_END,
} MC5010_rate_type;

/* Sensor dps scaler. REG:0x0E[7:4] */
typedef enum
{
    MC5010_DPS_2kdps  = 0x00,	// DPS = +/- 2k
    MC5010_DPS_1kdps  = 0x10,	// DPS = +/- 1k
    MC5010_DPS_500dps = 0x20,	// DPS = +/- 500
    MC5010_DPS_250dps = 0x30,	// DPS = +/- 250
    MC5010_DPS_125dps = 0x40,	// DPS = +/- 125
    MC5010_DPS_8kdps  = 0x50,	// DPS = +/- 8k
    MC5010_DPS_END,
} MC5010_dps_type;

/* LPF BW. REG:0x0F[3:0] */
typedef enum
{
    MC5010_LPF_DISABLE = 0x00,	//LPF DISABLE
    MC5010_LPF_128Hz   = 0x01,	//BW1 ~ 128Hz
    MC5010_LPF_64Hz    = 0x02,	//BW2 ~  64Hz
    MC5010_LPF_32Hz    = 0x03,	//BW3 ~  32Hz
    MC5010_LPF_16Hz	   = 0x04,	//BW4 ~  16Hz
    MC5010_LPF_8Hz	   = 0x05,	//BW5 ~   8Hz
    MC5010_LPF_ENABLE  = 0x08,	//LPF ENABLE
    MC5010_LPF_END,
} MC5010_lpf_mode_type;

/* HPF BW. REG:0x0F[7:4] */
typedef enum
{
    MC5010_HPF_DISABLE = 0x00,	//HPF DISABLE
    MC5010_HPF_32Hz    = 0x10,	//BW1 ~  32Hz
    MC5010_HPF_16Hz	   = 0x20,	//BW4 ~  16Hz
    MC5010_HPF_8Hz     = 0x30,	//BW1 ~   8Hz
    MC5010_HPF_4Hz     = 0x40,	//BW1 ~   4Hz
    MC5010_HPF_1Hz     = 0x50,	//BW1 ~   1Hz
    MC5010_HPF_0p5Hz   = 0x60,	//BW1 ~ 0.5Hz
    MC5010_HPF_0p1Hz   = 0x70,	//BW1 ~ 0.1Hz
    MC5010_HPF_ENABLE  = 0x80,	//HPF ENABLE
    MC5010_HPF_END,
} MC5010_hpf_mode_type;

/* FIFO controller. REG:0x11[7:0] */
typedef enum
{
    MC5010_FIFO_DISABLE           = 0x00,
    MC5010_FIFO_ENABLE            = 0x01,
    MC5010_FIFO_RD_STOP_ENABLE    = 0x11,
    MC5010_FIFO_WR_STOP_ENABLE    = 0x21,
    MC5010_FIFO_RD_WR_STOP_ENABLE = 0x31,
    MC5010_FIFO_END,
} MC5010_fifo_type;

/* INT mode controller. REG:0x12[7:0] */
typedef enum
{
    MC5010_INT_MODE_IPP_OPEN_DRAIN_IAH_LOW      = 0x00,	//External pullup to VDD for INT and active low
    MC5010_INT_MODE_IPP_MODE_PUSH_PULL_IAH_LOW  = 0x01,	//Logic high drive level is VDD for INT and active low
    MC5010_INT_MODE_IPP_OPEN_DRAIN_IAH_HIGH     = 0x02,	//External pullup to VDD for INT and active high
    MC5010_INT_MODE_IPP_MODE_PUSH_PULL_IAH_HIGH = 0x03,	//Logic high drive level is VDD for INT and active high
    MC5010_INT_MODE_LATCH                       = 0x40,	//Latch mode
    MC5010_INT_MODE_END,
} MC5010_int_mode_type;

/* INT controller. REG:0x13[7:0] */
typedef enum
{
    MC5010_INT_DIS   = 0x00,
    MC5010_INT_XL_EN = 0x01,
    MC5010_INT_XH_EN = 0x02,
    MC5010_INT_YL_EN = 0x04,
    MC5010_INT_YH_EN = 0x08,
    MC5010_INT_ZL_EN = 0x10,
    MC5010_INT_ZH_EN = 0x20,
    MC5010_INT_FIFO  = 0x40,
    MC5010_INT_ACQ	 = 0x80,
    MC5010_INT_END	 = 0x00,
} MC5010_int_type;

typedef struct
{
    short XAxis;
    short YAxis;
    short ZAxis;
    float XAxis_f;
    float YAxis_f;
    float ZAxis_f;
} MC5010_acc_t;

typedef struct
{
    unsigned char    bACQ_INT;
    unsigned char    bFIFO_INT;
    unsigned char    bZH_INT;
    unsigned char    bZL_INT;
    unsigned char    bYH_INT;
    unsigned char    bYL_INT;
    unsigned char    bXH_INT;
    unsigned char    bXL_INT;
} MC5010_interruptevent_t;

typedef enum
{
    GYRO_REG_WHO_AM_I        = 0X00,
    GYRO_REG_DEV_STAT        = 0X01,
    GYRO_REG_XOUT_L          = 0X02,
    GYRO_REG_XOUT_H          = 0X03,
    GYRO_REG_YOUT_L          = 0X04,
    GYRO_REG_YOUT_H          = 0X05,
    GYRO_REG_ZOUT_L          = 0X06,
    GYRO_REG_ZOUT_H          = 0X07,
    GYRO_REG_TOUT_L          = 0X08,
    GYRO_REG_TOUT_H          = 0X09,
    GYRO_REG_STATUS          = 0X0A,
    GYRO_REG_INT_STAT        = 0X0B,
    GYRO_REG_FIFO_STAT       = 0X0C,
    GYRO_REG_MODE            = 0X0D,
    GYRO_REG_SCALE_RATE_CTRL = 0X0E,
    GYRO_REG_FILTER_CTRL     = 0X0F,
    GYRO_REG_I2C_AUX_CTRL    = 0X10,
    GYRO_REG_FIFO_CTRL       = 0X11,
    GYRO_REG_INT_CTRL        = 0X12,
    GYRO_REG_INT_EN          = 0X13,
    GYRO_REG_INT_XTH_L       = 0X14,
    GYRO_REG_INT_XTH_H       = 0X15,
    GYRO_REG_INT_YTH_L       = 0X16,
    GYRO_REG_INT_YTH_H       = 0X17,
    GYRO_REG_INT_ZTH_L       = 0X18,
    GYRO_REG_INT_ZTH_H       = 0X19,

    GYRO_REG_DEVICEID_X      = 0X6C,
    GYRO_REG_DEVICEID_Y      = 0X6D,

    GYRO_REG_CHIPID          = 0X73,
    GYRO_REG_INTRL_SEL       = 0X74,
} MC5010_reg_type;

typedef struct
{
    float xOffset;
    float yOffset;
    float zOffset;
} mCubeGyroOffset_t;

typedef struct
{
    int xGain;
    int yGain;
   	int zGain;
} mCubeGyroGain_t;

typedef struct
{
    mCubeGyroOffset_t offsets;
   	mCubeGyroGain_t gains;
} mCubeGyroOffsetGain_t;

/*******************************************************************************
 *** EXTERNAL FUNCTION
 *******************************************************************************/
class MC5010{
 public:
    int	init(void);
	int setRate(MC5010_rate_type eCfgSR);
	int setDPS(MC5010_dps_type eCfgDPS);
	int getRate();
	int getDPS();
    MC5010_acc_t readSensorData();

 private:
    const int chipSelectPin = 10;

    void delay(uint32_t dwMs);
    void writeRegister8(uint8_t reg, uint8_t *data);
    void readRegisters(uint8_t reg, byte *buffer, uint8_t len);
	int	reset(void);
	int setMode(MC5010_mode_type eNextMode);
    int configmode(MC5010_mode_type eNextMode);
	int	configModeINT(int eCfgINTmode);
    int configINT(int eCfgINT);
    int	configFilter(int eCfgLPF, int eCfgHPF);
    int	configFIFO(MC5010_fifo_type eCtrl);
};

#endif /* _MC5010_H_ */
