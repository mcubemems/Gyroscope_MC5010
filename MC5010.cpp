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
 * @file    mc5010.cpp
 * @author  mCube
 * @date    5 March 2018
 * @brief   Driver interface for gyroscope mc5010.
 * @see     http://www.mcubemems.com
 */

/*******************************************************************************
 *** INFORMATION
 *******************************************************************************/
#define MC5010_VERSION    "1.0.0_alpha"

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/
#include <MC5010.h>
#include <Utility.h>

/*******************************************************************************
 *** CONFIGURATION
 *******************************************************************************/

#define MC5010_CFG_I2C_ADDR    MC5010_ADDR_5A
const uint8_t WRITE = 0x40;
const uint8_t READ  = 0xC0;

#define MC5010_CFG_RATE               MC5010_RATE_MAX
#define MC5010_CFG_DPS                MC5010_DPS_2kdps
#define MC5010_CFG_LPF                (MC5010_lpf_mode_type) (MC5010_LPF_ENABLE | MC5010_LPF_64Hz)
#define MC5010_CFG_HPF                MC5010_HPF_DISABLE
#define MC5010_CFG_ORIENTATION_MAP    UTIL_ORIENTATION_BOTTOM_RIGHT_DOWN

#define MC5010_CFG_XTHRESHOLD    0
#define MC5010_CFG_YTHRESHOLD    0
#define MC5010_CFG_ZTHRESHOLD    0

#define MC5010_GAIN_16BIT    16384

/*******************************************************************************
 *** MACRO
 *******************************************************************************/
#define regWrite(bRegAddr, pbDataBuf, bLength)    writeRegister8(bRegAddr, pbDataBuf)
#define regRead(bRegAddr, pbDataBuf, bLength)     readRegisters(bRegAddr, pbDataBuf, bLength)

#define MC5010_REG_STATUS_MODE(bRegStatus)    (bRegStatus & 0x01)

#define MC5010_REG_INTSTATUS_ACQ_INT(bRegINTStatus)     ((bRegINTStatus >> 7) & 0x01)
#define MC5010_REG_INTSTATUS_FIFO_INT(bRegINTStatus)    ((bRegINTStatus >> 6) & 0x01)
#define MC5010_REG_INTSTATUS_ZH_INT(bRegINTStatus)      ((bRegINTStatus >> 5) & 0x01)
#define MC5010_REG_INTSTATUS_ZL_INT(bRegINTStatus)      ((bRegINTStatus >> 4) & 0x01)
#define MC5010_REG_INTSTATUS_YH_INT(bRegINTStatus)      ((bRegINTStatus >> 3) & 0x01)
#define MC5010_REG_INTSTATUS_YL_INT(bRegINTStatus)      ((bRegINTStatus >> 2) & 0x01)
#define MC5010_REG_INTSTATUS_XH_INT(bRegINTStatus)      ((bRegINTStatus >> 1) & 0x01)
#define MC5010_REG_INTSTATUS_XL_INT(bRegINTStatus)      ((bRegINTStatus) 	  & 0x01)

/*******************************************************************************
 *** STATIC VARIABLES
 *******************************************************************************/
static MC5010_rate_type     s_eRate = MC5010_CFG_RATE;
static MC5010_dps_type      s_eDPS  = MC5010_CFG_DPS;
static MC5010_lpf_mode_type	s_eLPF  = MC5010_CFG_LPF;
static MC5010_hpf_mode_type	s_eHPF  = MC5010_CFG_HPF;

static uint16_t	s_bCfgXThr = MC5010_CFG_XTHRESHOLD;
static uint16_t	s_bCfgYThr = MC5010_CFG_YTHRESHOLD;
static uint16_t	s_bCfgZThr = MC5010_CFG_ZTHRESHOLD;

/*******************************************************************************
 *** STATIC FUNCTION
 *******************************************************************************/
 /*********************************************************************
 *** delay
 *********************************************************************/
void MC5010::delay(uint32_t dwMs)
{
  delayMicroseconds(dwMs);
}
 
/*********************************************************************
 *** WriteRegister8
 *** Write 8-bit to register
 *********************************************************************/
void MC5010::writeRegister8(uint8_t reg, uint8_t *data)
{
#ifdef MC5010_CFG_BUS_I2C
  Wire.beginTransmission(MC5010_CFG_I2C_ADDR);
  Wire.write(reg);
  Wire.write(*data);
  Wire.endTransmission();
#else  //SPI interface
  uint8_t regbuffer = reg | WRITE;
  digitalWrite(chipSelectPin,LOW);
  SPI.transfer(regbuffer);
  SPI.transfer(*data);
  digitalWrite(chipSelectPin,HIGH);
#endif
}

/*********************************************************************
 *** ReadRegisters
 *** Repeated Read Byte(s) from register
 *********************************************************************/
void MC5010::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
{
#ifdef MC5010_CFG_BUS_I2C
  Wire.beginTransmission(MC5010_CFG_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MC5010_CFG_I2C_ADDR, (int) len);
  
  while(Wire.available() < len);
	for(int x = 0 ; x < len ; x++)
	  buffer[x] = Wire.read();
#else  //SPI interface 
  uint8_t regbuffer = reg | READ;
  digitalWrite(chipSelectPin,LOW);
  SPI.transfer(regbuffer);
  
  for (int x = 0 ; x < len ; x++)
    buffer[x] = SPI.transfer(0x00);
		
  digitalWrite(chipSelectPin, HIGH);
#endif
}

/*********************************************************************
 *** Configmode
 *********************************************************************/
int	MC5010::configmode(MC5010_mode_type eNextMode)
{
  uint8_t _bCurrMode  = 0;
  uint8_t _bRegMODE_C = 0;
  uint8_t _bGuard     = 0;

  regRead(GYRO_REG_STATUS, &_bCurrMode, 1);

  if ((eNextMode < MC5010_MODE_STANDBY) && (eNextMode == MC5010_REG_STATUS_MODE(_bCurrMode)))
    return MCUBE_RATCODE_ERROR_STATUS;

  if (MC5010_MODE_SLEEP == eNextMode) {
    //Serial.print("[MC5010_SetMode] MODE_SLEEP\r\n");
  }
  else if (MC5010_MODE_STANDBY == eNextMode) {
    //Serial.print("[MC5010_SetMode] MODE_STANDBY\r\n");
  }
  else {
    //Serial.print("[MC5010_SetMode] MODE WAKE\r\n");
  }

  _bRegMODE_C |= eNextMode;
  regWrite(GYRO_REG_MODE, &_bRegMODE_C, 1);

  while (1)
  {
    _bGuard++;

    delay(3);

    regRead(GYRO_REG_STATUS, &_bCurrMode, 1);

    if ((eNextMode < MC5010_MODE_STANDBY) && (eNextMode == MC5010_REG_STATUS_MODE(_bCurrMode)))
    {
      break;
    }
    else if ((eNextMode == MC5010_MODE_STANDBY) && (0 == MC5010_REG_STATUS_MODE(_bCurrMode)))
    {
      _bRegMODE_C = 0x07;
      regWrite(GYRO_REG_MODE, &_bRegMODE_C, 1);
      break;
    }

    if (_bGuard > 64)
      return MCUBE_RATCODE_ERROR_SETUP;
  }

  return MCUBE_RATCODE_SUCCESS;
}

/*********************************************************************
 *** Reset
 *********************************************************************/
int	MC5010::reset(void)
{
  setMode(MC5010_MODE_STANDBY);

  uint8_t _xyPos[2];
  uint16_t _xyPos_buf;
  uint8_t _bRegData;
  int OTP_SIZE = 0;
  unsigned char *OTP_REG;
  unsigned char *OTP_VAL;

  regRead(GYRO_REG_DEVICEID_X, _xyPos, 2);
  _xyPos_buf = (( _xyPos[0] & 0x00FF) << 8 );
  _xyPos_buf = _xyPos_buf | ( _xyPos[1] & 0x00FF);
  //Serial.print("_xyPos_buf = "); Serial.println(_xyPos_buf, HEX);

  /*Use default power on sequence*/
  OTP_SIZE = sizeof (CHIP_R2B_REG);
  OTP_REG = (unsigned char *)malloc(OTP_SIZE);
  OTP_REG = CHIP_R2B_REG;
  OTP_VAL = (unsigned char *)malloc(OTP_SIZE);
  OTP_VAL = CHIP_R2B_VAL;

  for (int i=0;i<OTP_SIZE;i++) {
  	_bRegData = OTP_VAL[i];
    regWrite(OTP_REG[i], &_bRegData, 1);
  }

  free(OTP_REG);
  free(OTP_VAL);

  configModeINT(MC5010_INT_MODE_IPP_OPEN_DRAIN_IAH_LOW);
  configINT(MC5010_INT_DIS);
  configFilter(MC5010_LPF_DISABLE, MC5010_HPF_DISABLE);
  setMode(MC5010_MODE_STANDBY);

  return MCUBE_RATCODE_SUCCESS;
}

/*******************************************************************************
 *** GLOBAL FUNCTION
 *******************************************************************************/
/*********************************************************************
 *** Init
 *** Sensor Initialize
 *********************************************************************/
int	MC5010::init(void)
{
  Serial.print("[MC5010_Init]\r\n");
	
#ifdef MC5010_CFG_BUS_I2C
  Wire.begin();
#endif

  reset();

  configFilter(MC5010_CFG_LPF, MC5010_CFG_HPF);
  setRate(MC5010_CFG_RATE);
  setDPS(MC5010_CFG_DPS);
#ifdef MC5010_CFG_BUS_SPI
  configFIFO(MC5010_FIFO_ENABLE);
#endif
	
  setMode(MC5010_MODE_WAKE);

  return MCUBE_RATCODE_SUCCESS;
}

/*********************************************************************
 *** SetMode
 *********************************************************************/
int MC5010::setMode(MC5010_mode_type eNextMode)
{
  configmode(MC5010_MODE_STANDBY);
  configmode(eNextMode);

  return MCUBE_RATCODE_SUCCESS;
}

/*****************************************
 *** SetRate
 *****************************************/
int MC5010::setRate(MC5010_rate_type eCfgSR)
{
  uint8_t _bPreMode = 0;
  uint8_t _bCfgRate = eCfgSR & 0x0F;

  regRead(GYRO_REG_MODE, &_bPreMode, 1);
  setMode(MC5010_MODE_STANDBY);
  regWrite(GYRO_REG_SCALE_RATE_CTRL, &_bCfgRate, 1);
  regWrite(GYRO_REG_MODE, &_bPreMode, 1);

  s_eRate = eCfgSR;

  return MCUBE_RATCODE_SUCCESS;
}

/*****************************************
 *** SetDPS
 *****************************************/
int MC5010::setDPS(MC5010_dps_type eCfgDPS)
{
  uint8_t _bPreMode = 0;
  uint8_t _bCfgDPS = eCfgDPS & 0xF0;

  regRead(GYRO_REG_MODE, &_bPreMode, 1);
  setMode(MC5010_MODE_STANDBY);
  regWrite(GYRO_REG_SCALE_RATE_CTRL, &_bCfgDPS, 1);
  regWrite(GYRO_REG_MODE, &_bPreMode, 1);

  s_eDPS = eCfgDPS;

  return MCUBE_RATCODE_SUCCESS;
}

/*****************************************
 *** GetRate
 *****************************************/
int MC5010::getRate()
{
  uint8_t _bCfgRate = 0;

  regRead(GYRO_REG_SCALE_RATE_CTRL, &_bCfgRate, 1);

  return _bCfgRate & 0x0F;
}

/*****************************************
 *** GetDPS
 *****************************************/
int MC5010::getDPS()
{
  uint8_t _bCfgDPS = 0;

  regRead(GYRO_REG_SCALE_RATE_CTRL, &_bCfgDPS, 1);

  return _bCfgDPS & 0xF0;
}

/*********************************************************************
 *** ConfigModeINT
 *********************************************************************/
int	MC5010::configModeINT(int eCfgINTmode)
{
  uint8_t	_bPreMode = 0;
  uint8_t	_bINTmode = eCfgINTmode;

  regRead(GYRO_REG_MODE, &_bPreMode, 1);
  setMode(MC5010_MODE_STANDBY);
  regWrite(GYRO_REG_INT_CTRL, &_bINTmode, 1);
  regWrite(GYRO_REG_MODE, &_bPreMode, 1);

  return MCUBE_RATCODE_SUCCESS;
}

/*********************************************************************
 *** ConfigINT
 *********************************************************************/
int	MC5010::configINT(int eCfgINT)
{
  uint8_t _bPreMode = 0;
  uint8_t	_bINTmode = eCfgINT;

  regRead(GYRO_REG_MODE, &_bPreMode, 1);
  setMode(MC5010_MODE_STANDBY);
  regWrite(GYRO_REG_INT_EN, &_bINTmode, 1);
  regWrite(GYRO_REG_MODE, &_bPreMode, 1);

  return MCUBE_RATCODE_SUCCESS;
}

/*****************************************
 *** ConfigLPF
 *****************************************/
int	MC5010::configFilter(int eCfgLPF, int eCfgHPF)
{
  uint8_t _bPreMode	= 0;
  uint8_t _bFilterModeCheck	= 0;
  uint8_t _bFilter = eCfgLPF | eCfgHPF;

  regRead(GYRO_REG_MODE, &_bPreMode, 1);
  setMode(MC5010_MODE_STANDBY);
  regWrite(GYRO_REG_FILTER_CTRL, &_bFilter, 1);
  regRead(GYRO_REG_FILTER_CTRL, &_bFilterModeCheck, 1);
    
  if ((_bFilterModeCheck & 0x08) >> 3) {
  	//Serial.print("LPF Enable.\r\n");
  }
  else {
    //Serial.print("LPF Disable.\r\n");
  }

  if ((_bFilterModeCheck & 0x80) >> 7) {
    //Serial.print("HPF Enable.\r\n");
  }
  else {
    //Serial.print("HPF Disable.\r\n");
  }

  s_eLPF = (MC5010_lpf_mode_type) eCfgLPF;
  s_eHPF = (MC5010_hpf_mode_type) eCfgHPF;

  regWrite(GYRO_REG_MODE, &_bPreMode, 1);

  return MCUBE_RATCODE_SUCCESS;
}

/*********************************************************************
 *** ConfigFIFO
 *********************************************************************/
int	MC5010::configFIFO(MC5010_fifo_type eCtrl)
{
  unsigned char _bPreMode = 0;
  uint8_t	_bCfgFifo 			= eCtrl;

  if (eCtrl >= MC5010_FIFO_END)
    return MCUBE_RATCODE_ERROR_WRONG_ARGUMENT;

  regRead(GYRO_REG_MODE, &_bPreMode, 1);
  setMode(MC5010_MODE_STANDBY);
  regWrite(GYRO_REG_FIFO_CTRL, &_bCfgFifo, 1);
  regWrite(GYRO_REG_MODE, &_bPreMode, 1);

  return MCUBE_RATCODE_SUCCESS;
}

/*********************************************************************
 *** ReadSensorData
 *********************************************************************/
MC5010_acc_t MC5010::readSensorData()
{
  Serial.print("[MC5010_ReadSensorData]\r\n");
	
  short	_waRaw[MC5010_AXIS_NUM] = {0};
  uint8_t _baData[6]            = {0};
  int _bPrjCfgDPS               =  0 ;

  const MC_UTIL_ORIENTATIONREMAP   *_ptOrienMap = &g_MDrvUtilOrientationReMap[MC5010_CFG_ORIENTATION_MAP];

  regRead(GYRO_REG_XOUT_L , _baData , 6);
  _waRaw[MC5010_AXIS_X] = ((signed short) ((_baData[0])	| (_baData[1] << 8)));
  _waRaw[MC5010_AXIS_Y] = ((signed short) ((_baData[2])	| (_baData[3] << 8)));
  _waRaw[MC5010_AXIS_Z] = ((signed short) ((_baData[4])	| (_baData[5] << 8)));

  MC5010_acc_t raw;
  raw.XAxis = ((_ptOrienMap->bSign[MC5010_AXIS_X] * _waRaw[_ptOrienMap->bMap[MC5010_AXIS_X]]));
  raw.YAxis = ((_ptOrienMap->bSign[MC5010_AXIS_Y] * _waRaw[_ptOrienMap->bMap[MC5010_AXIS_Y]]));
  raw.ZAxis = ((_ptOrienMap->bSign[MC5010_AXIS_Z] * _waRaw[_ptOrienMap->bMap[MC5010_AXIS_Z]]));
  raw.XAxis_f = ((float)(_ptOrienMap->bSign[MC5010_AXIS_X] * _waRaw[_ptOrienMap->bMap[MC5010_AXIS_X]]));
  raw.YAxis_f = ((float)(_ptOrienMap->bSign[MC5010_AXIS_Y] * _waRaw[_ptOrienMap->bMap[MC5010_AXIS_Y]]));
  raw.ZAxis_f = ((float)(_ptOrienMap->bSign[MC5010_AXIS_Z] * _waRaw[_ptOrienMap->bMap[MC5010_AXIS_Z]]));

  switch((int)s_eDPS)
  {
    case MC5010_DPS_2kdps:
         _bPrjCfgDPS = 2000;
         break;
    case MC5010_DPS_1kdps:
         _bPrjCfgDPS = 1000;
         break;
    case MC5010_DPS_500dps:
         _bPrjCfgDPS = 500;
         break;
    case MC5010_DPS_250dps:
         _bPrjCfgDPS = 250;
         break;
    case MC5010_DPS_125dps:
         _bPrjCfgDPS = 125;
         break;
    case MC5010_DPS_8kdps:
         _bPrjCfgDPS = 8000;
         break;
  }

  raw.XAxis_f = (raw.XAxis_f / (MC5010_GAIN_16BIT/_bPrjCfgDPS));
  raw.YAxis_f = (raw.YAxis_f / (MC5010_GAIN_16BIT/_bPrjCfgDPS));
  raw.ZAxis_f = (raw.ZAxis_f / (MC5010_GAIN_16BIT/_bPrjCfgDPS));

  raw.XAxis_f = (raw.XAxis_f * 0.017453f);
  raw.YAxis_f = (raw.YAxis_f * 0.017453f);
  raw.ZAxis_f = (raw.ZAxis_f * 0.017453f);

  return raw;
}











































