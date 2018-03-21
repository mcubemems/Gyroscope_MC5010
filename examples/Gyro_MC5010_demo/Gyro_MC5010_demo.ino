#include <MC5010.h>

MC5010 MC5010_gyro = MC5010();

void setup()
{
  Serial.begin(115200);
  MC5010_gyro.init();
  checkRate();
  checkDPS();
  
}

void checkRate()
{
  switch(MC5010_gyro.getRate())
  {
    case MC5010_RATE_MAX:             Serial.println("Rate: 2KHz without LPF"); break;     
    case MC5010_RATE_1kHz:            Serial.println("Rate: 2KHz and 1KHz cutoff frequency"); break;         
    case MC5010_RATE_512Hz:           Serial.println("Rate: 1KHz and 512Hz cutoff frequency"); break;
    case MC5010_RATE_256Hz:           Serial.println("Rate: 512Hz and 256Hz cutoff frequency"); break;
    case MC5010_RATE_128Hz:           Serial.println("Rate: 256Hz and 128Hz cutoff frequency"); break;
    case MC5010_RATE_64Hz:            Serial.println("Rate: 128Hz and 64Hz cutoff frequency"); break;
    default:                        Serial.println("Rate: 2KHz without LPF"); break;

    /*
     * MC5010_RATE_MAX = 0x00 = Max : ODR 2kHz and all pass through for corner frequency
     * MC5010_RATE_1kHz = 0x01 = 1kHz : ODR 2kHz and 1kHz corner frequency
     * MC5010_RATE_512Hz = 0x02 = 512Hz : ODR 1kHz and 512Hz corner frequency
     * MC5010_RATE_256Hz = 0x03 = 256Hz : ODR 512Hz and 256Hz corner frequency
     * MC5010_RATE_128Hz = 0x04 = 128Hz : ODR 256Hz and 128Hz corner frequency
     * MC5010_RATE_64Hz = 0x05 = 64Hz : ODR 128Hz and 64Hz corner frequency
     */ 
  }   
}

void checkDPS()
{
  switch(MC5010_gyro.getDPS())
  {
    case MC5010_DPS_2kdps:            Serial.println("DPS: +/-2K"); break;     
    case MC5010_DPS_1kdps:            Serial.println("DPS: +/-1K"); break;         
    case MC5010_DPS_500dps:           Serial.println("DPS: +/-500"); break;
    case MC5010_DPS_250dps:           Serial.println("DPS: +/-250"); break;
    case MC5010_DPS_125dps:           Serial.println("DPS: +/-125"); break;
    case MC5010_DPS_8kdps:            Serial.println("DPS: +/-8K"); break;
    default:                        Serial.println("DPS: +/-2K"); break;

    /*
     * MC5010_DPS_2kdps = 0x00 = +/-2k 
     * MC5010_DPS_1kdps = 0x10 = +/-1k 
     * MC5010_DPS_500dps = 0x20 = +/-500 
     * MC5010_DPS_250dps = 0x30 = +/-250
     * MC5010_DPS_125dps = 0x40 = +/-125
     * MC5010_DPS_8kdps = 0x50 = +/-8k
     */
  }   
}

void loop()
{
  MC5010_acc_t raw = MC5010_gyro.readSensorData();
  Serial.print("X:\t"); Serial.print(raw.XAxis); Serial.print("\t");
  Serial.print("Y:\t"); Serial.print(raw.YAxis); Serial.print("\t");
  Serial.print("Z:\t"); Serial.print(raw.ZAxis); Serial.print("\t");
  Serial.println("counts");
  Serial.print("X: \t"); Serial.print(raw.XAxis_f); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(raw.YAxis_f); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(raw.ZAxis_f); Serial.print("\t");
  Serial.println("rad/s");
  delay(100);
}
