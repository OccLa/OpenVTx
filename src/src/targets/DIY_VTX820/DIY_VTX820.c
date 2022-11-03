#include "targets.h"
#include "common.h"
#include "openVTxEEPROM.h"
#include "gpio.h"
#include "pwm.h"
#include "printf.h"
#include "helpers.h"
#include <math.h>

#define OUTPUT_POWER_INTERVAL 5 // ms

/* SA2.1 powerlevels in dBm.
 *
 * INav:
 *    Max of 5 [https://github.com/iNavFlight/inav/blob/a8016edd0d6f05bb12a75b0ea75a3483772baaeb/src/main/io/vtx_smartaudio.h#L36]
 *    Index 0 is ignored [https://github.com/iNavFlight/inav/blob/a8016edd0d6f05bb12a75b0ea75a3483772baaeb/src/main/io/vtx_smartaudio.c#L334]
 *
 */
uint8_t saPowerLevelsLut[SA_NUM_POWER_LEVELS] = {20, 23, 26, 28, 29};

uint8_t saPowerLevelsLabel[SA_NUM_POWER_LEVELS * POWER_LEVEL_LABEL_LENGTH] = {'1', '0', '0',
                                                                              '2', '0', '0',
                                                                              '4', '0', '0',
                                                                              '6', '0', '0',
                                                                              '8', '0', '0'};

gpio_pwm_t outputPowerTimer;
gpio_out_t vref_pin;
gpio_adc_t vpd_pin;
uint32_t currentVpd = 0;

uint16_t pwm_val = 2500;
uint16_t VpdSetPoint = 0;
uint8_t amp_state = 0;

// #if defined(DIY_VTX820)
  #define CAL_FREQ_SIZE 9
  #define CAL_DBM_SIZE 6
  uint16_t calFreqs[CAL_FREQ_SIZE] = {5600,	5650, 5700, 5750, 5800,	5850, 5900, 5950, 6000};
  uint8_t calDBm[CAL_DBM_SIZE] = {20, 23, 26, 27, 28, 29}; // 100mW 200mW 400mW 500mW 600mW 800mW
  uint16_t calVpd[CAL_DBM_SIZE][CAL_FREQ_SIZE] = {
  // 5600, 5650, 5700, 5750, 5800, 5850, 5900, 5950, 6000
  {473, 463, 451, 451, 442, 449, 462, 462, 476}, // 100mW
  {586, 578, 549, 547, 545, 560, 562, 583, 595}, // 200mW
  {779, 770, 721, 716, 717, 733, 771, 750, 788}, // 400mW
  {894, 881, 816, 816, 811, 822, 841, 874, 892}, // 500mW
  {1096, 1075, 964, 958, 949, 964, 1013, 1038, 1076}, // 600mW
  {1500, 1500, 1185, 1190, 1170, 1310, 1400, 1450, 1500} // 800mW
  };  
// #endif



uint16_t bilinearInterpolation(float dB)
{
  uint16_t tempFreq = myEEPROM.currFreq;
  uint8_t i;
  uint8_t calFreqsIndex = 0;
  uint8_t calDBmIndex = 0;

  dB = dB + OFFSET;

  if (tempFreq < 5600) tempFreq = 5600;
  if (tempFreq > 6000) tempFreq = 6000;

  for (i = 0; i < (ARRAY_SIZE(calFreqs) - 1); i++)
  {
    if (tempFreq < calFreqs[i + 1])
    {
      calFreqsIndex = i;
      break;
    }
  }

  for (i = 0; i < (ARRAY_SIZE(calDBm) - 1); i++)
  {
    if (dB < calDBm[i + 1])
    {
      calDBmIndex = i;
      break;
    }
  }

  float x = dB;
  float x1 = calDBm[calDBmIndex];
  float x2 = calDBm[calDBmIndex + 1];

  float y = tempFreq;
  float y1 = calFreqs[calFreqsIndex];
  float y2 = calFreqs[calFreqsIndex + 1];

  float Q11 = calVpd[calDBmIndex][calFreqsIndex];
  float Q12 = calVpd[calDBmIndex][calFreqsIndex + 1];
  float Q21 = calVpd[calDBmIndex + 1][calFreqsIndex];
  float Q22 = calVpd[calDBmIndex + 1][calFreqsIndex + 1];

  float fxy1 = Q11 * (x2 - x) / (x2 - x1) + Q21 * (x - x1) / (x2 - x1);
  float fxy2 = Q12 * (x2 - x) / (x2 - x1) + Q22 * (x - x1) / (x2 - x1);

  uint16_t fxy = fxy1 * (y2 - y) / (y2 - y1) + fxy2 * (y - y1) / (y2 - y1);

  return fxy;
}

uint16_t setMaxPower()
{
  uint16_t tempFreq = myEEPROM.currFreq;
  uint8_t i;
  uint8_t freqsIndex = 0;
  uint8_t maxVpdIndex = ARRAY_SIZE(calVpd) - 1;

  if (tempFreq < 5600) tempFreq = 5600;
  if (tempFreq > 6000) tempFreq = 6000;

  for (i = 0; i < (ARRAY_SIZE(calFreqs) - 1); i++)
  {
    if (tempFreq < calFreqs[i + 1])
    {
      freqsIndex = i;
      break;
    }
  }

  return calVpd[maxVpdIndex][freqsIndex];
}

void target_rfPowerAmpPinSetup(void)
{
  vref_pin = gpio_out_setup(VREF, 0); // Power amp OFF

  outputPowerTimer = pwm_init(RTC_BIAS);
  pwm_out_write(outputPowerTimer, 3000);

  vpd_pin = adc_config(VPD);
}

uint32_t vpd_value_get(void)
{
  return adc_read(vpd_pin);
}

void increasePWMVal()
{
  if (pwm_val < 2500)
  {
    pwm_val++;
    pwm_out_write(outputPowerTimer, pwm_val);
  }
}

void decreasePWMVal()
{
  if (pwm_val > 2000)
  {
    pwm_val--;
    pwm_out_write(outputPowerTimer, pwm_val);
  }
}

void target_set_power_dB(float dB)
{
  if (dB < 20)
  {
    VpdSetPoint = 0;
    pwm_val = 2500;
    amp_state = 0;
  } else if (dB > 29)
  {
    VpdSetPoint = 1500;
    pwm_val = 2000;
    amp_state = 1;
  } else if (dB == 29)
  {
    VpdSetPoint = setMaxPower();
    amp_state = 1;
  } else
  {
    VpdSetPoint = bilinearInterpolation(dB);
    amp_state = 1;
  }

  pwm_out_write(outputPowerTimer, pwm_val);
  gpio_out_write(vref_pin, amp_state);

  #if OUTPUT_POWER_TESTING
  VpdSetPoint = 0;
  pwm_val = 3000;
  pwm_out_write(outputPowerTimer, pwm_val);
  amp_state = 1;
  gpio_out_write(vref_pin, amp_state);
  #endif /* OUTPUT_POWER_TESTING */
}

void checkPowerOutput(void)
{
  static uint32_t temp;
  uint32_t now = millis();
  if (OUTPUT_POWER_INTERVAL <= (now - temp))
  {
    temp = now;
    currentVpd = vpd_value_get();

    if (currentVpd > VpdSetPoint)
    {
      increasePWMVal();
    }
    else if (currentVpd < VpdSetPoint)
    {
      decreasePWMVal();
    }
  }
}

void target_setup(void)
{
  /* TODO: Configure WDG, fwdgt_config() */
}

void target_loop(void)
{
#if DEBUG
  static uint32_t temp;
  static char buff[32];
  int len;
  uint32_t now = millis();
  currentVpd = vpd_value_get();
  if (200 <= (now - temp)) {
    temp = now;

    len = snprintf(buff, sizeof(buff), "%lu,%lu,%lu,%lu\n", myEEPROM.currFreq, pwm_val, VpdSetPoint, currentVpd);
    Serial_write_len((uint8_t*)buff, len);

    #if OUTPUT_POWER_TESTING
    VpdSetPoint = VpdSetPoint + 5;
    if (VpdSetPoint > 2000)
    {
      VpdSetPoint = 0;
      pwm_val = 3000;
      pwm_out_write(outputPowerTimer, pwm_val);
      myEEPROM.currFreq = myEEPROM.currFreq + 50;
      if (myEEPROM.currFreq > 6000)
      {
        myEEPROM.currFreq = 5600;
      }
      rtc6705WriteFrequency(myEEPROM.currFreq);
    }
    #endif /* OUTPUT_POWER_TESTING */
  }
#endif /* DEBUG */

  /* Reset WD */
  fwdgt_counter_reload();
}
