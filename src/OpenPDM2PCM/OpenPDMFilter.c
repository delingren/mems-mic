/**
 *******************************************************************************
 * @file    OpenPDMFilter.c
 * @author  CL
 * @version V1.0.0
 * @date    9-September-2015
 * @brief   Open PDM audio software decoding Library.   
 *          This Library is used to decode and reconstruct the audio signal
 *          produced by ST MEMS microphone (MP45Dxxx, MP34Dxxx). 
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************
 */
 
 
/* Includes ------------------------------------------------------------------*/
 
#include "OpenPDMFilter.h"
 
 
/* Variables -----------------------------------------------------------------*/
 
uint32_t div_const = 0;
int64_t sub_const = 0;
uint32_t sinc[DECIMATION * SINCN];
uint32_t sinc1[DECIMATION];
uint32_t sinc2[DECIMATION * 2];
uint32_t coef[SINCN][DECIMATION];
int32_t lut[256][DECIMATION / 8][SINCN];
 
 
/* Functions -----------------------------------------------------------------*/

static inline uint8_t left (uint8_t* word) {
  return (((*word) & 0x55) << 1) | (*(word+1) & 0x55);
}

static inline uint8_t right(uint8_t* word) {
  return ((*word) & 0xAA) | ((*(word+1) & 0xAA) >> 1); 
}


int32_t filter_tables_left_64(uint8_t *data, uint8_t sincn)
{
  return (int32_t)
    lut[left(data+0)][0][sincn] +
    lut[left(data+2)][1][sincn] +
    lut[left(data+4)][2][sincn] +
    lut[left(data+6)][3][sincn] +
    lut[left(data+8)][4][sincn] +
    lut[left(data+10)][5][sincn] +
    lut[left(data+12)][6][sincn] +
    lut[left(data+14)][7][sincn];
}

int32_t filter_tables_right_64(uint8_t *data, uint8_t sincn)
{
  return (int32_t)
    lut[right(data+0)][0][sincn] +
    lut[right(data+2)][1][sincn] +
    lut[right(data+4)][2][sincn] +
    lut[right(data+6)][3][sincn] +
    lut[right(data+8)][4][sincn] +
    lut[right(data+10)][5][sincn] +
    lut[right(data+12)][6][sincn] +
    lut[right(data+14)][7][sincn];
}

void convolve(uint32_t Signal[/* SignalLen */], unsigned short SignalLen,
              uint32_t Kernel[/* KernelLen */], unsigned short KernelLen,
              uint32_t Result[/* SignalLen + KernelLen - 1 */])
{
  uint16_t n;
 
  for (n = 0; n < SignalLen + KernelLen - 1; n++)
  {
    unsigned short kmin, kmax, k;
    
    Result[n] = 0;
    
    kmin = (n >= KernelLen - 1) ? n - (KernelLen - 1) : 0;
    kmax = (n < SignalLen - 1) ? n : SignalLen - 1;
    
    for (k = kmin; k <= kmax; k++) {
      Result[n] += Signal[k] * Kernel[n - k];
    }
  }
}
 
void Open_PDM_Filter_Init(TPDMFilter_InitStruct *leftParam, TPDMFilter_InitStruct *rightParam, int sample_rate)
{
  uint16_t i, j;
  int64_t sum = 0;
  
  for (i = 0; i < SINCN; i++) {
    leftParam->Coef[i] = 0;
    leftParam->bit[i] = 0;
    rightParam->Coef[i] = 0;
    rightParam->bit[i] = 0;
  }
  for (i = 0; i < DECIMATION; i++) {
    sinc1[i] = 1;
  }
 
  leftParam->OldOut = leftParam->OldIn = leftParam->OldZ = 0;
  leftParam->LP_ALFA = (leftParam->LP_HZ != 0 ? (uint16_t) (leftParam->LP_HZ * 256 / (leftParam->LP_HZ + sample_rate / (2 * 3.14159))) : 0);
  leftParam->HP_ALFA = (leftParam->HP_HZ != 0 ? (uint16_t) (sample_rate * 256 / (2 * 3.14159 * leftParam->HP_HZ + sample_rate)) : 0); 
  leftParam->FilterLen = DECIMATION * SINCN;

  rightParam->OldOut = rightParam->OldIn = rightParam->OldZ = 0;
  rightParam->LP_ALFA = (rightParam->LP_HZ != 0 ? (uint16_t) (rightParam->LP_HZ * 256 / (rightParam->LP_HZ + sample_rate / (2 * 3.14159))) : 0);
  rightParam->HP_ALFA = (rightParam->HP_HZ != 0 ? (uint16_t) (sample_rate * 256 / (2 * 3.14159 * rightParam->HP_HZ + sample_rate)) : 0);
  rightParam->FilterLen = DECIMATION * SINCN;       

  sinc[0] = 0;
  sinc[DECIMATION * SINCN - 1] = 0;      
  convolve(sinc1, DECIMATION, sinc1, DECIMATION, sinc2);
  convolve(sinc2, DECIMATION * 2 - 1, sinc1, DECIMATION, &sinc[1]);     
  for(j = 0; j < SINCN; j++) {
    for (i = 0; i < DECIMATION; i++) {
      coef[j][i] = sinc[j * DECIMATION + i];
      sum += sinc[j * DECIMATION + i];
    }
  }
 
  sub_const = sum >> 1;
  div_const = sub_const * (leftParam->MaxVolume + rightParam->MaxVolume) / 32768 / (leftParam->Gain + rightParam->Gain);
  div_const = (div_const == 0 ? 1 : div_const);
 
  /* Look-Up Table. */
  uint16_t c, d, s;
  for (s = 0; s < SINCN; s++)
  {
    uint32_t *coef_p = &coef[s][0];
    for (c = 0; c < 256; c++)
      for (d = 0; d < DECIMATION / 8; d++) {
        lut[c][d][s] = ((c >> 7)       ) * coef_p[d * 8    ] +
                       ((c >> 5) & 0x01) * coef_p[d * 8 + 1] +
                       ((c >> 3) & 0x01) * coef_p[d * 8 + 2] +
                       ((c >> 1) & 0x01) * coef_p[d * 8 + 3] +
                       ((c >> 6) & 0x01) * coef_p[d * 8 + 4] +
                       ((c >> 4) & 0x01) * coef_p[d * 8 + 5] +
                       ((c >> 2) & 0x01) * coef_p[d * 8 + 6] +
                       ((c >> 0) & 0x01) * coef_p[d * 8 + 7];
      }
  }
}

// Decimation = 64, i.e. every 8 bytes produce an output value
void Open_PDM_Filter_64(uint8_t* data, int len, uint16_t* dataOut, uint16_t volume, TPDMFilter_InitStruct *leftParam, TPDMFilter_InitStruct *rightParam)
{
  uint8_t i, data_out_index;
  uint8_t input_stride = 64 / 8 * 2; 
  int64_t leftOldOut, leftOldIn, leftOldZ;
  int64_t rightOldOut, rightOldIn, rightOldZ;
 
  leftOldOut = leftParam->OldOut;
  leftOldIn = leftParam->OldIn;
  leftOldZ = leftParam->OldZ;

  rightOldOut = rightParam->OldOut;
  rightOldIn = rightParam->OldIn;
  rightOldZ = rightParam->OldZ;
  
  for (i = 0, data_out_index = 0; i < len; i++, data_out_index += 2) {
    int64_t Z, Z0, Z1, Z2;
    
    Z0 = filter_tables_left_64(data, 0);
    Z1 = filter_tables_left_64(data, 1);
    Z2 = filter_tables_left_64(data, 2);
 
    Z = leftParam->Coef[1] + Z2 - sub_const;
    leftParam->Coef[1] = leftParam->Coef[0] + Z1;
    leftParam->Coef[0] = Z0;
 
    leftOldOut = (leftParam->HP_ALFA * (leftOldOut + Z - leftOldIn)) >> 8;
    leftOldIn = Z;
    leftOldZ = ((256 - leftParam->LP_ALFA) * leftOldZ + leftParam->LP_ALFA * leftOldOut) >> 8;
 
    Z = leftOldZ * volume;
    Z = RoundDiv(Z, div_const);
    Z = SaturaLH(Z, -32700, 32700);
 
    dataOut[data_out_index] = Z;

    Z0 = filter_tables_right_64(data, 0);
    Z1 = filter_tables_right_64(data, 1);
    Z2 = filter_tables_right_64(data, 2);
 
    Z = rightParam->Coef[1] + Z2 - sub_const;
    rightParam->Coef[1] = rightParam->Coef[0] + Z1;
    rightParam->Coef[0] = Z0;
 
    rightOldOut = (rightParam->HP_ALFA * (rightOldOut + Z - rightOldIn)) >> 8;
    rightOldIn = Z;
    rightOldZ = ((256 - rightParam->LP_ALFA) * rightOldZ + rightParam->LP_ALFA * rightOldOut) >> 8;
 
    Z = rightOldZ * volume;
    Z = RoundDiv(Z, div_const);
    Z = SaturaLH(Z, -32700, 32700);
 
    dataOut[data_out_index+1] = Z;

    data += input_stride;
  }
 
  leftParam->OldOut = leftOldOut;
  leftParam->OldIn = leftOldIn;
  leftParam->OldZ = leftOldZ;

  rightParam->OldOut = rightOldOut;
  rightParam->OldIn = rightOldIn;
  rightParam->OldZ = rightOldZ;
}
