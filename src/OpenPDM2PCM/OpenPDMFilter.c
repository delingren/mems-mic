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
 
int32_t filter_tables_64(uint8_t *data, uint8_t sincn)
{
  return (int32_t)
    lut[data[0]][0][sincn] +
    lut[data[1]][1][sincn] +
    lut[data[2]][2][sincn] +
    lut[data[3]][3][sincn] +
    lut[data[4]][4][sincn] +
    lut[data[5]][5][sincn] +
    lut[data[6]][6][sincn] +
    lut[data[7]][7][sincn];
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
 
void Open_PDM_Filter_Init(TPDMFilter_InitStruct *Param)
{
  uint16_t i, j;
  int64_t sum = 0;
  
  for (i = 0; i < SINCN; i++) {
    Param->Coef[i] = 0;
    Param->bit[i] = 0;
  }
  for (i = 0; i < DECIMATION; i++) {
    sinc1[i] = 1;
  }
 
  Param->OldOut = Param->OldIn = Param->OldZ = 0;
  Param->LP_ALFA = (Param->LP_HZ != 0 ? (uint16_t) (Param->LP_HZ * 256 / (Param->LP_HZ + Param->Fs / (2 * 3.14159))) : 0);
  Param->HP_ALFA = (Param->HP_HZ != 0 ? (uint16_t) (Param->Fs * 256 / (2 * 3.14159 * Param->HP_HZ + Param->Fs)) : 0);
 
  Param->FilterLen = DECIMATION * SINCN;       
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
  div_const = sub_const * Param->MaxVolume / 32768 / FILTER_GAIN;
  div_const = (div_const == 0 ? 1 : div_const);
 
  /* Look-Up Table. */
  uint16_t c, d, s;
  for (s = 0; s < SINCN; s++)
  {
    uint32_t *coef_p = &coef[s][0];
    for (c = 0; c < 256; c++)
      for (d = 0; d < DECIMATION / 8; d++)
        lut[c][d][s] = ((c >> 7)       ) * coef_p[d * 8    ] +
                       ((c >> 6) & 0x01) * coef_p[d * 8 + 1] +
                       ((c >> 5) & 0x01) * coef_p[d * 8 + 2] +
                       ((c >> 4) & 0x01) * coef_p[d * 8 + 3] +
                       ((c >> 3) & 0x01) * coef_p[d * 8 + 4] +
                       ((c >> 2) & 0x01) * coef_p[d * 8 + 5] +
                       ((c >> 1) & 0x01) * coef_p[d * 8 + 6] +
                       ((c     ) & 0x01) * coef_p[d * 8 + 7];
  }
}

// Decimation = 64, i.e. every 8 bytes produce an output value
void Open_PDM_Filter_64(uint8_t* data, uint16_t* dataOut, uint16_t volume, TPDMFilter_InitStruct *Param)
{
  uint8_t i, data_out_index;
  uint8_t input_stride = 64 / 8; 
  int64_t Z, Z0, Z1, Z2;
  int64_t OldOut, OldIn, OldZ;
 
  OldOut = Param->OldOut;
  OldIn = Param->OldIn;
  OldZ = Param->OldZ;
  
  for (i = 0, data_out_index = 0; i < Param->Fs / 1000; i++, data_out_index ++) {
    Z0 = filter_tables_64(data, 0);
    Z1 = filter_tables_64(data, 1);
    Z2 = filter_tables_64(data, 2);
 
    Z = Param->Coef[1] + Z2 - sub_const;
    Param->Coef[1] = Param->Coef[0] + Z1;
    Param->Coef[0] = Z0;
 
    OldOut = (Param->HP_ALFA * (OldOut + Z - OldIn)) >> 8;
    OldIn = Z;
    OldZ = ((256 - Param->LP_ALFA) * OldZ + Param->LP_ALFA * OldOut) >> 8;
 
    Z = OldZ * volume;
    Z = RoundDiv(Z, div_const);
    Z = SaturaLH(Z, -32700, 32700);
 
    dataOut[data_out_index] = Z;
    data += input_stride;
  }
 
  Param->OldOut = OldOut;
  Param->OldIn = OldIn;
  Param->OldZ = OldZ;
}
