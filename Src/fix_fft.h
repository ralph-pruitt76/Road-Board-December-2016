/**
  ******************************************************************************
  * File Name          : fix_fft.h   Fixed-point in-place Fast Fourier Transform
  * Description        : This file provides code for the processing of the fixed fft 
  *                      needed to process the road sound from the ADC.
  *     All data are fixed-point short integers, in which -32768
  *       to +32768 represent -1.0 to +1.0 respectively. Integer
  *       arithmetic is used for speed, instead of the more natural
  *       floating-point.
  *     
  *       For the forward FFT (time -> freq), fixed scaling is
  *       performed to prevent arithmetic overflow, and to map a 0dB
  *       sine/cosine wave (i.e. amplitude = 32767) to two -6dB freq
  *       coefficients. The return value is always 0.
  *     
  *       For the inverse FFT (freq -> time), fixed scaling cannot be
  *       done, as two 0dB coefficients would sum to a peak amplitude
  *       of 64K, overflowing the 32k range of the fixed-point integers.
  *       Thus, the fix_fft() routine performs variable scaling, and
  *       returns a value which is the number of bits LEFT by which
  *       the output must be shifted to get the actual amplitude
  *       (i.e. if fix_fft() returns 3, each value of fr[] and fi[]
  *       must be multiplied by 8 (2**3) for proper scaling.
  *       Clearly, this cannot be done within fixed-point short
  *       integers. In practice, if the result is to be used as a
  *       filter, the scale_shift can usually be ignored, as the
  *       result will be approximately correctly normalized as is.
  *     
  *       Written by:  Tom Roberts  11/8/89
  *       Made portable:  Malcolm Slaney 12/15/94 malcolm@interval.com
  *       Enhanced:  Dimitrios P. Bouras  14 Jun 2006 dbouras@ieee.org
  *       Modified for 8bit values David Keller  10.10.2010
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 WeatherCloud
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of WeatherCloud nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FIXFFT_H
#define FIXFFT_H
#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
//#include <WProgram.h>
#include "stm32l1xx_hal.h"

/*
  fix_fft() - perform forward/inverse fast Fourier transform.
  fr[n],fi[n] are real and imaginary arrays, both INPUT AND
  RESULT (in-place FFT), with 0 <= n < 2**m; set inverse to
  0 for forward transform (FFT), or 1 for iFFT.
*/
int fix_fft(uint8_t fr[], uint8_t fi[], int m, int inverse);



/*
  fix_fftr() - forward/inverse FFT on array of real numbers.
  Real FFT/iFFT using half-size complex FFT by distributing
  even/odd samples into real/imaginary arrays respectively.
  In order to save data space (i.e. to avoid two arrays, one
  for real, one for imaginary samples), we proceed in the
  following two steps: a) samples are rearranged in the real
  array so that all even samples are in places 0-(N/2-1) and
  all imaginary samples in places (N/2)-(N-1), and b) fix_fft
  is called with fr and fi pointing to index 0 and index N/2
  respectively in the original array. The above guarantees
  that fix_fft "sees" consecutive real samples as alternating
  real and imaginary samples in the complex array.
*/
int fix_fftr(uint8_t f[], int m, int inverse);


#ifdef __cplusplus
}
#endif
#endif /* FIXFFT_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/


