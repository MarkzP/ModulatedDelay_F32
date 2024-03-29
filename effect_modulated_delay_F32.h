/*
   Copyright (c) 2014, Pete (El Supremo)
   Copyright (c) 2019-2021 H. Wirtz

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

#ifndef effect_modulated_delay_F32_h_
#define effect_modulated_delay_F32_h_

#include "OpenAudio_ArduinoLibrary.h"
#include "AudioStream_F32.h"

/*************************************************************************/
//                A u d i o E f f e c t M o d u l a t e d D e l a y
// Written by Pete (El Supremo) Jan 2014
// 140219 - correct storage class (not static)
// 190527 - added modulation input handling (Aug 2019 by Holger Wirtz)
// ported by Marc Paquette to the Teensy OpenAudio F32 library
//    https://github.com/chipaudette/OpenAudio_ArduinoLibrary

class AudioEffectModulatedDelay_F32 :
  public AudioStream_F32
{
//GUI: inputs:1, outputs:1  //this line used for automatic generation of GUI node
//GUI: shortName:effect_ModulatedDelay	
  public:
    AudioEffectModulatedDelay_F32(void):
      AudioStream_F32(2, inputQueueArray)
    {
      _sample_rate_Hz = AUDIO_SAMPLE_RATE_EXACT;
    }

    AudioEffectModulatedDelay_F32(const AudioSettings_F32 &settings):
      AudioStream_F32(2, inputQueueArray)
    {
      _sample_rate_Hz = settings.sample_rate_Hz;
    }    

    boolean begin(float *delayline, uint16_t delay_length);
    virtual void update(void);
    virtual void delay(float ms);
    virtual uint16_t get_delay_length(void);
    float get_mod_delay_ms(){ return 1000.0f * (_delay_offset + _mod_index) / _sample_rate_Hz; }

  private:
    audio_block_f32_t *inputQueueArray[2];
    float _sample_rate_Hz;
    float *_delayline;  // pointer for the circular buffer
    uint16_t _cb_index;   // current write pointer of the circular buffer
    uint16_t _delay_length; // calculated number of samples of the delay
    uint16_t _max_delay_length;
    float _mod_index;
    float _delay_offset;
};

#endif
