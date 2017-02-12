/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 68 Hz

fixed point precision: 16 bits

* 0 Hz - 2 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5 Hz - 34 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM 73

int filter_taps[FILTER_TAP_NUM] = {
  -14,
  -4,
  1,
  12,
  30,
  56,
  88,
  126,
  166,
  201,
  227,
  235,
  218,
  170,
  87,
  -31,
  -181,
  -353,
  -533,
  -703,
  -840,
  -919,
  -916,
  -808,
  -578,
  -218,
  274,
  887,
  1600,
  2383,
  3195,
  3992,
  4728,
  5356,
  5837,
  6139,
  6241,
  6139,
  5837,
  5356,
  4728,
  3992,
  3195,
  2383,
  1600,
  887,
  274,
  -218,
  -578,
  -808,
  -916,
  -919,
  -840,
  -703,
  -533,
  -353,
  -181,
  -31,
  87,
  170,
  218,
  235,
  227,
  201,
  166,
  126,
  88,
  56,
  30,
  12,
  1,
  -4,
  -14
};
