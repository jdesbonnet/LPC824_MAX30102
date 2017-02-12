/** 
 * Low pass filter. 
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 68 Hz

* 0.5 Hz - 3 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.857626063879591 dB

* 5 Hz - 34 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.66478284451494 dB

*/

#define FILTER_TAP_NUM 39

static double filter_taps[FILTER_TAP_NUM] = {
  -0.007824982398550288,
  -0.007868634890172817,
  -0.01087259298568141,
  -0.013737306504390728,
  -0.01598020193782839,
  -0.017072162448370464,
  -0.016483225448184834,
  -0.01372404332627383,
  -0.008429355871957256,
  -0.00042540050885079954,
  0.010239915690916417,
  0.02326953079599063,
  0.03810413690705348,
  0.0539574950202411,
  0.06988297275082522,
  0.08484402989792057,
  0.09780022159910191,
  0.1078127068797125,
  0.1141395080395603,
  0.11630355895247008,
  0.1141395080395603,
  0.1078127068797125,
  0.09780022159910191,
  0.08484402989792057,
  0.06988297275082522,
  0.0539574950202411,
  0.03810413690705348,
  0.02326953079599063,
  0.010239915690916417,
  -0.00042540050885079954,
  -0.008429355871957256,
  -0.01372404332627383,
  -0.016483225448184834,
  -0.017072162448370464,
  -0.01598020193782839,
  -0.013737306504390728,
  -0.01087259298568141,
  -0.007868634890172817,
  -0.007824982398550288
};


void main (int argc, char **argv) {

	int i,j=0;
	int t,v_red,v_ir;
	int dummy;
	double a_red[FILTER_TAP_NUM],a_ir[FILTER_TAP_NUM];
	double sum_red, sum_ir;
	while (!feof(stdin)) {

                fscanf (stdin,"%d %d %d %d %d %d", &t, &v_red, &v_ir, &dummy, &dummy, &dummy);

		for (i = 1; i < FILTER_TAP_NUM; i++) {
			a_red[i-1] = a_red[i];
			a_ir[i-1] = a_ir[i];
		}
		a_red[FILTER_TAP_NUM-1] = (double)v_red;
		a_ir[FILTER_TAP_NUM-1] = (double)v_ir;

		sum_red = 0;
		sum_ir = 0;
		for (i = 0; i < FILTER_TAP_NUM; i++) {
			sum_red += a_red[i]*filter_taps[i];
			sum_ir += a_ir[i]*filter_taps[i];
		}
		fprintf (stdout,"%d %f %f\n", t, sum_red, sum_ir);
	}
}





