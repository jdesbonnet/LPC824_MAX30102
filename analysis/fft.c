/** 
 * FFT waterfall plot
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <fftw3.h>


void main (int argc, char **argv) {

	int i,j=0;
	int n = atoi(argv[1]);
	int t;

	// Got N real values. Result of FFT is
	// N/2+1 complex values due to hermitian redundancy.
	double in_red[n],in_ir[n];
	double mag_red,mag_ir;
	fftw_complex out_red[n/2+1], out_ir[n/2+1];
	fftw_plan plan_red, plan_ir;

	plan_red = fftw_plan_dft_r2c_1d(n, in_red, out_red, FFTW_ESTIMATE);
	plan_ir = fftw_plan_dft_r2c_1d(n, in_ir, out_ir, FFTW_ESTIMATE);

	while (!feof(stdin)) {
		for (i = 0; i < n; i++) {
			if (feof(stdin)) {
				return;
			}
			fscanf (stdin,"%d %lf %lf", &t, &in_red[i], &in_ir[i]);
			//fprintf (stderr,"%f %f\n", in_red[i],in_ir[i]);
		}
		fftw_execute(plan_red);
		fftw_execute(plan_ir);
		for (i = 0; i < (n/2)+1; i++) {
			mag_red = sqrt(out_red[i][0]*out_red[i][0] + out_red[i][1]*out_red[i][1]);
			mag_ir = sqrt(out_ir[i][0]*out_ir[i][0] + out_ir[i][1]*out_ir[i][1]);
			fprintf (stdout,"%d %d %f %f\n",i,j,mag_red,mag_ir);
		}
		j++;
	}
}





