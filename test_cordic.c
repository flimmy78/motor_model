#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define pi		3.14159265358979323846	/* pi */
#define MY_PI 512

extern const int32_t cos_tb[1024];

void rotate_ccw(int32_t *v, int32_t a)
{
	int32_t x = v[0];
	int32_t y = v[1];

	v[0] = (x*cos_tb[a] 					- y*cos_tb[1023&(a+3*MY_PI/2)])/1024;
	v[1] = (x*cos_tb[1023&(a+3*MY_PI/2)] + y*cos_tb[a])/1024;
}

void rotate_cw(int32_t *v, int32_t a)
{
	int32_t x = v[0];
	int32_t y = v[1];

	v[0] = (x*cos_tb[a] 					+ y*cos_tb[1023&(a+3*MY_PI/2)])/1024;
	v[1] = (-x*cos_tb[1023&(a+3*MY_PI/2)] + y*cos_tb[a])/1024;
}

extern const int32_t cos_tb[1024];


int32_t getatan(int32_t *v)
{
	int32_t curang = MY_PI/4;
	int32_t sumang = 0;

	if(v[0] < 0) 
	{
		v[0] = -v[0];
		if(v[1] > 0) sumang = MY_PI/2;
		else sumang = 3*MY_PI/2;
	}else if(v[1] < 0) sumang = 2*MY_PI;
		
	while(v[1] && curang){
		
			printf("%d %d\n", curang, v[1]);
		
		if(v[1] > 0) {
			rotate_cw(v, curang);
			sumang += curang;
		}			
		else {
			rotate_ccw(v, curang);
			sumang -= curang;
		}		
		curang = curang/2;		
	}
	
	return sumang;	
}
/*
double cord_atan(int32_t *v)
{
	const double AngTable[] = {45, 26.565, 14.036, 7.125, 3.576, 1.790, 0.895, 0.448, 0.224, 0.112};
	const double kc[] = {0.7071, 0.6325, 0.6136,  0.6088, 0.6076, 0.6074, 0.6073, 0.6073, 0.6073, 0.6073};
	double SumAngle = 0; 
	int i = 0;
	int x, y, x1, y1;
	int ns = 0;

	x = abs(v[0]);
	y = v[1];

	for(i = 0; i < 10; i++)
	{		
		//printf("%.2f\n", SumAngle);
		ns++;
		
		x1 = x;
		y1 = y;
			
		if(y > 0){
			x = x1 + (y1 >> i); 
			y = y1 - (x1 >> i); 
			SumAngle = SumAngle + AngTable[i]; 
		}else{
			x = x1 - (y1 >> i); 
			y = y1 + (x1 >> i); 
			SumAngle = SumAngle - AngTable[i]; 
		}
		if(y == 0) break;
	}
	
	if(v[0] < 0) SumAngle = 180-SumAngle;		
	if(SumAngle < 0) SumAngle += 360;
	
	printf("ns=%d\n", ns);
	printf("angle=%.2f\n", SumAngle);
	printf("mag=%.2f\n", kc[ns-1]*x);
	
	return  SumAngle;
}
*/

int32_t cord_atan(int32_t *v)
{
	const int32_t AngTable[] = {128, 76, 40, 20, 10, 5, 3, 1};
	const int32_t kc[] = {724,  648, 628,  623,  623,  622,  622,  622};
	int32_t SumAngle = 0; 
	int i = 0;
	int x, y, x1, y1;
	int ns = 0;

	x = abs(v[0]);
	y = v[1];

	for(i = 0; i < 8; i++)
	{		
		ns++;
		
		x1 = x;
		y1 = y;
			
		if(y > 0){
			x = x1 + (y1 >> i); 
			y = y1 - (x1 >> i); 
			SumAngle = SumAngle + AngTable[i]; 
		}else{
			x = x1 - (y1 >> i); 
			y = y1 + (x1 >> i); 
			SumAngle = SumAngle - AngTable[i]; 
		}
		if(y == 0) break;
	}
	
	if(v[0] < 0) SumAngle = MY_PI-SumAngle;		
	if(SumAngle < 0) SumAngle += 2*MY_PI;
	
	printf("ns=%d\n", ns);
	printf("angle=%.2f\n", SumAngle*180.0/MY_PI);
	printf("mag=%d\n", (kc[ns-1]*x)/1024);
	
	return  SumAngle;
}

int main(int argc, char *argv[])
{
	double 	a[2];
	int32_t v[2];
	double phi;

	
	a[0] = atof(argv[1]);
	a[1] = atof(argv[2]);
	
	phi = atan(a[1]/a[0]);
		
	printf("x=%.2f y=%.2f phi=%.2f\n", a[0], a[1], phi*180/pi);
	
	v[0] = (int)a[0];
	v[1] = (int)a[1];

	//printf("angle=%.2f\n", 180.0*getatan(v)/MY_PI);
	//printf("angle=%.2f\n", cord_atan(v));
	cord_atan(v);
	
	/*
	printf("%d %d\n", v[0], v[1]);
	rotate_ccw(v, MY_PI/2);
	printf("%d %d\n", v[0], v[1]);
	*/
	
	
	return 0;
}


/*
  inline void svpwm(int32_t *abc, int32_t *dq, int32_t phase)
{
	int32_t mag;
	int32_t ang;
	cord_atan(dq, &ang, &mag);
	
	mag /= 1024;
	int32_t phi = 1023&(phase + ang);

	if(phi<MY_PI/3){
		int32_t r1 = mag*mysin(7*MY_PI/3-phi);
		int32_t r2 = mag*mysin(phi);	
		
		abc[0] = r1+r2;
		abc[1] = -r1+r2;
		abc[2] = -r1-r2;
	}
	else if(phi<2*MY_PI/3){
		phi -= MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi);
		int32_t r2 = mag*mysin(phi);		
		
		abc[0] = r1-r2;
		abc[1] = r1+r2;
		abc[2] = -r1-r2;		
	}
	else if(phi<MY_PI){
		phi -= 2*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi);
		int32_t r2 = mag*mysin(phi);		
		
		abc[0] = -r1-r2;
		abc[1] = r1+r2;
		abc[2] = -r1+r2;
	}
	else if(phi<4*MY_PI/3){
		phi -= 3*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi);
		int32_t r2 = mag*mysin(phi);		
		
		abc[0] = -r1-r2;
		abc[1] = r1-r2;
		abc[2] = r1+r2;		
	}		
	else if(phi<5*MY_PI/3){
		phi -= 4*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi);
		int32_t r2 = mag*mysin(phi);
		
		abc[0] = -r1+r2;
		abc[1] = -r1-r2;
		abc[2] = r1+r2;	
	}			
	else if(phi<2*MY_PI){
		phi -= 5*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi);
		int32_t r2 = mag*mysin(phi);		
		
		abc[0] = +r1+r2;
		abc[1] = -r1-r2;
		abc[2] = r1-r2;
	}
	
}
 */ 
